#define CC_SCRATCH_VBS_ARE_DYNAMIC
#include "../Core.h"
#if CC_GFX_BACKEND == CC_GFX_BACKEND_METAL
#include "../_GraphicsBase.h"
#include "../Errors.h"
#include "../Window.h"
#include "../Bitmap.h"
#include "../Stream.h"
#include "../Logger.h"
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>
#include <dispatch/dispatch.h>
#include <stdatomic.h>


/* Metal backend for macOS
   Design notes (performance):
   - Up to MAX_FRAMES_IN_FLIGHT frames are encoded ahead of the GPU (throttled by a semaphore)
   - Static vertex/index buffers live in unified memory, and are written to directly by the CPU
   - Dynamic/scratch vertex data (UI, particles, etc) is sub-allocated from a per-frame ring buffer
   - Texture uploads are batched into a single blit command buffer per frame
   - Redundant state changes (pipeline, buffers, textures, etc) are filtered out before
      reaching the Metal command encoder
*/
#define MAX_FRAMES_IN_FLIGHT 3

static id<MTLDevice>       mtl_device;
static id<MTLCommandQueue> mtl_queue;
static id<MTLLibrary>      mtl_library;
static CAMetalLayer*       mtl_layer;
static id<MTLTexture>      mtl_depthTex;
static dispatch_semaphore_t mtl_frameSem;

/* Current frame state */
static id<MTLCommandBuffer>        mtl_cmdBuf;
static id<MTLRenderCommandEncoder> mtl_enc;
static id<CAMetalDrawable>         mtl_drawable;
static id<MTLTexture>              mtl_target;    /* colour texture being rendered to this frame */
static id<MTLTexture>              mtl_offscreen; /* used as render target when VSync is disabled */
static cc_bool                     mtl_direct;    /* whether rendering directly into a drawable this frame */
static int mtl_passes; /* number of render passes so far in this frame */
static int mtl_pendingClear;

/* Frame counters, used to know when GPU has finished with a resource */
static cc_uint32 mtl_frameId;
static _Atomic cc_uint32 mtl_completedFrameId;

/* Batched texture uploads */
static id<MTLCommandBuffer>      upl_cmdBuf;
static id<MTLBlitCommandEncoder> upl_enc;

static float mtl_scale = 1.0f; /* backing pixels per window point */
static int   mtl_width, mtl_height; /* drawable size in pixels */
static GfxResourceID white_square;
static float clearR, clearG, clearB;

void Gfx_InitForLayer(CAMetalLayer* layer) {
	mtl_layer  = [layer retain];
	mtl_device = MTLCreateSystemDefaultDevice();

	[layer setDevice:mtl_device];
	[layer setPixelFormat:MTLPixelFormatBGRA8Unorm];
	/* Needed so that screenshots can read back the drawable */
	[layer setFramebufferOnly:NO];
	[layer setOpaque:YES];
}


/*########################################################################################################################*
*---------------------------------------------------------Shaders---------------------------------------------------------*
*#########################################################################################################################*/
static const char* shader_source =
"#include <metal_stdlib>\n"
"using namespace metal;\n"
"constant bool FC_TEXOFFSET [[function_constant(0)]];\n"
"constant bool FC_ALPHATEST [[function_constant(1)]];\n"
"constant int  FC_FOG       [[function_constant(2)]];\n"
"struct VSUniforms { float4x4 mvp; float2 texOffset; };\n"
"struct FSUniforms { float4 fogCol; float fogEnd; float fogDensity; };\n"
"struct ColIn { float3 pos [[attribute(0)]]; float4 col [[attribute(1)]]; };\n"
"struct TexIn { float3 pos [[attribute(0)]]; float4 col [[attribute(1)]]; float2 uv [[attribute(2)]]; };\n"
"struct ColOut { float4 pos [[position]]; float4 col; float depth; };\n"
"struct TexOut { float4 pos [[position]]; float4 col; float2 uv; float depth; };\n"
"\n"
"vertex ColOut vs_coloured(ColIn in [[stage_in]], constant VSUniforms& u [[buffer(1)]]) {\n"
"  ColOut o; o.pos = u.mvp * float4(in.pos, 1.0); o.col = in.col; o.depth = o.pos.w; return o;\n"
"}\n"
"vertex TexOut vs_textured(TexIn in [[stage_in]], constant VSUniforms& u [[buffer(1)]]) {\n"
"  TexOut o; o.pos = u.mvp * float4(in.pos, 1.0); o.col = in.col; o.depth = o.pos.w;\n"
"  o.uv = FC_TEXOFFSET ? in.uv + u.texOffset : in.uv; return o;\n"
"}\n"
"static float4 ApplyFog(float4 col, float depth, constant FSUniforms& u) {\n"
"  if (FC_FOG == 1) { float f = clamp(depth * u.fogEnd, 0.0, 1.0); col.rgb = mix(col.rgb, u.fogCol.rgb, f); }\n"
"  if (FC_FOG == 2) { float f = clamp(exp(u.fogDensity * depth), 0.0, 1.0); col.rgb = mix(u.fogCol.rgb, col.rgb, f); }\n"
"  return col;\n"
"}\n"
"fragment float4 fs_coloured(ColOut in [[stage_in]], constant FSUniforms& u [[buffer(0)]]) {\n"
"  float4 col = in.col;\n"
"  if (FC_ALPHATEST && col.a < 0.5) discard_fragment();\n"
"  return ApplyFog(col, in.depth, u);\n"
"}\n"
"fragment float4 fs_textured(TexOut in [[stage_in]], constant FSUniforms& u [[buffer(0)]],\n"
"                            texture2d<float> tex [[texture(0)]], sampler smp [[sampler(0)]]) {\n"
"  float4 col = tex.sample(smp, in.uv) * in.col;\n"
"  if (FC_ALPHATEST && col.a < 0.5) discard_fragment();\n"
"  return ApplyFog(col, in.depth, u);\n"
"}\n";

static void CompileShaders(void) {
	NSError* err = nil;
	NSString* src = [NSString stringWithUTF8String:shader_source];
	MTLCompileOptions* opts = [[MTLCompileOptions alloc] init];

	mtl_library = [mtl_device newLibraryWithSource:src options:opts error:&err];
	[opts release];
	if (mtl_library) return;

	if (err) Platform_LogConst([[err localizedDescription] UTF8String]);
	Process_Abort("Failed to compile Metal shaders");
}


/*########################################################################################################################*
*---------------------------------------------------------Ring buffer-----------------------------------------------------*
*#########################################################################################################################*/
/* Per-frame ring of CPU-written memory, used for dynamic vertex data */
static struct FrameRing {
	id<MTLBuffer> buf;
	int size, used;
} rings[MAX_FRAMES_IN_FLIGHT];
static struct FrameRing* curRing;

#define RING_ALIGN 256
#define RING_INITIAL_SIZE (4 * 1024 * 1024)

static void BeginFrameIfNeeded(void);
static void DeferRelease(id obj, cc_uint32 frame);

/* Allocates memory that is valid for use by the GPU in the current frame */
static void* Ring_Alloc(int size, id<MTLBuffer>* buf, int* offset) {
	struct FrameRing* r;
	int newSize;
	BeginFrameIfNeeded();
	r = curRing;

	if (r->used + size > r->size) {
		/* Not enough space, so replace with a larger buffer */
		newSize = max(r->size * 2, size + RING_ALIGN);
		DeferRelease(r->buf, mtl_frameId);
		r->buf  = [mtl_device newBufferWithLength:newSize options:MTLResourceStorageModeShared | MTLResourceCPUCacheModeWriteCombined];
		if (!r->buf) Process_Abort("Out of memory for Metal ring buffer");
		r->size = newSize;
		r->used = 0;
	}

	*buf    = r->buf;
	*offset = r->used;
	r->used = (r->used + size + (RING_ALIGN - 1)) & ~(RING_ALIGN - 1);
	return (cc_uint8*)[r->buf contents] + *offset;
}


/*########################################################################################################################*
*-----------------------------------------------------------Frames--------------------------------------------------------*
*#########################################################################################################################*/
static void InvalidateEncoderState(void);
static void EndUploads(void);
static void DeferRelease(id obj, cc_uint32 frame);
static void ProcessDeferredReleases(void);
static void Pool_ProcessPending(void);

/* Starts a new frame (command buffer), waiting for the GPU if too many frames are in flight */
static void BeginFrameIfNeeded(void) {
	if (mtl_cmdBuf) return;
	dispatch_semaphore_wait(mtl_frameSem, DISPATCH_TIME_FOREVER);

	mtl_frameId++;
	/* Unretained references avoids the cost of retaining every resource used in the frame */
	/*  (so resources must be kept alive until GPU is done with them - see DeferRelease) */
	mtl_cmdBuf = [[mtl_queue commandBufferWithUnretainedReferences] retain];
	ProcessDeferredReleases();
	Pool_ProcessPending();
	mtl_passes = 0;

	curRing = &rings[mtl_frameId % MAX_FRAMES_IN_FLIGHT];
	curRing->used = 0;
}

/* With VSync, frames are rendered directly into the layer's drawable. */
/* However, CAMetalLayer in a window can never provide drawables faster than the */
/*  display refresh rate (nextDrawable blocks), even with displaySyncEnabled = NO. */
/* So without VSync, frames are instead rendered into an offscreen texture, and the */
/*  latest frame is copied to a drawable and presented from another thread whenever */
/*  a drawable becomes available (so frames rendered in the meantime are simply not */
/*  displayed - which matches what the OpenGL backends do without VSync) */
static void SelectRenderTarget(void) {
	MTLTextureDescriptor* desc;
	mtl_direct = gfx_vsync;

	if (mtl_direct) {
		/* Acquire drawable as late as possible, as it may block */
		mtl_drawable = [[mtl_layer nextDrawable] retain];
		if (!mtl_drawable) Process_Abort("Failed to get Metal drawable");
		mtl_target = [mtl_drawable texture];
		return;
	}

	if (!mtl_offscreen || [mtl_offscreen width] != mtl_width || [mtl_offscreen height] != mtl_height) {
		DeferRelease(mtl_offscreen, mtl_frameId);
		desc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
					width:mtl_width height:mtl_height mipmapped:NO];
		desc.storageMode = MTLStorageModePrivate;
		desc.usage       = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
		mtl_offscreen    = [mtl_device newTextureWithDescriptor:desc];
		if (!mtl_offscreen) Process_Abort("Failed to create offscreen texture");
	}
	mtl_target = mtl_offscreen;
}

/* Presents the latest frame from the offscreen render target. */
/* Acquiring a drawable may block until the compositor releases one, so this */
/*  is done on a separate thread, to avoid stalling the main thread. */
/* NOTE: Command buffers execute in order of submission, so the copy always */
/*  sees a fully rendered frame (whichever frame was most recently submitted) */
static dispatch_queue_t mtl_presentQueue;
static _Atomic cc_bool  mtl_presenting;

static void PresentOffscreen(void) {
	id<MTLTexture> src;
	if (atomic_load(&mtl_presenting)) return; /* previous present still waiting for a drawable */
	atomic_store(&mtl_presenting, true);

	if (!mtl_presentQueue) {
		mtl_presentQueue = dispatch_queue_create("ClassiCube.present", DISPATCH_QUEUE_SERIAL);
	}
	src = [mtl_offscreen retain];

	dispatch_async(mtl_presentQueue, ^{
		@autoreleasepool {
			id<CAMetalDrawable> drawable = [mtl_layer nextDrawable];
			id<MTLTexture> dst = [drawable texture];

			if (drawable && [dst width] == [src width] && [dst height] == [src height]) {
				id<MTLCommandBuffer> cmd = [mtl_queue commandBuffer];
				id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
				[blit copyFromTexture:src toTexture:dst];
				[blit endEncoding];
				[cmd presentDrawable:drawable];
				[cmd commit];
			}
			[src release];
			atomic_store(&mtl_presenting, false);
		}
	});
}

/* Ensures that a render command encoder is active */
static void BeginRenderPass(void) {
	MTLRenderPassDescriptor* desc;
	if (mtl_enc) return;
	BeginFrameIfNeeded();
	if (!mtl_depthTex) Gfx_OnWindowResize(Game.Width, Game.Height);

	if (!mtl_target) SelectRenderTarget();

	desc = [MTLRenderPassDescriptor renderPassDescriptor];
	desc.colorAttachments[0].texture     = mtl_target;
	desc.colorAttachments[0].clearColor  = MTLClearColorMake(clearR, clearG, clearB, 1.0);
	desc.colorAttachments[0].storeAction = MTLStoreActionStore;

	if (mtl_pendingClear & GFX_BUFFER_COLOR) {
		desc.colorAttachments[0].loadAction = MTLLoadActionClear;
	} else {
		desc.colorAttachments[0].loadAction = mtl_passes ? MTLLoadActionLoad : MTLLoadActionDontCare;
	}

	desc.depthAttachment.texture     = mtl_depthTex;
	desc.depthAttachment.clearDepth  = 1.0;
	desc.depthAttachment.storeAction = MTLStoreActionUnknown; /* decided when pass ends */
	if ((mtl_pendingClear & GFX_BUFFER_DEPTH) || !mtl_passes) {
		desc.depthAttachment.loadAction = MTLLoadActionClear;
	} else {
		desc.depthAttachment.loadAction = MTLLoadActionLoad;
	}

	mtl_pendingClear = 0;
	mtl_enc = [[mtl_cmdBuf renderCommandEncoderWithDescriptor:desc] retain];
	mtl_passes++;
	InvalidateEncoderState();
}

/* Ends the current render pass, keeping the contents of all attachments */
static void EndRenderPass(cc_bool keepDepth) {
	if (!mtl_enc) return;
	[mtl_enc setDepthStoreAction:keepDepth ? MTLStoreActionStore : MTLStoreActionDontCare];
	[mtl_enc endEncoding];
	[mtl_enc release];
	mtl_enc = nil;
}

void Gfx_BeginFrame(void) { }

void Gfx_ClearBuffers(GfxBuffers buffers) {
	/* Clearing in the middle of a render pass requires starting a new render pass */
	EndRenderPass(true);
	mtl_pendingClear |= buffers;
}

void Gfx_ClearColor(PackedCol color) {
	clearR = PackedCol_R(color) / 255.0f;
	clearG = PackedCol_G(color) / 255.0f;
	clearB = PackedCol_B(color) / 255.0f;
}

void Gfx_EndFrame(void) {
	cc_uint32 frameId;
	BeginRenderPass(); /* in case nothing was drawn, still need to clear */
	EndRenderPass(false);
	EndUploads();

	frameId = mtl_frameId;
	[mtl_cmdBuf addCompletedHandler:^(id<MTLCommandBuffer> buf) {
		cc_uint32 cur = atomic_load(&mtl_completedFrameId);
		/* Frames should complete in order, but be safe */
		while (cur < frameId && !atomic_compare_exchange_weak(&mtl_completedFrameId, &cur, frameId)) { }
		dispatch_semaphore_signal(mtl_frameSem);
	}];

	if (mtl_direct) [mtl_cmdBuf presentDrawable:mtl_drawable];
	[mtl_cmdBuf commit];
	if (!mtl_direct) PresentOffscreen();

	[mtl_cmdBuf release];   mtl_cmdBuf   = nil;
	[mtl_drawable release]; mtl_drawable = nil;
	mtl_target = nil;
}

static cc_bool GPUFinishedWith(cc_uint32 frameId) {
	return frameId <= atomic_load(&mtl_completedFrameId);
}

static void WaitForGPU(void) {
	int i;
	/* Submit the partially completed frame (if any), so it releases its frame slot */
	if (mtl_cmdBuf) Gfx_EndFrame();
	/* Acquire then release all frame slots, so all frames have finished */
	for (i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) dispatch_semaphore_wait(mtl_frameSem, DISPATCH_TIME_FOREVER);
	for (i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) dispatch_semaphore_signal(mtl_frameSem);
}


/*########################################################################################################################*
*---------------------------------------------------------General---------------------------------------------------------*
*#########################################################################################################################*/
static void Gfx_RestoreState(void) {
	struct Bitmap bmp;
	BitmapCol pixels[1] = { BITMAPCOLOR_WHITE };
	InitDefaultResources();

	/* 1x1 dummy white texture */
	Bitmap_Init(bmp, 1, 1, pixels);
	white_square = Gfx_CreateTexture(&bmp, 0, false);
}

static void Gfx_FreeState(void) {
	FreeDefaultResources();
	Gfx_DeleteTexture(&white_square);
}

void Gfx_Create(void) {
	int i;
	if (!mtl_layer) Process_Abort("Metal layer not created");

	Gfx.MaxTexWidth  = 16384;
	Gfx.MaxTexHeight = 16384;
	Gfx.Created      = true;
	Gfx.BackendType  = CC_GFX_BACKEND_METAL;
	customMipmapsLevels = true;

	mtl_queue    = [mtl_device newCommandQueue];
	mtl_frameSem = dispatch_semaphore_create(MAX_FRAMES_IN_FLIGHT);
	CompileShaders();

	for (i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
	{
		rings[i].size = RING_INITIAL_SIZE;
		rings[i].buf  = [mtl_device newBufferWithLength:RING_INITIAL_SIZE
						options:MTLResourceStorageModeShared | MTLResourceCPUCacheModeWriteCombined];
	}
	Gfx_SetVSync(gfx_vsync);
}

void Gfx_Free(void) {
	WaitForGPU();
	Gfx_FreeState();
}

cc_bool Gfx_TryRestoreContext(void) { return true; }
cc_bool Gfx_WarnIfNecessary(void) { return false; }
cc_bool Gfx_GetUIOptions(struct MenuOptionsScreen* s) { return false; }

void Gfx_GetApiInfo(cc_string* info) {
	const char* name = [[mtl_device name] UTF8String];
	String_AppendConst(info, "-- Using Metal --\n");
	if (name) String_Format1(info, "Device: %c\n", name);
	PrintMaxTextureInfo(info);
}

void Gfx_SetVSync(cc_bool vsync) {
	gfx_vsync = vsync;
	if (!mtl_layer) return;
	if ([mtl_layer respondsToSelector:@selector(setDisplaySyncEnabled:)]) {
		[mtl_layer setDisplaySyncEnabled:vsync];
	}

}

void Gfx_OnWindowResize(int width, int height) {
	MTLTextureDescriptor* desc;
	mtl_scale  = [mtl_layer contentsScale];
	if (mtl_scale <= 0.0f) mtl_scale = 1.0f;
	mtl_width  = max(1, (int)(width  * mtl_scale + 0.5f));
	mtl_height = max(1, (int)(height * mtl_scale + 0.5f));

	/* The current frame must not continue using the old depth texture */
	EndRenderPass(false);
	[mtl_layer setDrawableSize:CGSizeMake(mtl_width, mtl_height)];

	desc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
				width:mtl_width height:mtl_height mipmapped:NO];
	desc.storageMode = MTLStorageModePrivate;
	desc.usage       = MTLTextureUsageRenderTarget;

	DeferRelease(mtl_depthTex, mtl_frameId);
	mtl_depthTex = [mtl_device newTextureWithDescriptor:desc];
	if (!mtl_depthTex) Process_Abort("Failed to create depth texture");

	Gfx_SetViewport(0, 0, width, height);
	Gfx_SetScissor (0, 0, width, height);
}


/*########################################################################################################################*
*---------------------------------------------------------Textures--------------------------------------------------------*
*#########################################################################################################################*/
struct MetalTexture { id<MTLTexture> tex; int sampler; cc_uint32 usedFrame; };
static struct MetalTexture* boundTex;
#define SAMPLER_BILINEAR 1
#define SAMPLER_MIPMAPS  2
static id<MTLSamplerState> samplers[4];

static id<MTLSamplerState> GetSampler(int idx) {
	MTLSamplerDescriptor* desc;
	if (samplers[idx]) return samplers[idx];

	desc = [[MTLSamplerDescriptor alloc] init];
	desc.minFilter    = (idx & SAMPLER_BILINEAR) ? MTLSamplerMinMagFilterLinear : MTLSamplerMinMagFilterNearest;
	desc.magFilter    = desc.minFilter;
	/* Same as GL_NEAREST_MIPMAP_LINEAR in the OpenGL backends */
	desc.mipFilter    = (idx & SAMPLER_MIPMAPS)  ? MTLSamplerMipFilterLinear : MTLSamplerMipFilterNotMipmapped;
	desc.sAddressMode = MTLSamplerAddressModeRepeat;
	desc.tAddressMode = MTLSamplerAddressModeRepeat;

	samplers[idx] = [mtl_device newSamplerStateWithDescriptor:desc];
	[desc release];
	return samplers[idx];
}

static void BeginUploads(void) {
	if (upl_enc) return;
	upl_cmdBuf = [[mtl_queue commandBuffer] retain];
	upl_enc    = [[upl_cmdBuf blitCommandEncoder] retain];
}

/* Uploads are committed before the frame's command buffer, so they're visible to all draws in the frame */
static void EndUploads(void) {
	if (!upl_enc) return;
	[upl_enc endEncoding];
	[upl_cmdBuf commit];

	[upl_enc release];    upl_enc    = nil;
	[upl_cmdBuf release]; upl_cmdBuf = nil;
}

static void UploadRegion(id<MTLTexture> tex, int lvl, int x, int y, int width, int height,
						BitmapCol* pixels, int rowWidth) {
	int rowBytes = width * 4, y2;
	id<MTLBuffer> staging;
	cc_uint8* dst;

	staging = [mtl_device newBufferWithLength:rowBytes * height options:MTLResourceStorageModeShared];
	if (!staging) Process_Abort("Out of memory for texture upload");
	dst = (cc_uint8*)[staging contents];

	for (y2 = 0; y2 < height; y2++)
	{
		Mem_Copy(dst + y2 * rowBytes, pixels + y2 * rowWidth, rowBytes);
	}

	BeginUploads();
	[upl_enc copyFromBuffer:staging sourceOffset:0 sourceBytesPerRow:rowBytes sourceBytesPerImage:rowBytes * height
				sourceSize:MTLSizeMake(width, height, 1) toTexture:tex destinationSlice:0
				destinationLevel:lvl destinationOrigin:MTLOriginMake(x, y, 0)];
	[staging release]; /* retained by the command buffer */
}

static void UploadMipmaps(id<MTLTexture> tex, int x, int y, struct Bitmap* bmp, int rowWidth) {
	BitmapCol* prev = bmp->scan0;
	BitmapCol* cur;
	int lvls = CalcMipmapsLevels(bmp->width, bmp->height);
	int lvl, width = bmp->width, height = bmp->height;

	for (lvl = 1; lvl <= lvls; lvl++)
	{
		x /= 2; y /= 2;
		if (width > 1)  width  /= 2;
		if (height > 1) height /= 2;

		cur = (BitmapCol*)Mem_Alloc(width * height, BITMAPCOLOR_SIZE, "mipmaps");
		GenMipmaps(width, height, cur, prev, rowWidth);
		UploadRegion(tex, lvl, x, y, width, height, cur, width);

		if (prev != bmp->scan0) Mem_Free(prev);
		prev     = cur;
		rowWidth = width;
	}
	if (prev != bmp->scan0) Mem_Free(prev);
}

GfxResourceID Gfx_AllocTexture(struct Bitmap* bmp, int rowWidth, cc_uint8 flags, cc_bool mipmaps) {
	struct MetalTexture* t;
	MTLTextureDescriptor* desc;
	id<MTLTexture> tex;

	desc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
				width:bmp->width height:bmp->height mipmapped:NO];
	if (mipmaps) desc.mipmapLevelCount = CalcMipmapsLevels(bmp->width, bmp->height) + 1;
	desc.storageMode = MTLStorageModePrivate;
	desc.usage       = MTLTextureUsageShaderRead;

	tex = [mtl_device newTextureWithDescriptor:desc];
	if (!tex) return NULL;

	t = (struct MetalTexture*)Mem_TryAllocCleared(1, sizeof(struct MetalTexture));
	if (!t) { [tex release]; return NULL; }
	t->tex     = tex;
	t->sampler = ((flags & TEXTURE_FLAG_BILINEAR) ? SAMPLER_BILINEAR : 0) | (mipmaps ? SAMPLER_MIPMAPS : 0);

	UploadRegion(tex, 0, 0, 0, bmp->width, bmp->height, bmp->scan0, rowWidth);
	if (mipmaps) UploadMipmaps(tex, 0, 0, bmp, rowWidth);
	return t;
}

void Gfx_UpdateTexture(GfxResourceID texId, int x, int y, struct Bitmap* part, int rowWidth, cc_bool mipmaps) {
	struct MetalTexture* t = (struct MetalTexture*)texId;
	UploadRegion(t->tex, 0, x, y, part->width, part->height, part->scan0, rowWidth);
	if (mipmaps && [t->tex mipmapLevelCount] > 1) UploadMipmaps(t->tex, x, y, part, rowWidth);
}

void Gfx_DeleteTexture(GfxResourceID* texId) {
	struct MetalTexture* t = (struct MetalTexture*)(*texId);
	if (!t) return;

	if (boundTex == t) boundTex = NULL;
	DeferRelease(t->tex, t->usedFrame);
	Mem_Free(t);
	*texId = NULL;
}

void Gfx_EnableMipmaps(void)  { }
void Gfx_DisableMipmaps(void) { }


/*########################################################################################################################*
*---------------------------------------------------------Encoder state---------------------------------------------------*
*#########################################################################################################################*/
#define DIRTY_VIEWPORT  (1 << 0)
#define DIRTY_SCISSOR   (1 << 1)
#define DIRTY_PIPELINE  (1 << 2)
#define DIRTY_DEPTH     (1 << 3)
#define DIRTY_VS_UNI    (1 << 4)
#define DIRTY_FS_UNI    (1 << 5)
#define DIRTY_CULL      (1 << 6)
#define DIRTY_TEXTURE   (1 << 7)
#define DIRTY_ALL       0xFF
static int dirty;

static MTLViewport    viewport;
static MTLScissorRect scissor;
static cc_bool cullEnabled;

/* Uniforms */
static struct VSUniforms { struct Matrix mvp; float texX, texY, pad[2]; } vs_uni;
static struct FSUniforms { float fogR, fogG, fogB, fogA, fogEnd, fogDensity, pad[2]; } fs_uni;
static struct Matrix _view, _proj;
static cc_bool texOffsetting;

/* Bound resources */
static struct MetalVb*      boundVb;
static id<MTLBuffer>        boundIb;

/* What's actually set on the encoder (to filter redundant changes) */
static id<MTLRenderPipelineState> enc_pipeline;
static id<MTLBuffer>              enc_vb;
static int                        enc_vbOffset;
static id<MTLTexture>             enc_tex;
static id<MTLSamplerState>        enc_sampler;

static void InvalidateEncoderState(void) {
	dirty        = DIRTY_ALL;
	enc_pipeline = nil;
	enc_vb       = nil;
	enc_vbOffset = -1;
	enc_tex      = nil;
	enc_sampler  = nil;
}


/*########################################################################################################################*
*-----------------------------------------------------State management----------------------------------------------------*
*#########################################################################################################################*/
static void UpdateFogUniforms(void) {
	fs_uni.fogR = PackedCol_R(gfx_fogColor) / 255.0f;
	fs_uni.fogG = PackedCol_G(gfx_fogColor) / 255.0f;
	fs_uni.fogB = PackedCol_B(gfx_fogColor) / 255.0f;
	fs_uni.fogA = 1.0f;
	fs_uni.fogEnd     = 1.0f / gfx_fogEnd;
	/* The equation for EXP mode is exp(-density * z), so just negate density here */
	fs_uni.fogDensity = -gfx_fogDensity;
	dirty |= DIRTY_FS_UNI;
}

void Gfx_SetFog(cc_bool enabled)         { gfx_fogEnabled = enabled; dirty |= DIRTY_PIPELINE; }
static void SetFogColor(PackedCol color) { UpdateFogUniforms(); }
static void SetFogDensity(float value)   { UpdateFogUniforms(); }
static void SetFogEnd(float value)       { UpdateFogUniforms(); }
static void SetFogMode(FogFunc func)     { dirty |= DIRTY_PIPELINE; }

void Gfx_SetFaceCulling(cc_bool enabled) {
	if (cullEnabled == enabled) return;
	cullEnabled = enabled;
	dirty |= DIRTY_CULL;
}

static void SetAlphaTest(cc_bool enabled)  { dirty |= DIRTY_PIPELINE; }
static void SetAlphaBlend(cc_bool enabled) { dirty |= DIRTY_PIPELINE; }
void Gfx_SetAlphaArgBlend(cc_bool enabled) { }

static void SetDepthTest(cc_bool enabled)  { dirty |= DIRTY_DEPTH; }
static void SetDepthWrite(cc_bool enabled) { dirty |= DIRTY_DEPTH; }
/* NOTE: Can't rely on gfx_colorMask, as depth only rendering changes the */
/*  effective colour write mask without changing gfx_colorMask */
static int mtl_colorMask = 0xF;
static void SetColorWrite(cc_bool r, cc_bool g, cc_bool b, cc_bool a) {
	mtl_colorMask = (r ? 1 : 0) | (g ? 2 : 0) | (b ? 4 : 0) | (a ? 8 : 0);
	dirty |= DIRTY_PIPELINE;
}

void Gfx_DepthOnlyRendering(cc_bool depthOnly) {
	DefaultDepthOnlyRendering(depthOnly);
}

void Gfx_BindTexture(GfxResourceID texId) {
	struct MetalTexture* t = (struct MetalTexture*)texId;
	if (!t) t = (struct MetalTexture*)white_square;
	if (t == boundTex) return;

	boundTex = t;
	dirty |= DIRTY_TEXTURE;
}

void Gfx_SetViewport(int x, int y, int w, int h) {
	viewport.originX = x * mtl_scale;
	viewport.originY = y * mtl_scale;
	viewport.width   = w * mtl_scale;
	viewport.height  = h * mtl_scale;
	viewport.znear   = 0.0;
	viewport.zfar    = 1.0;
	dirty |= DIRTY_VIEWPORT;
}

void Gfx_SetScissor(int x, int y, int w, int h) {
	int x1 = (int)(x * mtl_scale), y1 = (int)(y * mtl_scale);
	int x2 = (int)((x + w) * mtl_scale), y2 = (int)((y + h) * mtl_scale);

	x1 = max(0, min(x1, mtl_width));  x2 = max(x1, min(x2, mtl_width));
	y1 = max(0, min(y1, mtl_height)); y2 = max(y1, min(y2, mtl_height));
	scissor.x = x1; scissor.width  = x2 - x1;
	scissor.y = y1; scissor.height = y2 - y1;
	dirty |= DIRTY_SCISSOR;
}


/*########################################################################################################################*
*-------------------------------------------------------Depth states------------------------------------------------------*
*#########################################################################################################################*/
static id<MTLDepthStencilState> depthStates[4];

static id<MTLDepthStencilState> GetDepthState(void) {
	int idx = (gfx_depthTest ? 1 : 0) | (gfx_depthWrite ? 2 : 0);
	MTLDepthStencilDescriptor* desc;
	if (depthStates[idx]) return depthStates[idx];

	desc = [[MTLDepthStencilDescriptor alloc] init];
	desc.depthWriteEnabled    = gfx_depthWrite ? YES : NO;
	desc.depthCompareFunction = gfx_depthTest  ? MTLCompareFunctionLessEqual : MTLCompareFunctionAlways;

	depthStates[idx] = [mtl_device newDepthStencilStateWithDescriptor:desc];
	[desc release];
	return depthStates[idx];
}


/*########################################################################################################################*
*----------------------------------------------------------Pipelines------------------------------------------------------*
*#########################################################################################################################*/
#define PIPE_TEXTURED   (1 << 0)
#define PIPE_TEXOFFSET  (1 << 1)
#define PIPE_ALPHATEST  (1 << 2)
#define PIPE_BLEND      (1 << 3)
#define PIPE_FOG_SHIFT  4 /* 2 bits: 0 = none, 1 = linear, 2 = exp */
#define PIPE_MASK_SHIFT 6 /* 4 bits: colour write mask */
#define PIPE_COUNT      (1 << 10)
static id<MTLRenderPipelineState> pipelines[PIPE_COUNT];

static void FillVertexDescriptor(MTLVertexDescriptor* desc, cc_bool textured) {
	desc.attributes[0].format      = MTLVertexFormatFloat3;
	desc.attributes[0].offset      = 0;
	desc.attributes[0].bufferIndex = 0;

	desc.attributes[1].format      = MTLVertexFormatUChar4Normalized;
	desc.attributes[1].offset      = 12;
	desc.attributes[1].bufferIndex = 0;

	if (textured) {
		desc.attributes[2].format      = MTLVertexFormatFloat2;
		desc.attributes[2].offset      = 16;
		desc.attributes[2].bufferIndex = 0;
	}
	desc.layouts[0].stride       = textured ? SIZEOF_VERTEX_TEXTURED : SIZEOF_VERTEX_COLOURED;
	desc.layouts[0].stepRate     = 1;
	desc.layouts[0].stepFunction = MTLVertexStepFunctionPerVertex;
}

static id<MTLFunction> MakeFunction(const char* name, int idx) {
	MTLFunctionConstantValues* consts = [[MTLFunctionConstantValues alloc] init];
	bool texOffset = (idx & PIPE_TEXOFFSET) != 0;
	bool alphaTest = (idx & PIPE_ALPHATEST) != 0;
	int  fog       = (idx >> PIPE_FOG_SHIFT) & 3;
	NSError* err   = nil;
	id<MTLFunction> func;

	[consts setConstantValue:&texOffset type:MTLDataTypeBool atIndex:0];
	[consts setConstantValue:&alphaTest type:MTLDataTypeBool atIndex:1];
	[consts setConstantValue:&fog       type:MTLDataTypeInt  atIndex:2];

	func = [mtl_library newFunctionWithName:[NSString stringWithUTF8String:name] constantValues:consts error:&err];
	[consts release];
	if (!func) Process_Abort("Failed to create Metal shader function");
	return func;
}

static id<MTLRenderPipelineState> BuildPipeline(int idx) {
	cc_bool textured = (idx & PIPE_TEXTURED) != 0;
	int mask = (idx >> PIPE_MASK_SHIFT) & 0xF;
	MTLRenderPipelineDescriptor* desc = [[MTLRenderPipelineDescriptor alloc] init];
	MTLRenderPipelineColorAttachmentDescriptor* fb = desc.colorAttachments[0];
	MTLVertexDescriptor* vdesc = [MTLVertexDescriptor vertexDescriptor];
	id<MTLRenderPipelineState> pso;
	NSError* err = nil;

	FillVertexDescriptor(vdesc, textured);
	desc.vertexDescriptor = vdesc;
	desc.vertexFunction   = MakeFunction(textured ? "vs_textured" : "vs_coloured", idx);
	desc.fragmentFunction = MakeFunction(textured ? "fs_textured" : "fs_coloured", idx);

	fb.pixelFormat     = MTLPixelFormatBGRA8Unorm;
	fb.blendingEnabled = (idx & PIPE_BLEND) ? YES : NO;
	fb.sourceRGBBlendFactor        = MTLBlendFactorSourceAlpha;
	fb.sourceAlphaBlendFactor      = MTLBlendFactorSourceAlpha;
	fb.destinationRGBBlendFactor   = MTLBlendFactorOneMinusSourceAlpha;
	fb.destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
	fb.writeMask = ((mask & 1) ? MTLColorWriteMaskRed   : 0) | ((mask & 2) ? MTLColorWriteMaskGreen : 0) |
				   ((mask & 4) ? MTLColorWriteMaskBlue  : 0) | ((mask & 8) ? MTLColorWriteMaskAlpha : 0);
	desc.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;

	pso = [mtl_device newRenderPipelineStateWithDescriptor:desc error:&err];
	[desc.vertexFunction release];
	[desc.fragmentFunction release];
	[desc release];

	if (!pso) {
		if (err) Platform_LogConst([[err localizedDescription] UTF8String]);
		Process_Abort("Failed to create Metal pipeline");
	}
	return pso;
}

static id<MTLRenderPipelineState> GetPipeline(void) {
	int fog = 0, idx;
	if (gfx_fogEnabled) fog = gfx_fogMode >= 1 ? 2 : 1;

	idx = (gfx_format == VERTEX_FORMAT_TEXTURED ? PIPE_TEXTURED : 0)
		| (texOffsetting  ? PIPE_TEXOFFSET : 0)
		| (gfx_alphaTest  ? PIPE_ALPHATEST : 0)
		| (gfx_alphaBlend ? PIPE_BLEND     : 0)
		| (fog << PIPE_FOG_SHIFT)
		| (mtl_colorMask << PIPE_MASK_SHIFT);
	/* Texture offset is only used with textured vertices */
	if (!(idx & PIPE_TEXTURED)) idx &= ~PIPE_TEXOFFSET;

	if (!pipelines[idx]) pipelines[idx] = BuildPipeline(idx);
	return pipelines[idx];
}

static void SetVertexFormat(VertexFormat fmt) { dirty |= DIRTY_PIPELINE; }


/*########################################################################################################################*
*---------------------------------------------------------Matrices--------------------------------------------------------*
*#########################################################################################################################*/
void Gfx_LoadMatrix(MatrixType type, const struct Matrix* matrix) {
	if (type == MATRIX_VIEW) _view = *matrix;
	if (type == MATRIX_PROJ) _proj = *matrix;

	Matrix_Mul(&vs_uni.mvp, &_view, &_proj);
	dirty |= DIRTY_VS_UNI;
}

void Gfx_LoadMVP(const struct Matrix* view, const struct Matrix* proj, struct Matrix* mvp) {
	_view = *view;
	_proj = *proj;

	Matrix_Mul(mvp, view, proj);
	vs_uni.mvp = *mvp;
	dirty |= DIRTY_VS_UNI;
}

void Gfx_EnableTextureOffset(float x, float y) {
	vs_uni.texX   = x;
	vs_uni.texY   = y;
	texOffsetting = true;
	dirty |= DIRTY_PIPELINE | DIRTY_VS_UNI;
}

void Gfx_DisableTextureOffset(void) {
	texOffsetting = false;
	dirty |= DIRTY_PIPELINE;
}

void Gfx_CalcOrthoMatrix(struct Matrix* matrix, float width, float height, float zNear, float zFar) {
	/* Source https://learn.microsoft.com/en-us/windows/win32/direct3d9/d3dxmatrixorthooffcenterrh */
	/*   The simplified calculation below uses: L = 0, R = width, T = 0, B = height */
	/* NOTE: This calculation is shared with Direct3D 11 backend */
	*matrix = Matrix_Identity;

	matrix->row1.x =  2.0f / width;
	matrix->row2.y = -2.0f / height;
	matrix->row3.z =  1.0f / (zNear - zFar);

	matrix->row4.x = -1.0f;
	matrix->row4.y =  1.0f;
	matrix->row4.z = zNear / (zNear - zFar);
}

static float Cotangent(float x) { return Math_CosF(x) / Math_SinF(x); }
void Gfx_CalcPerspectiveMatrix(struct Matrix* matrix, float fov, float aspect, float zFar) {
	float zNear = 0.1f;
	/* Source https://learn.microsoft.com/en-us/windows/win32/direct3d9/d3dxmatrixperspectivefovrh */
	/* NOTE: This calculation is shared with Direct3D 11 backend */
	float c = Cotangent(0.5f * fov);
	*matrix = Matrix_Identity;

	matrix->row1.x =  c / aspect;
	matrix->row2.y =  c;
	matrix->row3.z = zFar / (zNear - zFar);
	matrix->row3.w = -1.0f;
	matrix->row4.z = (zNear * zFar) / (zNear - zFar);
	matrix->row4.w =  0.0f;
}


/*########################################################################################################################*
*-------------------------------------------------------Index buffers-----------------------------------------------------*
*#########################################################################################################################*/
GfxResourceID Gfx_CreateIb2(int count, Gfx_FillIBFunc fillFunc, void* obj) {
	id<MTLBuffer> buf = [mtl_device newBufferWithLength:count * 2
						options:MTLResourceStorageModeShared | MTLResourceHazardTrackingModeUntracked];
	if (!buf) return NULL;

	fillFunc((cc_uint16*)[buf contents], count, obj);
	return buf;
}

void Gfx_BindIb(GfxResourceID ib) { boundIb = (id<MTLBuffer>)ib; }

void Gfx_DeleteIb(GfxResourceID* ib) {
	id<MTLBuffer> buf = (id<MTLBuffer>)(*ib);
	if (buf == boundIb) boundIb = nil;
	DeferRelease(buf, mtl_frameId);
	*ib = NULL;
}


/*########################################################################################################################*
*------------------------------------------------------Vertex buffers-----------------------------------------------------*
*#########################################################################################################################*/
struct MetalVb {
	id<MTLBuffer> buf;    /* static: owned buffer. dynamic: ring buffer holding latest data */
	int offset;           /* offset of vertex data in buf */
	int size;             /* capacity in bytes */
	cc_uint32 usedFrame;  /* last frame that this buffer was used for drawing in */
	cc_bool dynamic;
	/* Dynamic vertex buffers only */
	void* data;           /* CPU copy of latest contents */
	int dataSize;         /* size of latest contents */
	cc_uint32 dataFrame;  /* frame that latest contents were copied into the ring buffer for */
};

#define STATIC_VB_OPTIONS (MTLResourceStorageModeShared | MTLResourceHazardTrackingModeUntracked)

static GfxResourceID Gfx_AllocStaticVb(VertexFormat fmt, int count) {
	struct MetalVb* vb = (struct MetalVb*)Mem_TryAllocCleared(1, sizeof(struct MetalVb));
	if (!vb) return NULL;

	vb->size = count * strideSizes[fmt];
	vb->buf  = [mtl_device newBufferWithLength:vb->size options:STATIC_VB_OPTIONS];
	if (!vb->buf) { Mem_Free(vb); return NULL; }
	return vb;
}

void Gfx_BindVb(GfxResourceID vb) { boundVb = (struct MetalVb*)vb; }

void Gfx_DeleteVb(GfxResourceID* vb) {
	struct MetalVb* v = (struct MetalVb*)(*vb);
	if (!v) return;
	if (boundVb == v) boundVb = NULL;

	/* NOTE: Unretained command buffers, so must keep buffer alive until GPU is done with it */
	if (!v->dynamic) DeferRelease(v->buf, v->usedFrame);
	Mem_Free(v->data);
	Mem_Free(v);
	*vb = NULL;
}

void* Gfx_LockVb(GfxResourceID vb, VertexFormat fmt, int count) {
	struct MetalVb* v = (struct MetalVb*)vb;

	/* Writing directly into a buffer that the GPU may still be reading from is unsafe, */
	/*  so in that case just replace it with a new buffer */
	if (v->usedFrame && !GPUFinishedWith(v->usedFrame)) {
		DeferRelease(v->buf, v->usedFrame);
		v->buf       = [mtl_device newBufferWithLength:v->size options:STATIC_VB_OPTIONS];
		v->usedFrame = 0;
		if (!v->buf) Process_Abort("Out of memory for vertex buffer");
	}
	return [v->buf contents];
}

void Gfx_UnlockVb(GfxResourceID vb) { }


/*########################################################################################################################*
*--------------------------------------------------Dynamic vertex buffers-------------------------------------------------*
*#########################################################################################################################*/
static GfxResourceID Gfx_AllocDynamicVb(VertexFormat fmt, int maxVertices) {
	struct MetalVb* vb = (struct MetalVb*)Mem_TryAllocCleared(1, sizeof(struct MetalVb));
	if (!vb) return NULL;

	vb->dynamic = true;
	vb->size    = maxVertices * strideSizes[fmt];
	vb->data    = Mem_TryAlloc(vb->size, 1);
	if (!vb->data) { Mem_Free(vb); return NULL; }
	return vb;
}

void Gfx_BindDynamicVb(GfxResourceID vb) { Gfx_BindVb(vb); }

void Gfx_DeleteDynamicVb(GfxResourceID* vb) { Gfx_DeleteVb(vb); }

void* Gfx_LockDynamicVb(GfxResourceID vb, VertexFormat fmt, int count) {
	struct MetalVb* v = (struct MetalVb*)vb;
	v->dataSize = count * strideSizes[fmt];
	return v->data;
}

/* Copies latest contents of a dynamic vertex buffer into this frame's ring buffer */
static void UploadDynamicVb(struct MetalVb* v) {
	void* dst = Ring_Alloc(v->dataSize, &v->buf, &v->offset);
	Mem_Copy(dst, v->data, v->dataSize);
	v->dataFrame = mtl_frameId;
}

void Gfx_UnlockDynamicVb(GfxResourceID vb) {
	struct MetalVb* v = (struct MetalVb*)vb;
	UploadDynamicVb(v);
	Gfx_BindVb(vb);
}


/*########################################################################################################################*
*------------------------------------------------------Chunk VB pool------------------------------------------------------*
*#########################################################################################################################*/
/* Rather than each chunk mesh being in its own buffer, chunk meshes are sub-allocated */
/*  from large 'arena' buffers. Since the vertex buffer then rarely changes between draws, */
/*  the driver has much less state to re-emit per draw call (baseVertex selects the mesh) */
/* An arena starts small and is grown in place when full (by copying into a larger buffer), */
/*  so usually only one arena is needed. Since buffers are in unified memory, the CPU */
/*  writes chunk meshes directly into the arena, and freed ranges are only reused once */
/*  the GPU has finished all frames that may have been using them. */
#define POOL_MIN_VERTICES (256 * 1024)      /*   6 MB */
#define POOL_MAX_VERTICES (8 * 1024 * 1024) /* 192 MB */
#define POOL_MAX_ARENAS   64
#define POOL_GRANULARITY  4

struct PoolRange { int start, count; };
struct PoolArena {
	id<MTLBuffer> buf;
	int size, used, generation;
	int numFree, maxFree;
	struct PoolRange* free; /* sorted by start */
};
struct PoolAlloc { int arena, generation, start, count; cc_uint32 frame; };

static struct PoolArena pool_arenas[POOL_MAX_ARENAS];
static int pool_base;
static void ApplyState(void);
/* Ranges that were freed, but may still be in use by the GPU */
static struct PoolAlloc* pool_pending;
static int pool_numPending, pool_maxPending;

static cc_bool Pool_InsertFree(struct PoolArena* a, int index, int start, int count) {
	struct PoolRange* ranges;
	int i;
	if (a->numFree == a->maxFree) {
		int max = a->maxFree ? a->maxFree * 2 : 16;
		ranges  = (struct PoolRange*)Mem_TryRealloc(a->free, max, sizeof(struct PoolRange));
		if (!ranges) return false;
		a->free = ranges; a->maxFree = max;
	}

	for (i = a->numFree; i > index; i--) a->free[i] = a->free[i - 1];
	a->free[index].start = start;
	a->free[index].count = count;
	a->numFree++;
	return true;
}

static void Pool_RemoveFree(struct PoolArena* a, int index) {
	int i;
	for (i = index; i < a->numFree - 1; i++) a->free[i] = a->free[i + 1];
	a->numFree--;
}

/* Returns a range to the arena's free list, merging with adjacent free ranges */
static void Pool_ReleaseRange(struct PoolArena* a, int start, int count) {
	struct PoolRange* r;
	int lo = 0, hi = a->numFree, mid;

	while (lo < hi) {
		mid = (lo + hi) >> 1;
		if (a->free[mid].start < start) lo = mid + 1; else hi = mid;
	}

	if (lo > 0) {
		r = &a->free[lo - 1];
		if (r->start + r->count == start) {
			r->count += count;
			if (lo < a->numFree && r->start + r->count == a->free[lo].start) {
				r->count += a->free[lo].count;
				Pool_RemoveFree(a, lo);
			}
			return;
		}
	}

	if (lo < a->numFree && start + count == a->free[lo].start) {
		a->free[lo].start  = start;
		a->free[lo].count += count;
		return;
	}
	/* If out of memory, the range is just leaked until the arena is freed */
	Pool_InsertFree(a, lo, start, count);
}

static int Pool_TryAlloc(struct PoolArena* a, int count) {
	struct PoolRange* r;
	int i, start;

	for (i = 0; i < a->numFree; i++) 
	{
		r = &a->free[i];
		if (r->count < count) continue;

		start     = r->start;
		r->start += count;
		r->count -= count;
		if (!r->count) Pool_RemoveFree(a, i);

		a->used += count;
		return start;
	}
	return -1;
}

static void Pool_FreeArena(struct PoolArena* a) {
	DeferRelease(a->buf, mtl_frameId);
	Mem_Free(a->free);
	a->buf  = nil; a->size = 0; a->used = 0;
	a->free = NULL; a->numFree = 0; a->maxFree = 0;
	a->generation++;
}

static id<MTLBuffer> Pool_AllocBuffer(int vertices) {
	return [mtl_device newBufferWithLength:vertices * SIZEOF_VERTEX_TEXTURED options:STATIC_VB_OPTIONS];
}

static cc_bool Pool_CreateArena(struct PoolArena* a) {
	a->buf = Pool_AllocBuffer(POOL_MIN_VERTICES);
	if (!a->buf) return false;
	a->size = POOL_MIN_VERTICES;

	if (!Pool_InsertFree(a, 0, 0, a->size)) { Pool_FreeArena(a); return false; }
	return true;
}

/* Grows an arena to twice its size, keeping existing allocations at the same offsets */
static cc_bool Pool_GrowArena(struct PoolArena* a) {
	int newSize = min(a->size * 2, POOL_MAX_VERTICES);
	int last    = a->numFree - 1, extent = a->size;
	cc_bool freeAtEnd = last >= 0 && a->free[last].start + a->free[last].count == a->size;
	id<MTLBuffer> buf;

	buf = Pool_AllocBuffer(newSize);
	if (!buf) return false;
	if (freeAtEnd) extent = a->free[last].start;

	/* GPU only ever reads from the arena, so the CPU can safely copy it at any time */
	Mem_Copy([buf contents], [a->buf contents], extent * SIZEOF_VERTEX_TEXTURED);

	if (freeAtEnd) {
		a->free[last].count += newSize - a->size;
	} else if (!Pool_InsertFree(a, a->numFree, a->size, newSize - a->size)) {
		[buf release]; return false;
	}

	DeferRelease(a->buf, mtl_frameId);
	a->buf  = buf;
	a->size = newSize;
	return true;
}

static GfxResourceID Pool_TryCreate(int count) {
	struct PoolAlloc* alloc;
	struct PoolArena* a = NULL;
	int i, start = -1, empty = -1, growable = -1;

	alloc = (struct PoolAlloc*)Mem_TryAllocCleared(1, sizeof(struct PoolAlloc));
	if (!alloc) return NULL;

	for (i = 0; i < POOL_MAX_ARENAS; i++) 
	{
		a = &pool_arenas[i];
		if (!a->buf) { if (empty == -1) empty = i; continue; }

		start = Pool_TryAlloc(a, count);
		if (start >= 0) break;
		if (a->size < POOL_MAX_VERTICES && growable == -1) growable = i;
	}

	/* Prefer growing an existing arena over creating a new arena */
	if (start < 0 && growable >= 0) {
		i = growable;
		a = &pool_arenas[i];
		if (Pool_GrowArena(a)) start = Pool_TryAlloc(a, count);
	}

	if (start < 0) {
		if (empty == -1) { Mem_Free(alloc); return NULL; }
		i = empty;
		a = &pool_arenas[i];
		if (!Pool_CreateArena(a)) { Mem_Free(alloc); return NULL; }
		start = Pool_TryAlloc(a, count);
	}

	alloc->arena      = i;
	alloc->generation = a->generation;
	alloc->start      = start;
	alloc->count      = count;
	return alloc;
}

GfxResourceID Gfx_CreateChunkVb(int count) {
	GfxResourceID vb;
	count = (count + (POOL_GRANULARITY - 1)) & ~(POOL_GRANULARITY - 1);
	if (count > POOL_MIN_VERTICES) return NULL;

	/* Same behaviour as Gfx_TryCreateStaticVb when out of memory */
	for (;;)
	{
		if ((vb = Pool_TryCreate(count))) return vb;
		if (!Game_ReduceVRAM()) return NULL;
	}
}

static void Pool_Release(struct PoolAlloc* alloc) {
	struct PoolArena* a = &pool_arenas[alloc->arena];
	if (alloc->generation != a->generation) return; /* arena already freed */

	a->used -= alloc->count;
	if (a->used <= 0) { Pool_FreeArena(a); return; }
	Pool_ReleaseRange(a, alloc->start, alloc->count);
}

void Gfx_DeleteChunkVb(GfxResourceID* vb) {
	struct PoolAlloc* alloc = (struct PoolAlloc*)(*vb);
	struct PoolAlloc* pending;
	int max;
	if (!alloc) return;
	*vb = NULL;

	/* GPU may still be reading this range, so delay reusing it */
	if (alloc->frame && !GPUFinishedWith(alloc->frame)) {
		if (pool_numPending == pool_maxPending) {
			max     = pool_maxPending ? pool_maxPending * 2 : 256;
			pending = (struct PoolAlloc*)Mem_TryRealloc(pool_pending, max, sizeof(struct PoolAlloc));
			if (pending) { pool_pending = pending; pool_maxPending = max; }
		}

		if (pool_numPending < pool_maxPending) {
			pool_pending[pool_numPending++] = *alloc;
		} /* else out of memory, so just leak the range until the arena is freed */
	} else {
		Pool_Release(alloc);
	}
	Mem_Free(alloc);
}

static void Pool_ProcessPending(void) {
	cc_uint32 completed = atomic_load(&mtl_completedFrameId);
	int i, j = 0;

	for (i = 0; i < pool_numPending; i++) 
	{
		if (pool_pending[i].frame <= completed) {
			Pool_Release(&pool_pending[i]);
		} else {
			pool_pending[j++] = pool_pending[i];
		}
	}
	pool_numPending = j;
}

void* Gfx_LockChunkVb(GfxResourceID vb, int count) {
	struct PoolAlloc* alloc = (struct PoolAlloc*)vb;
	cc_uint8* data = (cc_uint8*)[pool_arenas[alloc->arena].buf contents];
	return data + alloc->start * SIZEOF_VERTEX_TEXTURED;
}

void Gfx_UnlockChunkVb(GfxResourceID vb) { }

void Gfx_BindChunkVb(GfxResourceID vb) {
	struct PoolAlloc* alloc = (struct PoolAlloc*)vb;
	id<MTLBuffer> buf = pool_arenas[alloc->arena].buf;
	alloc->frame = mtl_frameId;
	pool_base    = alloc->start;
	BeginRenderPass();

	if (buf != enc_vb || enc_vbOffset != 0) {
		[mtl_enc setVertexBuffer:buf offset:0 atIndex:0];
		enc_vb = buf; enc_vbOffset = 0;
	}
	boundVb = NULL; /* so normal vertex buffer gets rebound when next used */
}

void Gfx_DrawChunkVb(int verticesCount, int startVertex) {
	if (dirty) ApplyState();
	[mtl_enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle indexCount:ICOUNT(verticesCount) 
		indexType:MTLIndexTypeUInt16 indexBuffer:boundIb indexBufferOffset:0 
		instanceCount:1 baseVertex:pool_base + startVertex baseInstance:0];
}

void Gfx_EndChunkDraws(void) { }


/*########################################################################################################################*
*-------------------------------------------------------Deferred release--------------------------------------------------*
*#########################################################################################################################*/
/* Since command buffers don't retain the resources they use (for performance), */
/*  buffers must be kept alive until the GPU has finished all frames that use them */
static struct DeferredRelease { id obj; cc_uint32 frame; }* deferred;
static int deferredCount, deferredCapacity;

static void DeferRelease(id obj, cc_uint32 frame) {
	if (!obj) return;
	if (!frame || GPUFinishedWith(frame)) { [obj release]; return; }

	if (deferredCount == deferredCapacity) {
		deferredCapacity = deferredCapacity ? deferredCapacity * 2 : 256;
		deferred = (struct DeferredRelease*)Mem_Realloc(deferred, deferredCapacity,
						sizeof(struct DeferredRelease), "deferred releases");
	}
	deferred[deferredCount].obj   = obj;
	deferred[deferredCount].frame = frame;
	deferredCount++;
}

static void ProcessDeferredReleases(void) {
	cc_uint32 completed = atomic_load(&mtl_completedFrameId);
	int i, j = 0;

	for (i = 0; i < deferredCount; i++)
	{
		if (deferred[i].frame <= completed) {
			[deferred[i].obj release];
		} else {
			deferred[j++] = deferred[i];
		}
	}
	deferredCount = j;
}


/*########################################################################################################################*
*---------------------------------------------------------Drawing---------------------------------------------------------*
*#########################################################################################################################*/
static void ApplyState(void) {
	struct MetalVb* v = boundVb;
	id<MTLRenderPipelineState> pso;
	id<MTLSamplerState> smp;
	BeginRenderPass();

	if (dirty) {
		if (dirty & DIRTY_PIPELINE) {
			pso = GetPipeline();
			if (pso != enc_pipeline) { [mtl_enc setRenderPipelineState:pso]; enc_pipeline = pso; }
		}
		if (dirty & DIRTY_DEPTH)    [mtl_enc setDepthStencilState:GetDepthState()];
		if (dirty & DIRTY_VIEWPORT) [mtl_enc setViewport:viewport];
		if (dirty & DIRTY_SCISSOR)  [mtl_enc setScissorRect:scissor];
		if (dirty & DIRTY_VS_UNI)   [mtl_enc setVertexBytes:&vs_uni length:sizeof(vs_uni) atIndex:1];
		if (dirty & DIRTY_FS_UNI)   [mtl_enc setFragmentBytes:&fs_uni length:sizeof(fs_uni) atIndex:0];

		if (dirty & DIRTY_CULL) {
			[mtl_enc setFrontFacingWinding:MTLWindingCounterClockwise];
			[mtl_enc setCullMode:cullEnabled ? MTLCullModeBack : MTLCullModeNone];
		}
		if ((dirty & DIRTY_TEXTURE) && !boundTex) boundTex = (struct MetalTexture*)white_square;
		if ((dirty & DIRTY_TEXTURE) && boundTex) {
			boundTex->usedFrame = mtl_frameId;
			if (boundTex->tex != enc_tex) { [mtl_enc setFragmentTexture:boundTex->tex atIndex:0]; enc_tex = boundTex->tex; }
			smp = GetSampler(boundTex->sampler);
			if (smp != enc_sampler) { [mtl_enc setFragmentSamplerState:smp atIndex:0]; enc_sampler = smp; }
		}
		dirty = 0;
	}

	if (!v) return;
	if (v->dynamic && v->dataFrame != mtl_frameId) UploadDynamicVb(v);
	v->usedFrame = mtl_frameId;

	if (v->buf != enc_vb) {
		[mtl_enc setVertexBuffer:v->buf offset:v->offset atIndex:0];
		enc_vb = v->buf; enc_vbOffset = v->offset;
	} else if (v->offset != enc_vbOffset) {
		[mtl_enc setVertexBufferOffset:v->offset atIndex:0];
		enc_vbOffset = v->offset;
	}
}

void Gfx_DrawVb_Lines(int verticesCount) {
	ApplyState();
	[mtl_enc drawPrimitives:MTLPrimitiveTypeLine vertexStart:0 vertexCount:verticesCount];
}

static CC_INLINE void DrawQuads(int verticesCount, int startVertex) {
	ApplyState();
	[mtl_enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle indexCount:ICOUNT(verticesCount)
		indexType:MTLIndexTypeUInt16 indexBuffer:boundIb indexBufferOffset:0
		instanceCount:1 baseVertex:startVertex baseInstance:0];
}

void Gfx_DrawVb_IndexedTris_Range(int verticesCount, int startVertex, DrawHints hints) {
	DrawQuads(verticesCount, startVertex);
}

void Gfx_DrawVb_IndexedTris(int verticesCount) {
	DrawQuads(verticesCount, 0);
}

void Gfx_DrawIndexedTris_T2fC4b(int verticesCount, int startVertex, DrawHints hints) {
	DrawQuads(verticesCount, startVertex);
}


/*########################################################################################################################*
*-----------------------------------------------------------Misc----------------------------------------------------------*
*#########################################################################################################################*/
/* Copies the current contents of the render target into a CPU accessible buffer */
/* NOTE: Pixels are in top-down order, and in the same format as BitmapCol */
static id<MTLBuffer> ReadRenderTarget(int* width, int* height) {
	id<MTLTexture> tex;
	id<MTLBuffer> readback;
	id<MTLBlitCommandEncoder> blit;
	int w, h;

	BeginRenderPass();
	EndRenderPass(true);
	EndUploads();

	tex = mtl_target;
	w   = (int)[tex width];
	h   = (int)[tex height];
	readback = [mtl_device newBufferWithLength:w * h * 4 options:MTLResourceStorageModeShared];
	if (!readback) return nil;

	blit = [mtl_cmdBuf blitCommandEncoder];
	[blit copyFromTexture:tex sourceSlice:0 sourceLevel:0 sourceOrigin:MTLOriginMake(0, 0, 0)
		sourceSize:MTLSizeMake(w, h, 1) toBuffer:readback destinationOffset:0
		destinationBytesPerRow:w * 4 destinationBytesPerImage:w * h * 4];
	[blit endEncoding];

	/* Submit everything rendered so far, then continue the frame in a new command buffer */
	[mtl_cmdBuf commit];
	[mtl_cmdBuf waitUntilCompleted];
	[mtl_cmdBuf release];
	mtl_cmdBuf = [[mtl_queue commandBufferWithUnretainedReferences] retain];

	*width = w; *height = h;
	return readback;
}

cc_result Gfx_TakeScreenshot(struct Stream* output) {
	id<MTLBuffer> readback;
	struct Bitmap bmp;
	int width, height;
	cc_result res;

	readback = ReadRenderTarget(&width, &height);
	if (!readback) return ERR_OUT_OF_MEMORY;

	Bitmap_Init(bmp, width, height, (BitmapCol*)[readback contents]);
	res = Png_Encode(&bmp, output, NULL, false, NULL);
	[readback release];
	return res;
}

cc_result Gfx_ReadBackbuffer(struct Bitmap* bmp) {
	id<MTLBuffer> readback;
	BitmapCol* src;
	BitmapCol* dst;
	int width, height, x, y, srcY;

	readback = ReadRenderTarget(&width, &height);
	if (!readback) return ERR_OUT_OF_MEMORY;
	src = (BitmapCol*)[readback contents];

	/* Render target is in pixels, but callers work in window (point) dimensions, */
	/*  so on high DPI displays the render target is larger than the bitmap. */
	/* Point sampling is used so that exact colours are preserved */
	for (y = 0; y < bmp->height; y++) {
		srcY = (int)((cc_uint64)y * height / bmp->height);
		dst  = Bitmap_GetRow(bmp, y);

		if (width == bmp->width) {
			Mem_Copy(dst, src + srcY * width, width * BITMAPCOLOR_SIZE);
			continue;
		}
		for (x = 0; x < bmp->width; x++) {
			dst[x] = src[srcY * width + (int)((cc_uint64)x * width / bmp->width)];
		}
	}

	[readback release];
	return 0;
}
#endif
