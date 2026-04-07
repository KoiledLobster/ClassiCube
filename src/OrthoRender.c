#include "OrthoRender.h"
#include "Builder.h"
#include "Chat.h"
#include "Lighting.h"
#include "Commands.h"
#include "Game.h"
#include "Graphics.h"
#include "Camera.h"
#include "World.h"
#include "Vectors.h"
#include "ExtMath.h"
#include "Funcs.h"
#include "MapRenderer.h"
#include "EnvRenderer.h"
#include "Bitmap.h"
#include "Stream.h"
#include "Platform.h"
#include "Logger.h"
#include "Utils.h"
#include "Event.h"
#include "String_.h"
#include "Entity.h"
#include "Particle.h"
#include "SelectionBox.h"
#include "Picking.h"
#include "Input.h"
#include "Gui.h"

#include <stdio.h>

/* Convenience macro missing from Vectors.h */
#define IVec3_Set(v, px, py, pz) do { (v).x = (px); (v).y = (py); (v).z = (pz); } while(0)

cc_bool OrthoRender_Requested;
static float ortho_pitch = -1.0f; /* -1 means "use player viewpoint" */
static float ortho_yaw   = -1.0f;
static cc_bool ortho_gradientBG; /* mode:gradient — replace border with sky→fog gradient */
static int ortho_pxPerBlock;     /* pixels per block; 0 = use default */

/* Region selection state — in-memory, cleared after render or map change */
static cc_bool ortho_regionActive;   /* P1/P2 are set and should constrain the render */
static IVec3   ortho_regionP1, ortho_regionP2;
static cc_bool ortho_markingActive;  /* waiting for block clicks to set P1/P2 */
static int     ortho_markStep;       /* 0 = waiting for P1, 1 = waiting for P2 */
static float   ortho_markSavedReach; /* Picking reach before we enlarged it */

/* Last confirmed region, for /client orthoregion last */
static IVec3   ortho_lastP1, ortho_lastP2;
static cc_bool ortho_hasLastRegion;

/* Selection IDs for the region preview (high range to avoid server conflicts) */
#define ORTHO_SEL_BOX 240

/* Pixels per world unit (block) — default when width:N is not specified */
#define ORTHO_PX_PER_BLOCK 16


/*########################################################################################################################*
*-------------------------------------------------Orthographic projection-------------------------------------------------*
*#########################################################################################################################*/
/* Build a general 3D orthographic projection matrix with arbitrary bounds.
   Unlike Gfx_CalcOrthoMatrix (which is for 2D UI), this handles
   arbitrary left/right/bottom/top and correct depth for each backend. */
static void OrthoRender_CalcOrthoMatrix(struct Matrix* matrix,
	float left, float right, float bottom, float top, float zNear, float zFar) {
	*matrix = Matrix_Identity;

	matrix->row1.x = 2.0f / (right - left);
	matrix->row2.y = 2.0f / (top - bottom);
	matrix->row4.x = -(right + left)   / (right - left);
	matrix->row4.y = -(top   + bottom) / (top - bottom);

#if CC_GFX_BACKEND == CC_GFX_BACKEND_GL1 || CC_GFX_BACKEND == CC_GFX_BACKEND_GL2 || CC_GFX_BACKEND == CC_GFX_BACKEND_GL11
	/* OpenGL: Z clip space [-1, 1] */
	matrix->row3.z = -2.0f / (zFar - zNear);
	matrix->row4.z = -(zFar + zNear) / (zFar - zNear);
#elif CC_GFX_BACKEND == CC_GFX_BACKEND_D3D9 || CC_GFX_BACKEND == CC_GFX_BACKEND_D3D11
	/* Direct3D 9/11: reversed depth, Z clip [0, 1] with near->1, far->0 */
	matrix->row3.z =  1.0f / (zFar - zNear);
	matrix->row4.z =  zFar / (zFar - zNear);
#else
	/* Default: standard Z clip [0, 1] */
	matrix->row3.z = -1.0f / (zFar - zNear);
	matrix->row4.z = -zNear / (zFar - zNear);
#endif
}


/*########################################################################################################################*
*--------------------------------------------------Region selection helpers------------------------------------------------*
*#########################################################################################################################*/
static void OrthoRegion_ExitMarkingMode(void);
static void OrthoRegion_OnBlockChanged(void* obj, IVec3 coords, BlockID oldBlock, BlockID block);
static void OrthoRegion_OnPointerDown(void* obj, int idx);
static void OrthoRegion_OnInputDown(void* obj, int key, cc_bool was, struct InputDevice* device);
static void OrthoRegion_OnMarkPoint(IVec3 pos);

static void OrthoRegion_ExitMarkingMode(void) {
	struct LocalPlayer* lp;
	if (!ortho_markingActive) return;
	ortho_markingActive = false;

	lp = Entities.CurPlayer;
	lp->ReachDistance = ortho_markSavedReach;
	Event_Unregister_(&PointerEvents.Down,      NULL, OrthoRegion_OnPointerDown);
	Event_Unregister_(&InputEvents.Down2,        NULL, OrthoRegion_OnInputDown);
	Event_Unregister_(&UserEvents.BlockChanged,  NULL, OrthoRegion_OnBlockChanged);
}

static void OrthoRegion_Clear(void) {
	Selections_Remove(ORTHO_SEL_BOX);
	ortho_regionActive = false;
	ortho_markStep     = 0;
	OrthoRegion_ExitMarkingMode();
}

static void OrthoRegion_UpdatePreview(void) {
	IVec3 lo, hi;
	lo.x = min(ortho_regionP1.x, ortho_regionP2.x);
	lo.y = min(ortho_regionP1.y, ortho_regionP2.y);
	lo.z = min(ortho_regionP1.z, ortho_regionP2.z);
	hi.x = max(ortho_regionP1.x, ortho_regionP2.x) + 1;
	hi.y = max(ortho_regionP1.y, ortho_regionP2.y) + 1;
	hi.z = max(ortho_regionP1.z, ortho_regionP2.z) + 1;
	Selections_Add(ORTHO_SEL_BOX, &lo, &hi, PackedCol_Make(0, 180, 255, 70));
}

/* Called by both click handlers and /client mark */
static void OrthoRegion_OnMarkPoint(IVec3 pos) {
	if (ortho_markStep == 0) {
		ortho_regionP1 = pos;
		ortho_markStep = 1;
		Chat_AddRaw("&eOrthoRegion: &fP1 set. Left/right-click or /client mark for P2.");
	} else {
		ortho_regionP2 = pos;
		OrthoRegion_ExitMarkingMode();
		ortho_regionActive  = true;
		ortho_lastP1        = ortho_regionP1;
		ortho_lastP2        = ortho_regionP2;
		ortho_hasLastRegion = true;
		OrthoRegion_UpdatePreview();
		Chat_AddRaw("&eOrthoRegion: &fRegion set. Run /client orthorender to render it.");
	}
}

/* PointerEvents.Down: left-click — sets corner to the targeted block's position */
static void OrthoRegion_OnPointerDown(void* obj, int idx) {
	(void)obj; (void)idx;
	if (!ortho_markingActive || !Game_SelectedPos.valid) return;
	OrthoRegion_OnMarkPoint(Game_SelectedPos.pos);
}

/* InputEvents.Down2: right-click — sets corner to where a block would be placed */
static void OrthoRegion_OnInputDown(void* obj, int key, cc_bool was, struct InputDevice* device) {
	(void)obj; (void)was; (void)device;
	if (!ortho_markingActive || key != CCMOUSE_R) return;
	if (!Game_SelectedPos.valid || Gui_GetInputGrab()) return;
	OrthoRegion_OnMarkPoint(Game_SelectedPos.translatedPos);
}

/* UserEvents.BlockChanged: restore any block accidentally changed during marking */
static void OrthoRegion_OnBlockChanged(void* obj, IVec3 coords, BlockID oldBlock, BlockID block) {
	(void)obj; (void)block;
	if (!oldBlock) return;
	World_SetBlock(coords.x, coords.y, coords.z, oldBlock);
	MapRenderer_OnBlockChanged(coords.x, coords.y, coords.z, oldBlock);
}

static void OrthoRegion_EnterMarkingMode(void) {
	struct LocalPlayer* lp = Entities.CurPlayer;
	if (ortho_markingActive) {
		Chat_AddRaw("&eOrthoRegion: &fAlready marking — click or /client mark for P1.");
		return;
	}

	ortho_markingActive  = true;
	ortho_markStep       = 0;
	ortho_markSavedReach = lp->ReachDistance;
	lp->ReachDistance    = 4096.0f;

	Event_Register_(&PointerEvents.Down,      NULL, OrthoRegion_OnPointerDown);
	Event_Register_(&InputEvents.Down2,        NULL, OrthoRegion_OnInputDown);
	Event_Register_(&UserEvents.BlockChanged,  NULL, OrthoRegion_OnBlockChanged);
	Chat_AddRaw("&eOrthoRegion: &fLeft/right-click (or /client mark) for P1.");
}

static void OrthoRegion_OnMapLoaded(void* obj) {
	(void)obj;
	OrthoRegion_Clear();
}

/* Exposed so OrthoRender_Execute can call it after a region render */
static void OrthoRegion_FinishRender(void) {
	OrthoRegion_Clear();
}

/*########################################################################################################################*
*-------------------------------------------------Entity region culling---------------------------------------------------*
*#########################################################################################################################*/
static struct Entity* ortho_savedEntities[ENTITIES_MAX_COUNT];

/* Null out entries in Entities.List whose position falls outside the active region.
   Saved pointers are restored by OrthoRender_EndEntityCull after rendering. */
static void OrthoRender_BeginEntityCull(void) {
	IVec3 lo, hi;
	int i;

	if (!ortho_regionActive) return;

	lo.x = min(ortho_regionP1.x, ortho_regionP2.x);
	lo.y = min(ortho_regionP1.y, ortho_regionP2.y);
	lo.z = min(ortho_regionP1.z, ortho_regionP2.z);
	hi.x = max(ortho_regionP1.x, ortho_regionP2.x) + 1;
	hi.y = max(ortho_regionP1.y, ortho_regionP2.y) + 1;
	hi.z = max(ortho_regionP1.z, ortho_regionP2.z) + 1;

	for (i = 0; i < ENTITIES_MAX_COUNT; i++) {
		struct Entity* ent = Entities.List[i];
		ortho_savedEntities[i] = ent;
		if (!ent) continue;
		if (ent->Position.x < (float)lo.x || ent->Position.x >= (float)hi.x ||
			ent->Position.y < (float)lo.y || ent->Position.y >= (float)hi.y ||
			ent->Position.z < (float)lo.z || ent->Position.z >= (float)hi.z) {
			Entities.List[i] = NULL;
		}
	}
}

/* Restore Entities.List to its original state after a region render. */
static void OrthoRender_EndEntityCull(void) {
	int i;
	if (!ortho_regionActive) return;
	for (i = 0; i < ENTITIES_MAX_COUNT; i++) {
		Entities.List[i] = ortho_savedEntities[i];
	}
}



static void OrthoRender_ComputeView(float yawDeg, float pitchDeg,
	float* outLeft, float* outRight, float* outBottom, float* outTop,
	float* outNear, float* outFar, struct Matrix* outView) {
	struct Matrix rotX, rotY, rotation;
	Vec3 corners[8], transformed, mapCenter, camPos, viewDir;
	float yawRad, pitchRad;
	float minX, maxX, minY, maxY;
	float w, h, l, diag, dist, pad;
	Vec2 rot;
	int i;

	if (ortho_regionActive) {
		/* Use region corners instead of full world bounds */
		w = (float)(max(ortho_regionP1.x, ortho_regionP2.x) + 1 - min(ortho_regionP1.x, ortho_regionP2.x));
		h = (float)(max(ortho_regionP1.y, ortho_regionP2.y) + 1 - min(ortho_regionP1.y, ortho_regionP2.y));
		l = (float)(max(ortho_regionP1.z, ortho_regionP2.z) + 1 - min(ortho_regionP1.z, ortho_regionP2.z));
		mapCenter = Vec3_Create3(
			(float)(min(ortho_regionP1.x, ortho_regionP2.x)) + w * 0.5f,
			(float)(min(ortho_regionP1.y, ortho_regionP2.y)) + h * 0.5f,
			(float)(min(ortho_regionP1.z, ortho_regionP2.z)) + l * 0.5f
		);
		{
			float rx = mapCenter.x - w * 0.5f, ry = mapCenter.y - h * 0.5f, rz = mapCenter.z - l * 0.5f;
			Vec3_Set(corners[0], rx,   ry,   rz  );  Vec3_Set(corners[1], rx+w, ry,   rz  );
			Vec3_Set(corners[2], rx,   ry+h, rz  );  Vec3_Set(corners[3], rx+w, ry+h, rz  );
			Vec3_Set(corners[4], rx,   ry,   rz+l);  Vec3_Set(corners[5], rx+w, ry,   rz+l);
			Vec3_Set(corners[6], rx,   ry+h, rz+l);  Vec3_Set(corners[7], rx+w, ry+h, rz+l);
		}
	} else {
		w = (float)World.Width;
		h = (float)World.Height;
		l = (float)World.Length;
		mapCenter = Vec3_Create3(w * 0.5f, h * 0.5f, l * 0.5f);
		Vec3_Set(corners[0], 0, 0, 0);  Vec3_Set(corners[1], w, 0, 0);
		Vec3_Set(corners[2], 0, h, 0);  Vec3_Set(corners[3], w, h, 0);
		Vec3_Set(corners[4], 0, 0, l);  Vec3_Set(corners[5], w, 0, l);
		Vec3_Set(corners[6], 0, h, l);  Vec3_Set(corners[7], w, h, l);
	}

	yawRad   = yawDeg   * MATH_DEG2RAD;
	pitchRad = pitchDeg * MATH_DEG2RAD;

	/* Build rotation-only matrix (same order as Matrix_LookRot) */
	Matrix_RotateY(&rotY, yawRad);
	Matrix_RotateX(&rotX, pitchRad);
	Matrix_Mul(&rotation, &rotY, &rotX);

	/* Find X/Y extents in view-rotation space (centered on region/map) */
	minX = minY =  1e30f;
	maxX = maxY = -1e30f;

	for (i = 0; i < 8; i++) {
		Vec3 centered;
		Vec3_Sub(&centered, &corners[i], &mapCenter);
		Vec3_Transform(&transformed, &centered, &rotation);

		if (transformed.x < minX) minX = transformed.x;
		if (transformed.x > maxX) maxX = transformed.x;
		if (transformed.y < minY) minY = transformed.y;
		if (transformed.y > maxY) maxY = transformed.y;
	}

	/* Add 5% padding */
	pad = (maxX - minX) * 0.025f;
	minX -= pad; maxX += pad;
	pad = (maxY - minY) * 0.025f;
	minY -= pad; maxY += pad;

	*outLeft   = minX;
	*outRight  = maxX;
	*outBottom = minY;
	*outTop    = maxY;

	/* Position camera far behind the map along the view direction */
	diag = Math_SqrtF(w * w + h * h + l * l);
	dist = diag * 2.0f;

	viewDir = Vec3_GetDirVector(yawRad, pitchRad);
	Vec3_Mul1(&camPos, &viewDir, -dist);
	Vec3_Add(&camPos, &mapCenter, &camPos);

	*outNear = 0.1f;
	*outFar  = dist * 3.0f;

	/* Build the full view matrix */
	rot.x = yawRad;
	rot.y = pitchRad;
	Matrix_LookRot(outView, camPos, rot);
}


/*########################################################################################################################*
*----------------------------------------------Tile file I/O helpers------------------------------------------------------*
*#########################################################################################################################*/
/* Delete a file by its encoded platform path */
static void OrthoRender_DeleteFile(const cc_filepath* path) {
#if defined CC_BUILD_WIN || defined CC_BUILD_WINCE
	remove(path->ansi);
#else
	remove(path->buffer);
#endif
}

/* Build the temp file path for a given tile coordinate */
static void OrthoRender_TilePath(cc_string* path, cc_filepath* rawPath, int ty, int tx) {
	String_Format2(path, "screenshots/ortho_tmp_%i_%i.raw", &ty, &tx);
	Platform_EncodePath(rawPath, path);
}

/* Write raw tile pixel data to a temporary file */
static cc_result OrthoRender_WriteTileRaw(int ty, int tx, BitmapCol* pixels, int tileW, int tileH) {
	cc_string path; char pathBuf[FILENAME_SIZE];
	cc_filepath rawPath;
	struct Stream stream;
	cc_result res;

	String_InitArray(path, pathBuf);
	OrthoRender_TilePath(&path, &rawPath, ty, tx);

	res = Stream_CreatePath(&stream, &rawPath);
	if (res) return res;

	res = Stream_Write(&stream, (const cc_uint8*)pixels,
		(cc_uint32)tileW * tileH * BITMAPCOLOR_SIZE);
	stream.Close(&stream);
	return res;
}

/* Delete all temporary tile files */
static void OrthoRender_CleanupTileFiles(int numTilesY, int numTilesX) {
	cc_string path; char pathBuf[FILENAME_SIZE];
	cc_filepath rawPath;
	int ty, tx;

	for (ty = 0; ty < numTilesY; ty++) {
		for (tx = 0; tx < numTilesX; tx++) {
			String_InitArray(path, pathBuf);
			OrthoRender_TilePath(&path, &rawPath, ty, tx);
			OrthoRender_DeleteFile(&rawPath);
		}
	}
}


/*########################################################################################################################*
*-----------------------------------------Iterative image assembly from tiles---------------------------------------------*
*#########################################################################################################################*/
/* Context passed to the PNG row getter for iterative encoding */
struct OrthoRowCtx {
	BitmapCol* stripBuf;   /* imgW * tileH pixels — one horizontal strip */
	BitmapCol* tileBuf;    /* tileW * tileH pixels — for reading one tile file */
	int imgW, imgH;
	int tileW, tileH;
	int numTilesX;
	int loadedTileRow;     /* which ty is currently in stripBuf (-1 = none) */
	cc_result error;       /* set if file I/O fails during load */
};

/* Load all tile files for a given tile row into the strip buffer */
static void OrthoRender_LoadTileRow(struct OrthoRowCtx* ctx, int ty) {
	int tx, tw, th, row;
	cc_string path; char pathBuf[FILENAME_SIZE];
	cc_filepath rawPath;
	struct Stream stream;
	cc_result res;

	th = min(ctx->tileH, ctx->imgH - ty * ctx->tileH);
	Mem_Set(ctx->stripBuf, 0, (cc_uint32)ctx->imgW * ctx->tileH * BITMAPCOLOR_SIZE);

	for (tx = 0; tx < ctx->numTilesX; tx++) {
		tw = min(ctx->tileW, ctx->imgW - tx * ctx->tileW);

		String_InitArray(path, pathBuf);
		OrthoRender_TilePath(&path, &rawPath, ty, tx);

		res = Stream_OpenPath(&stream, &rawPath);
		if (res) { ctx->error = res; return; }

		res = Stream_Read(&stream, (cc_uint8*)ctx->tileBuf,
			(cc_uint32)ctx->tileW * ctx->tileH * BITMAPCOLOR_SIZE);
		stream.Close(&stream);
		if (res) { ctx->error = res; return; }

		/* Copy valid portion (tw x th) into the strip */
		for (row = 0; row < th; row++) {
			BitmapCol* src = ctx->tileBuf + row * ctx->tileW;
			BitmapCol* dst = ctx->stripBuf + row * ctx->imgW + tx * ctx->tileW;
			Mem_Copy(dst, src, tw * BITMAPCOLOR_SIZE);
		}
	}

	ctx->loadedTileRow = ty;
}

/* PNG row getter: returns the requested row from the tile-backed strip buffer.
   Loads the appropriate tile row from disk on demand. */
static BitmapCol* OrthoRender_GetRow(struct Bitmap* bmp, int y, void* ctx) {
	struct OrthoRowCtx* c = (struct OrthoRowCtx*)ctx;
	int ty = y / c->tileH;

	if (ty != c->loadedTileRow && !c->error) {
		OrthoRender_LoadTileRow(c, ty);
	}

	return c->stripBuf + (y % c->tileH) * c->imgW;
}

/* Build a 1/4-scale small image from the tile files using the row context.
   Allocates fourRows (4 * imgW pixels) to stage source rows for averaging,
   so we never need the full image in memory. */
static void OrthoRender_BuildSmallImage(struct OrthoRowCtx* ctx,
	struct Bitmap* smallBmp, int imgW, int imgH) {
	BitmapCol* fourRows;
	BitmapCol pixel;
	int smallW = imgW / 4, smallH = imgH / 4;
	int dx, dy, i, j, sy, ty;
	int r, g, b, count;

	if (smallW <= 0 || smallH <= 0) return;

	fourRows = (BitmapCol*)Mem_TryAlloc(imgW * 4, BITMAPCOLOR_SIZE);
	if (!fourRows) return;

	smallBmp->width  = smallW;
	smallBmp->height = smallH;
	smallBmp->scan0  = (BitmapCol*)Mem_TryAlloc(smallW * smallH, BITMAPCOLOR_SIZE);
	if (!smallBmp->scan0) { Mem_Free(fourRows); return; }

	for (dy = 0; dy < smallH; dy++) {
		int sy0 = dy * 4;

		/* Copy 4 source rows into fourRows buffer (handles tile-row boundaries) */
		for (i = 0; i < 4 && sy0 + i < imgH; i++) {
			sy = sy0 + i;
			ty = sy / ctx->tileH;
			if (ty != ctx->loadedTileRow && !ctx->error) {
				OrthoRender_LoadTileRow(ctx, ty);
			}
			if (ctx->error) goto done;
			Mem_Copy(fourRows + i * imgW,
				ctx->stripBuf + (sy % ctx->tileH) * ctx->imgW,
				imgW * BITMAPCOLOR_SIZE);
		}

		/* Box-average each 4x4 block */
		for (dx = 0; dx < smallW; dx++) {
			int sx0 = dx * 4;
			r = 0; g = 0; b = 0; count = 0;
			for (i = 0; i < 4 && sy0 + i < imgH; i++) {
				for (j = 0; j < 4 && sx0 + j < imgW; j++) {
					pixel = fourRows[i * imgW + sx0 + j];
					r += BitmapCol_R(pixel);
					g += BitmapCol_G(pixel);
					b += BitmapCol_B(pixel);
					count++;
				}
			}
			if (count > 0) { r /= count; g /= count; b /= count; }
			Bitmap_GetPixel(smallBmp, dx, dy) = BitmapCol_Make(r, g, b, 255);
		}
	}

done:
	Mem_Free(fourRows);
}


/* Render cloud plane with halved texture scale (clouds appear smaller/denser).
   Uses UV divisor of 1024 instead of the default 2048. */
static GfxResourceID ortho_clouds_vb;

static void OrthoRender_RenderClouds(void) {
	struct VertexTextured v[4];
	GfxResourceID tex;
	float x1, z1, x2, z2, y, u1, u2, v1, v2, ext;
	PackedCol col;

	tex = EnvRenderer_GetCloudsTex();
	if (!tex || Env.CloudsHeight < -2000) return;

	/* Extend clouds well beyond map so they fill the visible ortho area */
	ext = Math_SqrtF((float)(World.Width * World.Width + World.Length * World.Length)) * 2.0f;
	if (ext < 512.0f) ext = 512.0f;

	x1 = -ext; z1 = -ext;
	x2 = (float)World.Width  + ext;
	z2 = (float)World.Length + ext;
	y  = (float)Env.CloudsHeight + 0.1f;
	col = Env.CloudsCol;

	/* Half the divisor (1024 vs 2048) = clouds appear at half scale */
	u1 = x1 / 1024.0f; u2 = x2 / 1024.0f;
	v1 = z1 / 1024.0f; v2 = z2 / 1024.0f;

	v[0].x = x1; v[0].y = y; v[0].z = z1; v[0].Col = col; v[0].U = u1; v[0].V = v1;
	v[1].x = x1; v[1].y = y; v[1].z = z2; v[1].Col = col; v[1].U = u1; v[1].V = v2;
	v[2].x = x2; v[2].y = y; v[2].z = z2; v[2].Col = col; v[2].U = u2; v[2].V = v2;
	v[3].x = x2; v[3].y = y; v[3].z = z1; v[3].Col = col; v[3].U = u2; v[3].V = v1;

	if (!ortho_clouds_vb) {
		ortho_clouds_vb = Gfx_CreateDynamicVb(VERTEX_FORMAT_TEXTURED, 4);
	}

	Gfx_SetAlphaTest(true);
	Gfx_BindTexture(tex);
	Gfx_SetVertexFormat(VERTEX_FORMAT_TEXTURED);
	Gfx_SetDynamicVbData(ortho_clouds_vb, v, 4);
	Gfx_DrawVb_IndexedTris(4);
	Gfx_SetAlphaTest(false);
}


/*########################################################################################################################*
*---------------------------------------------------Rendering core--------------------------------------------------------*
*#########################################################################################################################*/
#define ORTHO_SKYBOX_VERTS (6 * 4)
static GfxResourceID ortho_skybox_vb;

/* Render skybox as a large cube centered on the map using the current ortho projection.
   Unlike the built-in skybox (which uses perspective from origin), this scales the cube
   to surround the map so it tiles correctly across ortho render tiles. */
static void OrthoRender_RenderSkybox(void) {
	static const float uvs[ORTHO_SKYBOX_VERTS][2] = {
		/* Front */  {0.25f,1.00f}, {0.50f,1.00f}, {0.50f,0.50f}, {0.25f,0.50f},
		/* Left */   {0.00f,1.00f}, {0.25f,1.00f}, {0.25f,0.50f}, {0.00f,0.50f},
		/* Back */   {0.75f,1.00f}, {1.00f,1.00f}, {1.00f,0.50f}, {0.75f,0.50f},
		/* Right */  {0.50f,1.00f}, {0.75f,1.00f}, {0.75f,0.50f}, {0.50f,0.50f},
		/* Top */    {0.50f,0.50f}, {0.50f,0.00f}, {0.25f,0.00f}, {0.25f,0.50f},
		/* Bottom */ {0.75f,0.50f}, {0.75f,0.00f}, {0.50f,0.00f}, {0.50f,0.50f},
	};
	static const float pos[ORTHO_SKYBOX_VERTS][3] = {
		/* Front */  {-1,-1,-1}, { 1,-1,-1}, { 1, 1,-1}, {-1, 1,-1},
		/* Left */   {-1,-1, 1}, {-1,-1,-1}, {-1, 1,-1}, {-1, 1, 1},
		/* Back */   { 1,-1, 1}, {-1,-1, 1}, {-1, 1, 1}, { 1, 1, 1},
		/* Right */  { 1,-1,-1}, { 1,-1, 1}, { 1, 1, 1}, { 1, 1,-1},
		/* Top */    { 1, 1,-1}, { 1, 1, 1}, {-1, 1, 1}, {-1, 1,-1},
		/* Bottom */ { 1,-1,-1}, { 1,-1, 1}, {-1,-1, 1}, {-1,-1,-1},
	};
	struct VertexTextured v[ORTHO_SKYBOX_VERTS];
	GfxResourceID tex;
	float cx, cy, cz, s;
	int i;

	tex = EnvRenderer_GetSkyboxTex();
	if (!tex) return;

	cx = (float)World.Width  * 0.5f;
	cy = (float)World.Height * 0.5f;
	cz = (float)World.Length * 0.5f;
	s  = Math_SqrtF(cx*cx + cy*cy + cz*cz) * 1.5f;
	if (s < 50.0f) s = 50.0f;

	for (i = 0; i < ORTHO_SKYBOX_VERTS; i++) {
		v[i].x   = pos[i][0] * s + cx;
		v[i].y   = pos[i][1] * s + cy;
		v[i].z   = pos[i][2] * s + cz;
		v[i].Col = Env.SkyboxCol;
		v[i].U   = uvs[i][0];
		v[i].V   = uvs[i][1];
	}

	if (!ortho_skybox_vb) {
		ortho_skybox_vb = Gfx_CreateDynamicVb(VERTEX_FORMAT_TEXTURED, ORTHO_SKYBOX_VERTS);
	}

	Gfx_SetDepthTest(false);
	Gfx_SetDepthWrite(false);
	Gfx_BindTexture(tex);
	Gfx_SetVertexFormat(VERTEX_FORMAT_TEXTURED);
	Gfx_SetDynamicVbData(ortho_skybox_vb, v, ORTHO_SKYBOX_VERTS);
	Gfx_DrawVb_IndexedTris(ORTHO_SKYBOX_VERTS);
	Gfx_SetDepthTest(true);
	Gfx_SetDepthWrite(true);
}

/* 4x4 Bayer ordered dithering matrix (values 0–15) */
static const int bayer4x4[4][4] = {
	{  0,  8,  2, 10 },
	{ 12,  4, 14,  6 },
	{  3, 11,  1,  9 },
	{ 15,  7, 13,  5 }
};

/* For each pixel in a tile that matches the sentinel clear colour, replace it with
   a Bayer-dithered sky→fog gradient based on its vertical position in the full image. */
static void OrthoRender_ApplyGradient(BitmapCol* pixels,
	int tileW, int tileH, int px0, int py0, int imgH,
	BitmapCol sentinel,
	int skyR, int skyG, int skyB,
	int fogR, int fogG, int fogB)
{
	int row, col, py, px, bv, r, g, b;
	float t, bias;
	BitmapCol* p;

	for (row = 0; row < tileH; row++) {
		py   = py0 + row;
		t    = (imgH > 1) ? (float)py / (float)(imgH - 1) : 0.0f;
		p    = pixels + row * tileW;

		for (col = 0; col < tileW; col++, p++) {
			if (*p != sentinel) continue;

			px   = px0 + col;
			bv   = bayer4x4[py & 3][px & 3];
			bias = (bv - 7.5f) / 16.0f;   /* range ≈ -0.47 .. +0.47 */

			r = (int)(skyR + t * (fogR - skyR) + bias + 0.5f);
			g = (int)(skyG + t * (fogG - skyG) + bias + 0.5f);
			b = (int)(skyB + t * (fogB - skyB) + bias + 0.5f);

			if (r < 0) r = 0; else if (r > 255) r = 255;
			if (g < 0) g = 0; else if (g > 255) g = 255;
			if (b < 0) b = 0; else if (b > 255) b = 255;

			*p = BitmapCol_Make(r, g, b, 255);
		}
	}
}

static void OrthoRender_RenderTile(struct Matrix* view, struct Matrix* proj,
	cc_bool renderSky, cc_bool renderClouds) {
	struct Matrix mvp;

	Gfx.View       = *view;
	Gfx.Projection = *proj;
	Gfx_LoadMVP(view, proj, &mvp);
	FrustumCulling_CalcFrustumEquations(&mvp);

	/* Disable backface culling — ortho views need all faces visible */
	Gfx_SetFaceCulling(false);
	/* Force all chunk face directions to draw */
	MapRenderer_SetAllFacesVisible();

	/* Render skybox BEFORE map, with depth test/write off so it fills background.
	   Uses a large cube rendered with the ortho projection (not perspective)
	   so the skybox tiles correctly across multi-tile renders. */
	if (EnvRenderer_ShouldRenderSkybox() && !ortho_gradientBG) {
		OrthoRender_RenderSkybox();
	}

	/* Sky plane is NOT rendered during ortho — it uses Camera.CurrentPos.y
	   which is very far away, placing the plane between camera and map.
	   The clear color (Env.SkyCol) already provides the sky background. */

	/* Render map geometry */
	MapRenderer_RenderNormal(0);
	if (!ortho_gradientBG && !ortho_regionActive) {
		EnvRenderer_RenderMapSides();
		EnvRenderer_RenderMapEdges();
	}

	/* Render particles and non-player entities */
	Particles_Render(1.0f);
	Entities_RenderModels(0, 1.0f);

	/* Render clouds between opaque and translucent if below player and not gradient mode */
	if (renderClouds && !ortho_gradientBG) OrthoRender_RenderClouds();

	MapRenderer_RenderTranslucent(0);
}

void OrthoRender_Execute(void) {
	struct Matrix savedProj, savedView;
	struct Matrix view, proj;
	struct Bitmap tileBmp, outBmp;
	BitmapCol* tilePixels;
	Vec3 savedCamPos;
	struct Entity* e;
	float usePitch, useYaw;

	float orthoL, orthoR, orthoB, orthoT, orthoN, orthoF;
	float tLeft, tRight, tBottom, tTop;
	int imgW, imgH, tileW, tileH;
	int numTilesX, numTilesY, tx, ty;
	int px0, py0;
	int savedViewDist, savedMaxViewDist;
	int pxPerBlock, neededDist;
	cc_bool savedFog, renderSky, renderClouds;

	struct OrthoRowCtx rowCtx;
	cc_string filename; char fileBuffer[STRING_SIZE];
	cc_string path;     char pathBuffer[FILENAME_SIZE];
	struct cc_datetime now;
	cc_filepath raw_path;
	struct Stream stream;
	cc_result res;
	int yawDeg, pitchDeg;
	BitmapCol gradSentinel;
	int gradSkyR, gradSkyG, gradSkyB, gradFogR, gradFogG, gradFogB;

	OrthoRender_Requested = false;

	if (!World.Loaded) {
		Chat_AddRaw("&eOrthoRender: &cNo map loaded");
		return;
	}

	/* Resolve pitch and yaw: use player viewpoint if not specified */
	e = &Entities.CurPlayer->Base;
	usePitch = ortho_pitch >= 0.0f ? ortho_pitch : e->Pitch;
	useYaw   = ortho_yaw   >= 0.0f ? ortho_yaw   : e->Yaw;
	yawDeg   = (int)(useYaw   + 0.5f) % 360;
	pitchDeg = (int)(usePitch + 0.5f) % 360;

	/* Compute view extents */
	OrthoRender_ComputeView(useYaw, usePitch,
		&orthoL, &orthoR, &orthoB, &orthoT, &orthoN, &orthoF, &view);

	/* Compute image dimensions, scaling down if too large */
	pxPerBlock = ortho_pxPerBlock > 0 ? ortho_pxPerBlock : ORTHO_PX_PER_BLOCK;
	imgW = (int)((orthoR - orthoL) * pxPerBlock + 0.5f);
	imgH = (int)((orthoT - orthoB) * pxPerBlock + 0.5f);

	if (imgW <= 0 || imgH <= 0) {
		Chat_AddRaw("&eOrthoRender: &cFailed to compute image dimensions");
		return;
	}

	if (!Utils_EnsureDirectory("screenshots")) return;

	Chat_Add2("&eOrthoRender: &fRendering %i x %i image...", &imgW, &imgH);

	/* Save state */
	savedProj       = Gfx.Projection;
	savedView       = Gfx.View;
	savedCamPos     = Camera.CurrentPos;
	savedViewDist   = Game_ViewDistance;
	savedMaxViewDist = Game_MaxViewDistance;
	savedFog        = Gfx_GetFog();

	/* Increase view distance to cover entire map */
	neededDist = World.Width + World.Height + World.Length;
	if (neededDist < 4096) neededDist = 4096;
	Game_MaxViewDistance = neededDist;
	Game_ViewDistance    = neededDist;
	Event_RaiseVoid(&GfxEvents.ViewDistanceChanged);

	/* Disable fog so distance doesn't affect the render */
	Gfx_SetFog(false);

	if (ortho_gradientBG) {
		/* Use a sentinel clear colour (never appears in real renders) so background
		   pixels can be detected and replaced with the dithered gradient after readback. */
		Gfx_ClearColor(PackedCol_Make(1, 2, 3, 255));
		gradSentinel = BitmapCol_Make(1, 2, 3, 255);
		gradSkyR = PackedCol_R(Env.SkyCol); gradSkyG = PackedCol_G(Env.SkyCol); gradSkyB = PackedCol_B(Env.SkyCol);
		gradFogR = PackedCol_R(Env.FogCol); gradFogG = PackedCol_G(Env.FogCol); gradFogB = PackedCol_B(Env.FogCol);
	} else {
		Gfx_ClearColor(Env.SkyCol);
	}

	/* Render sky/clouds only when they appear below the camera in ortho view */
	renderSky    = Env.EdgeHeight   < (int)e->Position.y;
	renderClouds = Env.CloudsHeight < (int)e->Position.y;

	/* Build all chunk meshes before rendering.
	   In gradient mode, show outer faces of world-edge blocks so no gaps appear.
	   In region mode, clip per-block geometry to the region bounds. */
	if (ortho_gradientBG) {
		Builder_ShowEdgeFaces = true;
		MapRenderer_RefreshBorderChunks();
	}
	if (ortho_regionActive) {
		IVec3 rlo, rhi;
		rlo.x = min(ortho_regionP1.x, ortho_regionP2.x);
		rlo.y = min(ortho_regionP1.y, ortho_regionP2.y);
		rlo.z = min(ortho_regionP1.z, ortho_regionP2.z);
		rhi.x = max(ortho_regionP1.x, ortho_regionP2.x) + 1; /* exclusive */
		rhi.y = max(ortho_regionP1.y, ortho_regionP2.y) + 1;
		rhi.z = max(ortho_regionP1.z, ortho_regionP2.z) + 1;
		Builder_RegionMin      = rlo;
		Builder_RegionMax      = rhi;
		Builder_UseRegionBounds = true;
		Lighting_RegionMin      = rlo;
		Lighting_RegionMax      = rhi;
		Lighting_UseRegionBounds = true;
		/* Refresh all chunks so they rebuild with per-block clipping */
		MapRenderer_Refresh();
	}
	MapRenderer_BuildAllChunks();

	/* When rendering a region, also cull whole chunks outside the region AABB */
	if (ortho_regionActive) {
		IVec3 rlo, rhi;
		rlo.x = min(ortho_regionP1.x, ortho_regionP2.x);
		rlo.y = min(ortho_regionP1.y, ortho_regionP2.y);
		rlo.z = min(ortho_regionP1.z, ortho_regionP2.z);
		rhi.x = max(ortho_regionP1.x, ortho_regionP2.x);
		rhi.y = max(ortho_regionP1.y, ortho_regionP2.y);
		rhi.z = max(ortho_regionP1.z, ortho_regionP2.z);
		MapRenderer_FilterRenderChunks(rlo, rhi);
	}

	/* Cull entities outside the active region for all tile renders */
	OrthoRender_BeginEntityCull();

	/* Tile dimensions = window size */
	tileW     = Game.Width;
	tileH     = Game.Height;
	numTilesX = (imgW + tileW - 1) / tileW;
	numTilesY = (imgH + tileH - 1) / tileH;

	/* Allocate tile read buffer */
	tilePixels = (BitmapCol*)Mem_TryAlloc(tileW * tileH, BITMAPCOLOR_SIZE);
	if (!tilePixels) {
		Chat_AddRaw("&eOrthoRender: &cOut of memory for tile buffer");
		goto restore;
	}

	/* Phase 1: Render each tile and write raw pixel data to disk */
	for (ty = 0; ty < numTilesY; ty++) {
		for (tx = 0; tx < numTilesX; tx++) {
			px0 = tx * tileW;
			py0 = ty * tileH;

			/* Compute ortho bounds for this tile.
			   Image Y increases downward; view Y increases upward.
			   Always use full tile dimensions for viewport/rendering to avoid
			   partial-viewport issues on some drivers. Ortho bounds for partial
			   edge tiles extend beyond the map — extra area renders as sky. */
			tLeft   = orthoL + (float)px0        / pxPerBlock;
			tRight  = orthoL + (float)(px0 + tileW) / pxPerBlock;
			tTop    = orthoT - (float)py0        / pxPerBlock;
			tBottom = orthoT - (float)(py0 + tileH) / pxPerBlock;

			OrthoRender_CalcOrthoMatrix(&proj,
				tLeft, tRight, tBottom, tTop, orthoN, orthoF);

			Gfx_SetViewport(0, 0, tileW, tileH);
			Gfx_SetScissor(0, 0, tileW, tileH);
			Gfx_ClearBuffers(GFX_BUFFER_COLOR | GFX_BUFFER_DEPTH);

			OrthoRender_RenderTile(&view, &proj, renderSky, renderClouds);

			/* Read back full tile (always tileW x tileH) */
			Bitmap_Init(tileBmp, tileW, tileH, tilePixels);
			res = Gfx_ReadBackbuffer(&tileBmp);
			if (res) {
				Chat_AddRaw("&eOrthoRender: &cFailed to read backbuffer");
				Mem_Free(tilePixels);
				OrthoRender_CleanupTileFiles(numTilesY, numTilesX);
				goto restore;
			}

			/* Replace sentinel background pixels with dithered gradient */
			if (ortho_gradientBG)
				OrthoRender_ApplyGradient(tilePixels, tileW, tileH, px0, py0, imgH,
					gradSentinel, gradSkyR, gradSkyG, gradSkyB, gradFogR, gradFogG, gradFogB);

			/* Write tile pixels to a temporary raw file */
			res = OrthoRender_WriteTileRaw(ty, tx, tilePixels, tileW, tileH);
			if (res) {
				Chat_AddRaw("&eOrthoRender: &cFailed to write tile to disk");
				Mem_Free(tilePixels);
				OrthoRender_CleanupTileFiles(numTilesY, numTilesX);
				goto restore;
			}
		}
	}

	/* Phase 2: Iteratively build the final PNG from tile files.
	   Only a strip of imgW * tileH pixels is held in memory at once,
	   instead of the entire imgW * imgH output. */
	rowCtx.tileW     = tileW;
	rowCtx.tileH     = tileH;
	rowCtx.imgW      = imgW;
	rowCtx.imgH      = imgH;
	rowCtx.numTilesX = numTilesX;
	rowCtx.loadedTileRow = -1;
	rowCtx.error     = 0;
	rowCtx.tileBuf   = tilePixels;  /* reuse the tile buffer for reading */
	rowCtx.stripBuf  = (BitmapCol*)Mem_TryAlloc((cc_uint32)imgW * tileH, BITMAPCOLOR_SIZE);

	if (!rowCtx.stripBuf) {
		Chat_AddRaw("&eOrthoRender: &cOut of memory for image assembly strip");
		Mem_Free(tilePixels);
		OrthoRender_CleanupTileFiles(numTilesY, numTilesX);
		goto restore;
	}

	DateTime_CurrentLocal(&now);
	String_InitArray(filename, fileBuffer);
	String_Format3(&filename, "ortho_%p4-%p2-%p2", &now.year, &now.month, &now.day);
	String_Format3(&filename, "-%p2-%p2-%p2", &now.hour, &now.minute, &now.second);
	String_Format2(&filename, "_%ideg_%ideg.png", &pitchDeg, &yawDeg);

	String_InitArray(path, pathBuffer);
	String_Format1(&path, "screenshots/%s", &filename);

	Platform_EncodePath(&raw_path, &path);
	res = Stream_CreatePath(&stream, &raw_path);
	if (res) {
		Logger_IOWarn2(res, "creating", &raw_path);
		Mem_Free(rowCtx.stripBuf);
		Mem_Free(tilePixels);
		OrthoRender_CleanupTileFiles(numTilesY, numTilesX);
		goto restore;
	}

	/* Encode the PNG row-by-row via the row getter, which loads
	   tile strips from disk on demand */
	outBmp.width  = imgW;
	outBmp.height = imgH;
	outBmp.scan0  = NULL; /* not used — row getter provides rows */

	res = Png_Encode(&outBmp, &stream, OrthoRender_GetRow, false, &rowCtx);
	if (!res) res = rowCtx.error;
	if (res) {
		Logger_IOWarn2(res, "saving to", &raw_path);
		stream.Close(&stream);
		Mem_Free(rowCtx.stripBuf);
		Mem_Free(tilePixels);
		OrthoRender_CleanupTileFiles(numTilesY, numTilesX);
		goto restore;
	}

	res = stream.Close(&stream);
	if (res) {
		Logger_IOWarn2(res, "closing", &raw_path);
	} else {
		Chat_Add1("&eOrthoRender: &fSaved as %s", &filename);
	}

	/* Phase 3: Build 1/4-scale small copy from tile files */
	{
		struct Bitmap smallBmp;
		cc_string smallName; char smallNameBuf[STRING_SIZE];
		cc_string smallPath; char smallPathBuf[FILENAME_SIZE];
		cc_filepath smallRawPath;
		struct Stream smallStream;

		smallBmp.scan0 = NULL;
		rowCtx.loadedTileRow = -1;  /* reset for second pass */
		rowCtx.error = 0;

		OrthoRender_BuildSmallImage(&rowCtx, &smallBmp, imgW, imgH);

		if (smallBmp.scan0 && !rowCtx.error) {
			String_InitArray(smallName, smallNameBuf);
			String_Format3(&smallName, "ortho_%p4-%p2-%p2", &now.year, &now.month, &now.day);
			String_Format3(&smallName, "-%p2-%p2-%p2", &now.hour, &now.minute, &now.second);
			String_Format2(&smallName, "_%ideg_%ideg_small.png", &pitchDeg, &yawDeg);

			String_InitArray(smallPath, smallPathBuf);
			String_Format1(&smallPath, "screenshots/%s", &smallName);

			Platform_EncodePath(&smallRawPath, &smallPath);
			res = Stream_CreatePath(&smallStream, &smallRawPath);
			if (!res) {
				res = Png_Encode(&smallBmp, &smallStream, NULL, false, NULL);
				if (res) Logger_IOWarn2(res, "saving to", &smallRawPath);
				smallStream.Close(&smallStream);
				if (!res) Chat_Add1("&eOrthoRender: &fSaved small as %s", &smallName);
			} else {
				Logger_IOWarn2(res, "creating", &smallRawPath);
			}
		}
		if (smallBmp.scan0) Mem_Free(smallBmp.scan0);
	}

	Mem_Free(rowCtx.stripBuf);
	Mem_Free(tilePixels);

	/* Phase 4: Clean up temporary tile files */
	OrthoRender_CleanupTileFiles(numTilesY, numTilesX);

restore:
	/* Restore entity list before any other state changes */
	OrthoRender_EndEntityCull();

	/* Clear region selection after render */
	if (ortho_regionActive) OrthoRegion_FinishRender();

	/* If region bounds clipping was active, reset it and refresh affected chunks */
	if (Builder_UseRegionBounds) {
		Builder_UseRegionBounds  = false;
		Lighting_UseRegionBounds = false;
		MapRenderer_Refresh(); /* chunks rebuild normally over next few frames */
	}

	/* If edge faces were exposed for gradient mode, restore normal culling */
	if (ortho_gradientBG && Builder_ShowEdgeFaces) {
		Builder_ShowEdgeFaces = false;
		MapRenderer_RefreshBorderChunks();
	}

	/* Restore all state */
	Gfx.Projection = savedProj;
	Gfx.View       = savedView;
	Camera.CurrentPos = savedCamPos;
	Game_MaxViewDistance = savedMaxViewDist;
	Game_ViewDistance    = savedViewDist;
	Event_RaiseVoid(&GfxEvents.ViewDistanceChanged);

	Gfx_SetFaceCulling(false); /* MapRenderer re-enables as needed */
	if (savedFog) Gfx_SetFog(true);
	EnvRenderer_UpdateFog(); /* Restore fog color/density from environment */
	Gfx_OnWindowResize(); /* Restores viewport and handles Retina/HiDPI scaling */
	Gfx_SetScissor(0, 0, Game.Width, Game.Height);
	Gfx_LoadMatrix(MATRIX_PROJ, &savedProj);
	Gfx_LoadMatrix(MATRIX_VIEW, &savedView);
	Camera_UpdateProjection();
}


/*########################################################################################################################*
*---------------------------------------------------Command handler-------------------------------------------------------*
*#########################################################################################################################*/
static void OrthoRegionCommand_Execute(const cc_string* args, int argsCount) {
	static const cc_string subMark  = String_FromConst("mark");
	static const cc_string subClear = String_FromConst("clear");
	static const cc_string subLast  = String_FromConst("last");

	if (argsCount == 0) {
		OrthoRegion_EnterMarkingMode();
	} else if (String_CaselessEquals(&args[0], &subMark)) {
		/* /client orthoregion mark x1 y1 z1 x2 y2 z2 */
		int x1, y1, z1, x2, y2, z2;
		if (argsCount < 7 ||
			!Convert_ParseInt(&args[1], &x1) || !Convert_ParseInt(&args[2], &y1) ||
			!Convert_ParseInt(&args[3], &z1) || !Convert_ParseInt(&args[4], &x2) ||
			!Convert_ParseInt(&args[5], &y2) || !Convert_ParseInt(&args[6], &z2)) {
			Chat_AddRaw("&eOrthoRegion: &cUsage: mark <x1> <y1> <z1> <x2> <y2> <z2>");
			return;
		}
		IVec3_Set(ortho_regionP1, x1, y1, z1);
		IVec3_Set(ortho_regionP2, x2, y2, z2);
		OrthoRegion_ExitMarkingMode();
		ortho_regionActive  = true;
		ortho_lastP1        = ortho_regionP1;
		ortho_lastP2        = ortho_regionP2;
		ortho_hasLastRegion = true;
		OrthoRegion_UpdatePreview();
		Chat_AddRaw("&eOrthoRegion: &fRegion set. Run /client orthorender to render it.");
	} else if (String_CaselessEquals(&args[0], &subLast)) {
		if (!ortho_hasLastRegion) {
			Chat_AddRaw("&eOrthoRegion: &cNo previous region to restore.");
			return;
		}
		OrthoRegion_Clear();
		ortho_regionP1 = ortho_lastP1;
		ortho_regionP2 = ortho_lastP2;
		ortho_regionActive = true;
		OrthoRegion_UpdatePreview();
		Chat_AddRaw("&eOrthoRegion: &fPrevious region restored.");
	} else if (String_CaselessEquals(&args[0], &subClear)) {
		OrthoRegion_Clear();
		Chat_AddRaw("&eOrthoRegion: &fRegion cleared.");
	} else {
		Chat_AddRaw("&eOrthoRegion: &cUsage: [mark x y z x y z | last | clear]");
	}
}

static struct ChatCommand OrthoRegionCommand = {
	"OrthoRegion", OrthoRegionCommand_Execute,
	0,
	{
		"&a/client orthoregion [mark x y z x y z | last | clear]",
		"&eMarks a 3D region of the world to be used by /client orthorender.",
		"&eNo args: enter interactive mode (left-click or /client mark to set corners).",
		"&elast: restore the previously used region.",
		"&eclear: remove the marked region and preview."
	}
};

static void OrthoRenderCommand_Execute(const cc_string* args, int argsCount) {
	static const cc_string modePrefix  = String_FromConst("mode:");
	static const cc_string modeGrad    = String_FromConst("gradient");
	static const cc_string modeNormal  = String_FromConst("normal");
	static const cc_string widthPrefix = String_FromConst("width:");
	float pitch = -1.0f, yaw = -1.0f;
	cc_bool gradient = false;
	int blockWidth = 0;
	int i;
	cc_string val;
	for (i = 0; i < argsCount; i++) {
		if (String_CaselessStarts(&args[i], &modePrefix)) {
			val = String_UNSAFE_SubstringAt(&args[i], 5); /* skip "mode:" */
			if (String_CaselessEquals(&val, &modeGrad)) {
				gradient = true;
			} else if (!String_CaselessEquals(&val, &modeNormal)) {
				Chat_AddRaw("&eOrthoRender: &cunknown mode — use mode:normal or mode:gradient");
				return;
			}
			argsCount--; break;
		}
	}
	for (i = 0; i < argsCount; i++) {
		if (String_CaselessStarts(&args[i], &widthPrefix)) {
			val = String_UNSAFE_SubstringAt(&args[i], 6); /* skip "width:" */
			if (!Convert_ParseInt(&val, &blockWidth) || blockWidth < 1 || blockWidth > 64) {
				Chat_AddRaw("&eOrthoRender: &cwidth must be an integer between 1 and 64");
				return;
			}
			argsCount--; break;
		}
	}

	/* Remaining positional args: pitch [yaw] */
	if (argsCount >= 1) {
		if (!Convert_ParseFloat(&args[0], &pitch)) {
			Chat_AddRaw("&eOrthoRender: &cPitch must be a number (0-90)");
			return;
		}
		if (pitch < 0.0f || pitch > 90.0f) {
			Chat_AddRaw("&eOrthoRender: &cPitch must be between 0 and 90");
			return;
		}
	}

	if (argsCount >= 2) {
		if (!Convert_ParseFloat(&args[1], &yaw)) {
			Chat_AddRaw("&eOrthoRender: &cYaw must be a number (0-360)");
			return;
		}
		if (yaw < 0.0f || yaw > 360.0f) {
			Chat_AddRaw("&eOrthoRender: &cYaw must be between 0 and 360");
			return;
		}
	}

	ortho_pitch      = pitch;
	ortho_yaw        = yaw;
	ortho_gradientBG = gradient;
	ortho_pxPerBlock = blockWidth;
	OrthoRender_Requested = true;
}

static struct ChatCommand OrthoRenderCommand = {
	"OrthoRender", OrthoRenderCommand_Execute,
	0,
	{
		"&a/client orthorender [pitch] [yaw] [mode:{mode}] [width:N]",
		"&eRenders an orthographic view of the entire map (or active region) as a PNG.",
		"&ePitch: downward angle 0-90 (default: current view).",
		"&eYaw: rotation 0-360 (default: current view).",
		"&emode:normal (default) | mode:gradient (sky→fog bg). width:N = pixels/block (1-64)."
	}
};

static void OrthoMarkCommand_Execute(const cc_string* args, int argsCount) {
	struct Entity* e = &Entities.CurPlayer->Base;
	IVec3 pos;
	(void)args; (void)argsCount;

	if (!ortho_markingActive) {
		Chat_AddRaw("&eOrthoRegion: &cUse /client orthoregion first to start marking.");
		return;
	}
	pos.x = (int)Math_Floor(e->Position.x);
	pos.y = (int)Math_Floor(e->Position.y);
	pos.z = (int)Math_Floor(e->Position.z);
	OrthoRegion_OnMarkPoint(pos);
}

static struct ChatCommand OrthoMarkCommand = {
	"Mark", OrthoMarkCommand_Execute,
	0,
	{
		"&a/client mark",
		"&eMarks the player's current position as the next ortho region corner.",
		"&eUse after /client orthoregion to set P1 or P2 without clicking.",
		NULL, NULL
	}
};


/*########################################################################################################################*
*-------------------------------------------------Component registration--------------------------------------------------*
*#########################################################################################################################*/
static void OrthoRender_Init(void) {
	Commands_Register(&OrthoRenderCommand);
	Commands_Register(&OrthoRegionCommand);
	Commands_Register(&OrthoMarkCommand);
	Event_Register_(&WorldEvents.MapLoaded, NULL, OrthoRegion_OnMapLoaded);
}

struct IGameComponent OrthoRender_Component = {
	OrthoRender_Init /* Init */
};
