#ifndef CC_BUILDER_H
#define CC_BUILDER_H
#include "Core.h"
#include "Vectors.h"
CC_BEGIN_HEADER

/* 
Converts a 16x16x16 chunk into a mesh of vertices
  NormalMeshBuilder:
    Implements a simple chunk mesh builder, where each block face is a single colour
    (whatever lighting engine returns as light colour for given block face at given coordinates)

Copyright 2014-2025 ClassiCube | Licensed under BSD-3
*/
struct ChunkInfo;
struct IGameComponent;
extern struct IGameComponent Builder_Component;

extern int Builder_SidesLevel, Builder_EdgeLevel;
/* Whether smooth/advanced lighting mesh builder is used. */
extern cc_bool Builder_SmoothLighting;
/* When true, outer faces of blocks at the world boundary are NOT culled. */
extern cc_bool Builder_ShowEdgeFaces;
/* When true, blocks outside [RegionMin, RegionMax) produce no geometry. */
extern cc_bool Builder_UseRegionBounds;
extern IVec3   Builder_RegionMin, Builder_RegionMax; /* inclusive min, exclusive max */

/* Builds the mesh of vertices for the given chunk. */
void Builder_MakeChunk(struct ChunkInfo* info);

void Builder_ApplyActive(void);

CC_END_HEADER
#endif
