#ifndef CC_ORTHORENDER_H
#define CC_ORTHORENDER_H
#include "Core.h"
CC_BEGIN_HEADER

/* 
Renders an orthographic projection of the entire map to a PNG file.
Triggered by the /client orthorender command.
Copyright 2014-2025 ClassiCube | Licensed under BSD-3
*/

struct IGameComponent;
extern struct IGameComponent OrthoRender_Component;

/* Whether an orthographic render has been requested */
extern cc_bool OrthoRender_Requested;
/* Executes the orthographic render (called from game loop) */
void OrthoRender_Execute(void);

CC_END_HEADER
#endif
