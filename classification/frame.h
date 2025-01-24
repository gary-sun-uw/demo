#ifndef FRAME_H
#define FRAME_H

#include "../commons/vec3d.h"

typedef struct Frame {
    Vec3d acc;
    Vec3d gyr;
} Frame;

#endif