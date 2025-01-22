#ifndef FRAME_H
#define FRAME_H

#include "vec3d.h"

typedef struct Frame {
    Vec3d acc;
    Vec3d gyro;
} Frame;

#endif