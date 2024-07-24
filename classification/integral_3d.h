#ifndef INTEGRAL_3D_H
#define INTEGRAL_3D_H

#include <stdint.h>
#include "integral.h"
#include "../commons/vec3d.h"
#include "analyze.h"


typedef enum Integral3dStatus {
    INTEGRAL_3D_OK,
    INTEGRAL_3D_NOTOK
} Integral3dStatus;

typedef struct Integral3dState {
    AnalysisFunctor base;
    Vec3d value;
    IntegralState x;
    IntegralState y;
    IntegralState z;
} Integral3dState;

void init_integral_3d_state(Integral3dState *state, double interval);
void free_integral_3d_state(Integral3dState *state);

Integral3dStatus integral_3d(Integral3dState *state, Vec3d new_sample);


#endif