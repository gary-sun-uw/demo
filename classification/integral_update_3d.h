#ifndef INTEGRAL_UPDATE_3D_H
#define INTEGRAL_UPDATE_3D_H

#include <stdint.h>
#include "integral_update.h"
#include "../commons/vec3d.h"
#include "analyze.h"


typedef enum IntegralUpdate3dStatus {
    INTEGRAL_UPDATE_3D_OK,
    INTEGRAL_UPDATE_3D_NOTOK
} IntegralUpdate3dStatus;

typedef struct IntegralUpdate3dState {
    AnalysisFunctor base;
    Vec3d value;
    IntegralUpdateState x;
    IntegralUpdateState y;
    IntegralUpdateState z;
} IntegralUpdate3dState;

void init_integral_update_3d_state(IntegralUpdate3dState *state, double interval);
void free_integral_update_3d_state(IntegralUpdate3dState *state);

IntegralUpdate3dStatus integral_update_3d(IntegralUpdate3dState *state, Vec3d new_sample);


#endif