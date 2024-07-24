#include "integral_update_3d.h"
#include "../commons/vec3d.h"
#include <string.h>
#include <stdio.h>


void init_integral_update_3d_state(IntegralUpdate3dState *state, double interval) {
    state->value = (Vec3d){0, 0, 0};

    state->x.base = state->base;
    strcat(state->x.base.name, "_x");
    init_integral_update_state(&state->x, interval);

    state->y.base = state->base;
    strcat(state->y.base.name, "_y");
    init_integral_update_state(&state->y, interval);

    state->z.base = state->base;
    strcat(state->z.base.name, "_z");
    init_integral_update_state(&state->z, interval);
}

void free_integral_update_3d_state(IntegralUpdate3dState *state) {
    free_integral_update_state(&state->x);
    free_integral_update_state(&state->y);
    free_integral_update_state(&state->z);
}

IntegralUpdate3dStatus integral_update_3d(IntegralUpdate3dState *state, Vec3d new_sample) {
    IntegralUpdateStatus status;
    
    status = integral_update(&state->x, new_sample.x);
    if (status == INTEGRAL_UPDATE_NOTOK) {
        return INTEGRAL_UPDATE_3D_NOTOK;
    }
    status = integral_update(&state->y, new_sample.y);
    if (status == INTEGRAL_UPDATE_NOTOK) {
        return INTEGRAL_UPDATE_3D_NOTOK;
    }
    status = integral_update(&state->z, new_sample.z);
    if (status == INTEGRAL_UPDATE_NOTOK) {
        return INTEGRAL_UPDATE_3D_NOTOK;
    }

    state->value = (Vec3d){
        state->x.value,
        state->y.value,
        state->z.value
    };

    return INTEGRAL_UPDATE_3D_OK;
}
