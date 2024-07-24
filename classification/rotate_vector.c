#include "rotate_vector.h"
#include <stdio.h>
#include "../uart.h"


void init_rotation_vector_state(RotateVectorState *state) {
    state->value = (Vec3d) {0, 0, 0};
}

RotateVectorStatus rotate_vector(RotateVectorState *state, Vec3d a, Vec3d rotation_vector) {
    state->value = vec3d_rotate_by_rotation_vector_assuming_things_that_arent_true(a, rotation_vector);

    if (state->base.debug) {
        uart_printf("%s_x:%f,%s_y:%f,%s_z:%f,", state->base.name, state->value.x, state->base.name, state->value.y, state->base.name, state->value.z);
    }
    
    return ROTATE_VECTOR_OK;
}