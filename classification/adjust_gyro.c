#include "adjust_gyro.h"
#include "../commons/constants.h"
#include <math.h>
#include <stdio.h>
#include "../uart.h"


void init_adjust_gyro_state(AdjustGyroState *state) {
    state->value.x = 0;
    state->value.y = 0;
    state->value.z = 0;
}

static double convert_axis_value(int16_t value) {
    return (double)value / 16384 * (2*M_PI/360) * 1000; /* IDK why this 1000 factor works(?) */
}

AdjustGyroStatus adjust_gyro(AdjustGyroState *state, BMI2SensAxisData new_sample) {
    state->value.x = convert_axis_value(new_sample.x);
    state->value.y = convert_axis_value(new_sample.y);
    state->value.z = convert_axis_value(new_sample.z);

    if (state->base.debug) {
        uart_printf("%s_x:%f,%s_y:%f,%s_z:%f,", state->base.name, state->value.x, state->base.name, state->value.y, state->base.name, state->value.z);
    }

    return ADJUST_GYRO_OK;
}
