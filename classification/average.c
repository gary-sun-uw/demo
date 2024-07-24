#include "average.h"
#include "stdio.h"
#include "../uart.h"

void init_average_state(AverageState *state) {
    state->n_samples = 0;
    state->value = 0;
}

AverageStatus average(AverageState *state, double new_sample) {
    state->n_samples++;
    double alpha = 1/(double)state->n_samples;
    state->value += alpha*(new_sample - state->value);

    if (state->base.debug) {
        uart_printf("%s:%f,", state->base.name, state->value);
    }

    return AVERAGE_OK;
} 
