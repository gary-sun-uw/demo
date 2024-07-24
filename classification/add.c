#include "add.h"
#include <stdio.h>
#include "../uart.h"


void init_add_state(AddState *state) {
    state->value = 0;
}

AddStatus add(AddState *state, double a, double b) {
    state->value = a + b;

    if (state->base.debug) {
        uart_printf("%s:%f,", state->base.name, state->value);
    }

    return ADD_OK;
}
