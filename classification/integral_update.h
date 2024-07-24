#ifndef INTEGRAL_UPDATE_H
#define INTEGRAL_UPDATE_H

#include <stdint.h>
#include "../commons/circular_buffer.h"
#include "analyze.h"

typedef enum IntegralUpdateStatus {
    INTEGRAL_UPDATE_OK,
    INTEGRAL_UPDATE_NOTOK
} IntegralUpdateStatus;

typedef struct IntegralUpdateState {
    AnalysisFunctor base;
    uint16_t n_samples;
    double value;
    double interval;
    CircularBuffer samples;
} IntegralUpdateState;

void init_integral_update_state(IntegralUpdateState *state, double interval);
void free_integral_update_state(IntegralUpdateState *state);

IntegralUpdateStatus integral_update(IntegralUpdateState *state, double new_sample);

#endif
