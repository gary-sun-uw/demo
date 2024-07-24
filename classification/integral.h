#ifndef INTEGRAL_H
#define INTEGRAL_H

#include <stdint.h>
#include "../commons/circular_buffer.h"
#include "analyze.h"


typedef enum IntegralStatus {
    INTEGRAL_OK,
    INTEGRAL_NOTOK
} IntegralStatus;

typedef struct IntegralState {
    AnalysisFunctor base;
    uint16_t n_samples;
    double value;
    double interval;
    CircularBuffer samples;
} IntegralState;

void init_integral_state(IntegralState *state, double interval);
void free_integral_state(IntegralState *state);

IntegralStatus integral(IntegralState *state, double new_sample);

#endif