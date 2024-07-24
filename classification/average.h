#ifndef AVERAGE_H
#define AVERAGE_H

#include <stdint.h>
#include "analyze.h"


typedef enum AverageStatus {
    AVERAGE_OK,
    AVERAGE_NOTOK
} AverageStatus;

typedef struct AverageState {
    AnalysisFunctor base;
    uint16_t n_samples; /* Number of samples */
    double value; /* Average value */
} AverageState;

void init_average_state(AverageState *state);

AverageStatus average(AverageState *state, double new_sample);

#endif