#ifndef ADJUST_ACCERATION_H
#define ADJUST_ACCERATION_H

#include <stdint.h>
#include "../commons/vec3d.h"
#include "../commons/data_formats.h"
#include "analyze.h"

typedef enum AdjustAccelerationStatus {
    ADJUST_ACCELERATION_OK,
    ADJUST_ACCELERATION_NOTOK
} AdjustAccelerationStatus;

typedef struct AdjustAccelerationState {
    AnalysisFunctor base;
    Vec3d value;
} AdjustAccelerationState;

void init_adjust_acceleration_state(AdjustAccelerationState *state);

AdjustAccelerationStatus adjust_acceleration(AdjustAccelerationState *state, BMI2SensAxisData new_sample);

#endif