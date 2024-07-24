#ifndef ADJUST_GYRO_H
#define ADJUST_GYRO_H

#include <stdint.h>
#include "../commons/vec3d.h"
#include "../commons/data_formats.h"
#include "analyze.h"

typedef enum AdjustGyroStatus {
    ADJUST_GYRO_OK,
    ADJUST_GYRO_NOTOK
} AdjustGyroStatus;

typedef struct AdjustGyroState {
    AnalysisFunctor base;
    Vec3d value;
} AdjustGyroState;

void init_adjust_gyro_state(AdjustGyroState *state);

AdjustGyroStatus adjust_gyro(AdjustGyroState *state, BMI2SensAxisData new_sample);

#endif
