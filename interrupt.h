#include "BMI270_SensorAPI/bmi2_defs.h"
#ifndef INTERRUPT_H
#define INTERRUPT_H

#define NUM_FEAT 1

extern const struct bmi2_sens_int_config feat_int_map[NUM_FEAT];
extern int8_t setup_features(struct bmi2_dev*);
extern int8_t setup_interrupt_pin(struct bmi2_dev*);

#endif 
