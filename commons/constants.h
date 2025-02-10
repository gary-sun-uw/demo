#ifndef CONSTANTS_H
#define CONSTANTS_H

#define PI 3.14159
#define G 9.80665

#define SAMPLE_PERIOD 5.3125 / 1000

static const n_interval_settings = 4;

static const IntervalSetting default_interval_setting = {{0,0,0}, 20};

static const IntervalSetting available_interval_settings[n_interval_settings] = {
    {{0,0,0}, 5},
    {{0,0,0}, 10},
    {{0,0,0}, 20},
    {{0,0,0}, 40}
};

static const interval_vector_min_similarity = 0.8;

#endif