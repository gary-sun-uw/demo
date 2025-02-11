#ifndef CONSTANTS_H
#define CONSTANTS_H

#define PI 3.14159
#define G 9.80665

#define SAMPLE_PERIOD 5.3125 / 1000

static const unsigned int n_interval_settings = 4;

static const IntervalSetting default_interval_setting = {{0,0,0}, 20};

static const IntervalSetting available_interval_settings[n_interval_settings] = {
    {{16620,-726,-575}, 5},//16620,-726,-575
    {{-16013,-663,-410}, 10},//-16013,-663,-410
    {{537,-16659,-442}, 20},//537,-16659,-442
    {{-31,16030,-664}, 40}//-31,16030,-664
};

static const double interval_vector_min_similarity = 0.8;

#endif
