#include "analyze.h"
#include "../commons/vec3d.h"
#include "classification/integral_update.h"
#include "classify.h"
#include "average.h"
#include "adjust_acceleration.h"
#include "adjust_gyro.h"
#include "integral.h"
#include "integral_3d.h"
#include "integral_update_3d.h"
#include "rotate_vector.h"
#include "vector_component.h"
#include "add.h"
#include "../commons/vec3d.h"
#include "../commons/constants.h"
#include <stdio.h>
#include <string.h>
#include <math.h>


void init_analysis_functor(AnalysisFunctor *functor, uint8_t debug, char name[ANALYSIS_FUNCTOR_NAME_LENGTH]) {
    functor->debug = debug;
    strcpy(functor->name, name);
}

AnalysisStatus analyze(
        BMI2SensData data[],
        uint16_t data_length,
        BMI2SensData g,
        uint8_t debug,
        AnalysisResult *out) {

    // AdjustAccelerationState acc_adj;
    // init_analysis_functor(&acc_adj.base, debug, "acc_adj");
    // init_adjust_acceleration_state(&acc_adj);

    // AdjustAccelerationState g_adj;
    // init_analysis_functor(&g_adj.base, 0, "g_adj");
    // init_adjust_acceleration_state(&g_adj);
    // adjust_acceleration(&g_adj, g.acc);

    // RotateVectorState g_rot;
    // init_analysis_functor(&g_rot.base, debug, "g_rot");
    // init_rotation_vector_state(&g_rot);
    // g_rot.value = g_adj.value;

    // AdjustGyroState gyr_adj;
    // init_analysis_functor(&gyr_adj.base, debug, "gyr_adj");
    // init_adjust_gyro_state(&gyr_adj);

    // IntegralUpdate3dState rot_vec;
    // init_analysis_functor(&rot_vec.base, debug, "rot_vec");
    // init_integral_update_3d_state(&rot_vec, SAMPLE_PERIOD);

    // VectorComponentState vert_acc;
    // init_analysis_functor(&vert_acc.base, debug, "vert_acc");
    // init_vector_component_state(&vert_acc);

    // IntegralState vert_vel;
    // init_analysis_functor(&vert_vel.base, debug, "vert_vel");
    // init_integral_state(&vert_vel, SAMPLE_PERIOD);

    // IntegralState vert_pos;
    // init_analysis_functor(&vert_pos.base, debug, "vert_pos");
    // init_integral_state(&vert_pos, SAMPLE_PERIOD);

    // AverageState avg_vert_pos;
    // init_analysis_functor(&avg_vert_pos.base, debug, "avg_vert_pos");
    // init_average_state(&avg_vert_pos);

    int i = 0;
    // for (i = 0; i < data_length; i++) {
    //     // printf("%d \n", i);
    //     adjust_acceleration(&acc_adj, data[i].acc);
    //     adjust_gyro(&gyr_adj, data[i].gyr);
    //     integral_update_3d(&rot_vec, gyr_adj.value);
    //     // rotate_vector(&g_rot, g_rot.value, rot_vec.value);
    //     vector_component(&vert_acc, acc_adj.value, g_rot.value);
    //     integral(&vert_vel, vert_acc.value);
    //     integral(&vert_pos, vert_vel.value);
    //     average(&avg_vert_pos, vert_pos.value);

    //     // if ((i > (10)) && (abs(vert_pos.value) < 0.1)) {
    //     //     printf("vert pos: %d\n", (int)(vert_pos.value * 1000));
    //     //     out->classification = OTHER;
    //     //     return ANALYSIS_OK;
    //     // }
    // }

    // //printf("vert pos: %d\n", (int)(vert_pos.value * 1000));

    Vec3d acc_adj;
    Vec3d g_adj;
    double vert_acc;
    double vert_vel = 0;
    double vert_pos = 0;

    g_adj.x = (double)g.acc.x / 16384 * 9.80665;
    g_adj.y = (double)g.acc.y / 16384 * 9.80665;
    g_adj.z = (double)g.acc.z / 16384 * 9.80665;

    for (i = 0; i < data_length; i++) {
        acc_adj.x = (double)data[i].acc.x / 16384 * 9.80665;
        acc_adj.y = (double)data[i].acc.y / 16384 * 9.80665;
        acc_adj.z = (double)data[i].acc.z / 16384 * 9.80665;
        vert_acc = vec3d_component(vec3d_sub_vectors(acc_adj, g_adj), g_adj, 0.0001);
        vert_vel += vert_acc * SAMPLE_PERIOD;
        vert_pos += vert_vel * SAMPLE_PERIOD;
    }

    /* Classify. */
    // out->classification = classify(avg_vert_pos.value);
    out->classification = classify(vert_pos);

    // free_integral_update_3d_state(&rot_vec);
    // free_integral_state(&vert_vel);
    // free_integral_state(&vert_pos);
    
    return ANALYSIS_OK;
}
