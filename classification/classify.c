#include "classify.h"
#include "../commons/vec3d.h"
#include "../commons/constants.h"
#include <math.h>


ActionType classify(double average_vertical_position) {
    if (average_vertical_position < -0.2) {
        return SIT;
    } else if (average_vertical_position > 0.2) {
        return STAND;
    } else {
        return OTHER;
    }
}

ActionType tree_classify(Frame frames[5]){
    const Vec3d acc_average = vec3d_avg_unit_5(frames[0].acc, frames[1].acc, frames[2].acc, frames[3].acc, frames[4].acc);
    const double sim_to_sit = 1 - abs(acc_average.z) / 16000;
    const double sim_to_stand = 1 - abs(acc_average.x) / 16000;
    return sim_to_sit >= 0.7 ? SIT : sim_to_stand >= 0.7 ? STAND : OTHER;
}

IntervalSetting classify_interval_setting(
        Frame *frames,
        int length, 
        double min_similarity
    ) {

    unsigned int i, j;
    for (i = 0; i < n_interval_settings; i++) {
        const Vec3d ref = available_interval_settings[i].reference;
        int all_in = 1;

        for (j = 0; j < length; j++) {
            if (vec3d_vector_similarity(ref, frames[j].acc) < min_similarity) {
                all_in = 0;
                break;
            }
        }
        if (all_in) {
            return available_interval_settings[i];
        }
    }

    return (IntervalSetting){{0,0,0}, -1};
}
