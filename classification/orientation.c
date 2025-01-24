#include "orientation.h"

#include "../commons/vec3d.h"

static Vec3d sitting = {1.99, 9.12, -0.01};
static Vec3d standing = {-9.68, 0.92, 0.00};

ActionType classify_orientation(BMI2SensAxisData data[], uint32_t length) {
    Vec3d data_as_vecs[5];
    int i;
    for (i = 0; i < 5; i++) {
        data_as_vecs[i] = bmisensdata_to_vec3d(data[i]);
    }

    Vec3d avg = vec3d_avg_unit_5(data_as_vecs[0], data_as_vecs[1], data_as_vecs[2], data_as_vecs[3], data_as_vecs[4]);

    if (vec3d_vector_similarity(avg, sitting) > 0.2) {
        return SIT;
    } else if (vec3d_vector_similarity(avg, standing) > 0.2) {
        return STAND;
    }
     else if (vec3d_vector_similarity(avg, sitting) > 0.05) {
        return REST;
    } 
    else {
        return OTHER;
    }
}
