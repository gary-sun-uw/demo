#ifndef VECTOR_COMPONENT_H
#define VECTOR_COMPONENT_H

#include "../commons/vec3d.h"
#include "analyze.h"


typedef enum VectorComponentStatus {
    VECTOR_COMPONENT_OK,
    VECTOR_COMPONENT_NOTOK
} VectorComponentStatus;

typedef struct VectorComponentState {
    AnalysisFunctor base;
    double value;
} VectorComponentState;

void init_vector_component_state(VectorComponentState *state);

VectorComponentStatus vector_component(VectorComponentState *state, Vec3d a, Vec3d b);


#endif