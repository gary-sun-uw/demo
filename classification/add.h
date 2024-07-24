#ifndef ADD_H
#define ADD_H

#include <stdint.h>
#include "analyze.h"


typedef enum AddStatus {
    ADD_OK,
    ADD_NOTOK
} AddStatus;

typedef struct AddState {
    AnalysisFunctor base;
    double value;
} AddState;

void init_add_state(AddState *state);

AddStatus add(AddState *state, double a, double b);


#endif