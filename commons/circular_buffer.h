#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>
#include "../classification/classify.h"

typedef struct CircularBuffer {
    uint16_t size;
    uint16_t tail;
    double *data;
} CircularBuffer;

void init_circular_buffer(CircularBuffer *buffer, uint16_t size);
void add_to_circular_buffer(CircularBuffer *buffer, double value);
void free_circular_buffer(CircularBuffer *buffer);

typedef struct ActionTypeCircularBuffer {
    uint16_t size;
    uint16_t tail;
    ActionType *data;
} ActionTypeCircularBuffer;

void init_actiontype_circular_buffer(ActionTypeCircularBuffer *buffer, uint16_t size);
void add_to_actiontype_circular_buffer(ActionTypeCircularBuffer *buffer, ActionType value);
void free_actiontype_circular_buffer(ActionTypeCircularBuffer *buffer);

#endif
