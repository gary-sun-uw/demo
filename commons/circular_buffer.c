#include "circular_buffer.h"
#include <stdlib.h>
#include "../classification/classify.h"


void init_circular_buffer(CircularBuffer *buffer, uint16_t size) {
    buffer->size = size;
    buffer->tail = 0;
    buffer->data = (double *)malloc(size * sizeof(double));
}

void add_to_circular_buffer(CircularBuffer *buffer, double value) {
    buffer->data[buffer->tail] = value;
    buffer->tail = (buffer->tail + 1) % buffer->size;
}

void free_circular_buffer(CircularBuffer *buffer) {
    free(buffer->data);
}


void init_actiontype_circular_buffer(ActionTypeCircularBuffer *buffer, uint16_t size) {
    buffer->size = size;
    buffer->tail = 0;
    buffer->data = (ActionType *)malloc(size * sizeof(ActionType));
}

void add_to_actiontype_circular_buffer(ActionTypeCircularBuffer *buffer, ActionType value) {
    buffer->data[buffer->tail] = value;
    buffer->tail = (buffer->tail + 1) % buffer->size;
}

void free_actiontype_circular_buffer(ActionTypeCircularBuffer *buffer) {
    free(buffer->data);
}
