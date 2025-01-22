#ifndef CLASSIFY_H
#define CLASSIFY_H

#include "frame.h"

typedef enum ActionType {
	SIT,
	STAND,
	REST,
	OTHER,
	LENGTH
} ActionType;

ActionType classify(double average_vertical_position);

ActionType tree_classify(Frame frames[1]);

#endif