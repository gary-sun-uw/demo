#ifndef CLASSIFY_H
#define CLASSIFY_H

typedef enum ActionType {
	SIT,
	STAND,
	OTHER,
	LENGTH
} ActionType;

ActionType classify(double average_vertical_position);

#endif