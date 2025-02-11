#ifndef CLASSIFY_H
#define CLASSIFY_H

#include "frame.h"
#include "../commons/vec3d.h"

typedef enum ActionType {
	SIT,
	STAND,
	REST,
	OTHER,
	LENGTH
} ActionType;

typedef struct IntervalSetting {
	Vec3d reference;
	int interval_mins;
} IntervalSetting;

ActionType classify(double average_vertical_position);

ActionType tree_classify(Frame frames[5]);

/*
	Return the interval setting matching the frame data if such a setting exists.
	Otherwise return the previous setting.

	Frame data matches a setting when all of it's vector are similar to the reference.
*/
IntervalSetting classify_interval_setting(
    Frame *frames,
    int length, 
    double min_similarity
);

#endif
