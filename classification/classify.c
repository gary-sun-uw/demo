#include "classify.h"

ActionType classify(double average_vertical_position) {
    if (average_vertical_position < -0.2) {
        return SIT;
    } else if (average_vertical_position > 0.2) {
        return STAND;
    } else {
        return OTHER;
    }
}
