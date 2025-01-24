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

ActionType tree_classify(Frame frames[1]){
    if (frames[4].acc.z <= -13379.00) {
        return REST;
    } else {
        if (frames[1].acc.z <= 10642.00) {
            return STAND;
        } else {
            if (frames[1].gyr.x <= -1936.50) {
                return STAND;
            } else {
                return SIT;
            }
        }
    }
}
