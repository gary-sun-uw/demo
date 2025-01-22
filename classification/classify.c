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
    Frame frame = frames[0];
    return (frame.acc.y <= -1095.50) ? 
           ((frame.acc.y <= -7084.50) ? 
            ((frame.acc.z <= -14.50) ? OTHER : STAND) 
            : REST) 
           : SIT;
}
