#gesture
uint32 GESTURE_NOT_DETECTED = 0

#hand
uint32 GESTURE_HAND_SWEEP = 1
uint32 GESTURE_HAND_PUSH_PULL = 2

#body
uint32 GESTURE_BODY_MOVE = 101
uint32 GESTURE_BODY_TWIST = 102
uint32 GESTURE_BODY_LEAN = 103

#hand(s)
uint32 HAND_ONE = 1  
uint32 HAND_TWO = 2

#direction flags
uint32 DIR_X_POS = 1
uint32 DIR_X_NEG = 2
uint32 DIR_Y_POS = 3
uint32 DIR_Y_NEG = 4
uint32 DIR_Z_POS = 5
uint32 DIR_Z_NEG = 6
uint32 ROT_X_POS = 7
uint32 ROT_X_NEG = 8
uint32 ROT_Y_POS = 9
uint32 ROT_Y_NEG = 10
uint32 ROT_Z_POS = 11
uint32 ROT_Z_NEG = 12


uint32 type
uint32 hand_count
uint32 direction

#Only used if the type specified has some use of them 
float64[] vars

#Only used if the type specified has some use of them
geometry_msgs/Vector3[] vectors

#Only used if the type specified has some use of them
geometry_msgs/Transform[] transforms


