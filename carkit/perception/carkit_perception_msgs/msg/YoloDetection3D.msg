uint8 TRAFFIC_LIGHT_UNKNOWN=0
uint8 TRAFFIC_LIGHT_RED=1
uint8 TRAFFIC_LIGHT_YELLOW=2
uint8 TRAFFIC_LIGHT_GREEN=3

int32 class_id
string class_name
float32 confidence

float32 x
float32 y
float32 z
bool position_valid

float32 bbox_x_min
float32 bbox_y_min
float32 bbox_x_max
float32 bbox_y_max

uint8 traffic_light_color
