std_msgs/Header header

int32 racing_line_id                     # ID of the RaceLine this map is based on -> for synchronization
bool friction_estimation_ignored_id   # if true, the friction estimation ignored the raceline and wants the velocity planing to disregard the raceline with this ID

float64[] mu_y                        # mu_x of each section
float64[] mu_x                        # mu_y of each section
float64[] section_end_distance        # end of each section as distance on RaceLine
int32[]   section_end_index           # end of each section as inedx on RaceLine

int32 updated_index_low               # lower apex index of updated intervall
int32 updated_index_upper             # upper apex index of updated intervall