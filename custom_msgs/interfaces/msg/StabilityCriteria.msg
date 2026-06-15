std_msgs/Header header
uint8 lap
uint8 section
uint8 total_number_of_sections

float64 mu_scaling 
float64 estimated_mu_scaling_cost_function
float64 side_slip_front
float64 side_slip_rear 
float64 side_slip_difference
float64 path_error
float64 yaw_rate_error
float64[] used_mu

float64 section_curvature
float64 tire_temp
float64 humidity_factor

bool path_closed             # if true the path is closed and we want to assume that section is stable
bool updated_sections        # if true section have been changed and we need to reset stable section values to uncertain