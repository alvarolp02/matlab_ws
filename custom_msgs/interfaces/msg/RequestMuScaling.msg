std_msgs/Header header

bool force_now # if true it will not wait until data from the current lap is available and predict with what is availiable now (for predictMUafterPass == false)

int32 section # section index of when Request is send
int32 lap     # lap index of when Request is send

float64 section_apex_curvature # curvature of the section apex
float64 tire_temp              # tire temperature on release (not in use)
float64 humidity_factor        # humidity factor on release (not in use)