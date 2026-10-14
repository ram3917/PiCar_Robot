# One snapshot of the onboard sensors, published by the controller every 30 ms.
# This is the planner's whole input.
std_msgs/Header header

# Forward ultrasonic range in metres (REP 117): NaN = invalid reading, -inf = too close,
# +inf = nothing within range.
float32 range_m
# False when range_m is NaN or -inf.
bool range_valid

# Ultrasonic servo angle in degrees; 0 is straight ahead.
float32 servo_angle_deg

# The three downward greyscale ADC readings.
uint16[3] greyscale

# The motor power the controller last applied (left front, right front, left rear, right rear).
int8[4] last_motor_power

# Whether the robot is armed.
bool armed
