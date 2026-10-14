# A timed sequence of motor powers, published by the motion planner at 5-10 Hz.
# The controller plays it from the moment it arrives, so header.stamp is informational.
std_msgs/Header header

# Duration of one step in seconds. Each step is a keyframe at the start of its slot; the
# controller ramps linearly between steps and holds the last one. v0.2 sends 10 steps of 0.2 s.
float32 step_s

MotorPower[] steps
