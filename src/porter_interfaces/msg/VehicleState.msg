uint8 OFFLINE=0
uint8 READY=1
uint8 BUSY=2
uint8 EMERGENCY_STOPPED=3
uint8 ERROR=4

std_msgs/Header header
string vehicle_id
uint8 state
string state_text
float32 battery_percent
float32 battery_voltage
geometry_msgs/PoseStamped pose
string current_command_id
bool nav2_ready
bool emergency_stopped
string locked_zone
float32 telemetry_age_sec
