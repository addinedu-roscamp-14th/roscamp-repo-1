uint8 VESSEL_ARRIVED=1
uint8 VESSEL_DEPARTED=2

std_msgs/Header header
string event_id
uint8 event_type
string event_type_text
bool active
float32 confidence
string details_json
