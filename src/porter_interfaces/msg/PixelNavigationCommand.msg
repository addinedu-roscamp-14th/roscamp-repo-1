std_msgs/Header header
string command_id
string predecessor_command_id
string requested_vehicle_id
string zone_id
string mode
bool queue_if_busy
geometry_msgs/Point target_pixel
geometry_msgs/Point heading_pixel
# True when the client's current camera frame shows no vehicle occupying
# this zone's target area. Used only to auto-clear a zone lock whose owner's
# telemetry has already gone stale (see fleet_dispatcher._zone_unknown) -
# never to bypass the lock for an owner that is still online.
bool zone_visually_empty
