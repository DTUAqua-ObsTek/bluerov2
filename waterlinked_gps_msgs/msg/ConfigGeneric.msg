std_msgs/Header header

# Example:
# {"carrier_frequency":150,"compass":"onboard","gps":"static","max_range":100,"static_lat":0,"static_lon":0,"static_orientation":0,"use_external_depth":true}

bool antenna_enabled        # Is antenna enabled?
uint32 channel              # Frequency channel
string compass			    # Compass provider setting [ onboard, static, external ]
string environment          # Type of water environment [ closed, openwater ]
bool external_pps_enabled   # Is PPS enabled?
string gps			        # GPS provider setting [ onboard, static, external ]
bool imu_vehicle_enabled    # Is vehicle IMU fusion provided
string locator_type         # Type of acoustic locator on vehicle
int16 range_max_x		    # Max range (meters)
int16 range_max_y		    # Max range (meters)
int16 range_max_z		    # Max range (meters)
int16 range_min_x		    # Min range (meters)
int16 range_min_y		    # Min range (meters)
int16 search_direction
int16 search_radius
int16 search_sector
int16 speed_of_sound
float32 static_lat		    # Latitude to use in static mode
float32 static_lon		    # Longitude to use in static mode
uint16 static_orientation	# Orientation/compass reading to use in static mode (degrees)