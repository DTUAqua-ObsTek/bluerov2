std_msgs/Header header

# Example:
# {"carrier_frequency":150,"compass":"onboard","gps":"static","max_range":100,"static_lat":0,"static_lon":0,"static_orientation":0,"use_external_depth":true}

uint16 carrier_frequency	# Carrier frequency (kHz)
string compass			# Compass provider setting [ onboard, static, external ]
string gps			# GPS provider setting
int16 range_max_x		# Max range (meters)
int16 range_max_y		# Max range (meters)
int16 range_max_z		# Max range (meters)
int16 range_min_x		# Min range (meters)
int16 range_min_y		# Min range (meters)
float32 static_lat		# Latitude to use in static mode
float32 static_lon		# Longitude to use in static mode
uint16 static_orientation	# Orientation/compass reading to use in static mode (degrees)
bool use_external_depth		# Use external depth. Required to be true when using Locator-A1.
