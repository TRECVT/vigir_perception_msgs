# This message provides per scan mutable data
# in minimal and (slightly) lossy discretized
# representation. The representation is lossy
# because scan data is discretized into
# millimeters and intensities into one byte

uint16 INVALID = 65535

# Scan start timestamp
time stamp

# We split scans, so have to specify min/max angle
float32 angle_min
float32 angle_max

# Scan data discretized to millimeters.
# The following conventions apply:
# 0 < value < max_range: valid scan point
# max_range < value < 2*max_range: self filtered scan point
# INVALID: Invalid scan point
uint16[] scan

# Intensities scaled in reasonable TBD way
uint8[] intensities

# The pose of the sensor in fixed frame when
# first ray was retrieved 
geometry_msgs/Transform transform_first_ray

# The pose of the sensor in fixed frame when
# last ray was retrieved 
geometry_msgs/Transform transform_last_ray
