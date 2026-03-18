# Single UWB range measurement
std_msgs/Header header

string anchor_id      # Unique identifier of the anchor
float64 range         # Measured distance in meters
float64 std_dev       # Expected standard deviation of the measurement
float64 rssi          # Received Signal Strength Indicator (dBm)
float64 fpp           # First Path Power (dBm)
bool is_nlos          # NLOS detection flag (if hardware/sim supports it)
