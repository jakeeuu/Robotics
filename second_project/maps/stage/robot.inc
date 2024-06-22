define laser ranger
(
  sensor
  (
    # pose [ -0.1 0.0 -0.11 0.0 ]  # Position and orientation of the laser sensor
    # size [ 0.1 0.1 0.1 ]         # Physical dimensions of the sensor
    range [ 0.2 150 ]            # Minimum and maximum range in meters
    fov 360.0                    # Field of view in degrees
    samples 1440                 # Number of samples within the FOV
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]        # Physical dimensions for visualization
)


define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]       # Initial position and orientation of the robot

  localization "odom"            # Localization method (e.g., "odom" for odometry)

  odom_error [0.0 0.0 0.0 0.0 ]  # Odometry error parameters (x, y, z, yaw)

  size [ 0.612 0.580 0.40 ]      # Physical dimensions of the robot
  origin [ 0.0 0.0 0.0 0.0 ]     # Origin of the robot model
  gui_nose 1                     # GUI visualization option
  drive "omni"                   # Drive type (e.g., "omni" for omnidirectional)
  color "grey"                   # Color of the robot
  
  laser()                        # Integrating the previously defined laser sensor
)