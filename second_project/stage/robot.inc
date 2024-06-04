####TODOOOOOOOOOOOOOO TUTTO

define kinect ranger
(
  sensor
  (
    pose [ -0.1 0.0 -0.11 0.0 ]
    size [0.1 0.1 0.1 ]
    range  [0.2 150]
    fov 360.0
    samples 1440
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  
  localization "odom"


  odom_error [0.2 0.2 0.0 0.3 ]

  size [ .612 0.580 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "grey"
  
  kinect()
)