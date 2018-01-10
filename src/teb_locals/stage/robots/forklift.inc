define laser ranger
(
  sensor
  (
    range_max 6.5
    fov 270.0
    samples 700
  )
  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
)

define forklift position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  odom_error [0.03 0.03 999999 999999 999999 0.02]
  #size [0.415 0.392 0.25]
  size [2.15 0.88 2.0]
  origin [0.9 0 0 0]
  gui_nose 1
  drive "car"
  laser(pose [ 1.35 0 0 0.000 ])
)

