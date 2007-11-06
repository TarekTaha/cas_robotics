# Wheelchair 
define wheelchair position
(
  # actual size
  size    [0.9 0.6]
  origin  [0.3 0 0] # the wheelchairs's center of rotation is offset from its center of area / I considered it to be 30 cm from the mid Axes
  #origin   [0 0 0]
  velocity [0 0 0]    # Initial Robot Velocity
  color "gray" # body colorx
  
  # GUI properties
  gui_nose 1 # draw a nose on the robot so we can see which way it points
  gui_movemask 1
  boundary 1
  # estimated mass in KG
  mass 100.0 
  
  # determine how the model appears in various sensors
  obstacle_return 1
  laser_return 1
  ranger_return 1
  fiducial_return 1
  # differential steering model
  drive "diff"

  #body shape
  polygons 1
  polygon[0].points 4
  polygon[0].point[0] [ 0.5 -0.3 ]
  polygon[0].point[1] [-0.4 -0.3 ]
  polygon[0].point[2] [-0.4  0.3 ]
  polygon[0].point[3] [ 0.5  0.3 ]

  # uncomment this line if you want to model real pioneers with SICK
  # lasers, where the laser is taller than the robot
  # laser_return 0
)
# a Pioneer 2 or 3 in standard configuration
define Obstacle position
(
  size [0.8 0.8]
  origin [0 0.0 0]
  gui_nose 1
  mass 15.0 

  polygons 1
  polygon[0].points 4
  polygon[0].point[0] [  0.4  0.4 ]
  polygon[0].point[1] [  0.4  -0.4 ]
  polygon[0].point[2] [ -0.4  -0.4 ]
  polygon[0].point[3] [ -0.4  0.4 ]

  # differential steering model
  drive "diff"

 laser_return 1
) 
