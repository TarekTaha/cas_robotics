define wheelchair position
(
  # actual size
  size     [0.9 0.6 1]
  origin   [0.3 0 0 0] # the wheelchairs's center of rotation is offset from its center of area / I considered it to be 30 cm from the mid Axes
  velocity [0 0 0 0]    # Initial Robot Velocity
  color "gray" # body colorx
  
  # GUI properties
  gui_nose 1 # draw a nose on the robot so we can see which way it points
  #gui_grid 1
  #gui_boundary 1
  #gui_movemask 1
  
  # estimated mass in KG
  mass 100.0 
  
  # determine how the model appears in various sensors
  obstacle_return 1
  laser_return 1
  ranger_return 1
  #blobfinder_return 1
  fiducial_return 1
  # differential steering model
  drive "diff"

  #body shape
  block
  (
	    points 4
	    point[0] [ 0.5 -0.3 ]
	    point[1] [-0.4 -0.3 ]
	    point[2] [-0.4  0.3 ]
	    point[3] [ 0.5  0.3 ]
	    #z[0 1] 
  )
)
