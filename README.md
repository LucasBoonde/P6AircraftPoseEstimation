This is a copy of the ROS workspace which was implemented for ROB664's 6th semester project:  Autonomous Cone Placement Around Aircraft Using Keypoint Detection. 

It is separated into 2 packages. The vision package which contains the XYZSubscriber script. The script which takes a disparity, extrinsic file and a rectified luma image as input and uses the 
trained yolo network to generate keypoints, which it then sends through a ROS publisher to the other package which contains all turtlebot related code. This package controls the lidar and control parameters for the turtlebot. It collects coordinates from the
Vision package and sents them to the turtlebot to initiate movement. 

The turtlebot is initiated in a ROS terminal and then the subscriber script is initiated. Then the Vision script is ran in a separate ROS terminal and it will publish the coordinates once it is finnished.
