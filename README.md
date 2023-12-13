To use this package, clone the code into the src directory of your ROS workspace and build using colcon. There are three seperate launch files.

apriltag_launch: launch necessary processes for baxter to see and track apriltags<br>
facedetect_launch: launch necessary processes for baxter to see and track faces<br>
sim_launch: launch the gazebo simulation for baxter<br>

To run the simulation for baxter, it is also necessary to clone and build the ros_gz_sim package into the src directory of your ROS workspace.<br>
You will also need to run "export GZ_SIM_RESOURCE_PATH=/path/to/baxter/directory" so that gazebo is able to find the simulation files.

To run the apriltag tracking process for baxter, you will need to have the apriltag_ros package installed<br>
To run the face tracking process, you will need to have cv2 (openCV) installed
