# Easy hand eye calibration for panda/fr3 and realsense camera
Movit-free hand eye calibration for panda and fr3. 

I have tested the code using ROS noetic, a real panda robot and a realsense d435 camera.
If you are using a realsense, please install the driver using the command:
```
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
```
I am using the apriltag library to detect the apriltag in the camera image.
## Installation of the controller on the computer connected to the robot
Follow the instructions here ``` https://github.com/franzesegiovanni/franka_human_friendly_controllers ``` to install the controller on the computer connected to the robot.

Run the controller before starting the calibration 
```bash
  roslaunch franka_human_friendly_controllers cartesian_variable_impedance_controller.launch robot_ip:=ROBOT_IP
```

# Create a catking workspace
```bash
  mkdir calibration_ws  
  cd calibration_ws 
  mkdir src
  cd src
  git clone --branch ros1-legacy https://github.com/IntelRealSense/realsense-ros.git
  git clone --depth 1 https://github.com/AprilRobotics/apriltag.git  
  git clone --depth 1 https://github.com/AprilRobotics/apriltag_ros.git 
  git clone --depth 1 https://github.com/IFL-CAMP/easy_handeye.git                         
  git clone --depth 1 https://github.com/franzesegiovanni/franka_easy_handeye.git
  git clone --depth 1 https://github.com/platonics-delft/panda-ros-py.git
  cd ..   
  catkin build 
  source devel/setup.bash  
```

# Print, measure, and place the marker in front of the robot.
Print the pdf file marker.pdf. 
# Run the calibration
``` bash
  roslaunch franka_easy_handeye calibrate.launch  
```

Now, just make the robot compliant using the rqt reconfigure, and move the robot to a different position where you can see the marker in  front of the robot. When launching  the calibrate file, you have pop-up window that allows you to record the current pose and relative camera position. This is used later to compute the robot camera transformation.
