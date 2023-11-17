# Easy hand eye calibration for panda/fr3 and realsense camera
Movit-free hand eye calibration for panda and fr3. 

I have tested the code using ROS noetic, a real panda robot and a realsense d435 camera.

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

# Record trajectories in python
``` bash
  roscd franka_easy_handeye/scripts
  python record_calibration_trajectories.py  
```
# Run the calibration in Python
``` bash
  roscd franka_easy_handeye/scripts
  python execute_calibration.py
```
After the trajectory is over, remember to compute the transformation on the rqt gui.

# Limitations
The calibration was tested only with panda with realsense camera on the wrist. 
