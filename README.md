# franka_easy_handeye
Movit free hand eye calibration for panda and fr3. 

I have tested the code using ROS noetic, a real panda robot and a realsense d435 camera.

I am using the apriltag library to detect the apriltag in the camera image.
## Installation of the controller on the computer connected to the robot
Follow the instructions here ``` https://github.com/franzesegiovanni/franka_human_friendly_controllers ``` to install the controller on the computer connected to the robot.

Run the controller before starting the calibration 
```bash
  roslaunch franka_human_friendly_controllers cartesian_variable_impedance_controller.launch robot_ip:=ROBOT_IP
```
## Installation on the comoputer connected to the camera
### Install the real sense camera
```bash
  sudo apt-get install ros-noetic-realsense2-camera  
```

# Create a catking workspace
```bash
  mkdir calibration_ws  
  cd calibration_ws 
  mkdir src
  cd src 
  git clone --depth 1 https://github.com/AprilRobotics/apriltag.git  
  git clone --depth 1 https://github.com/AprilRobotics/apriltag_ros.git 
  git clone --depth 1 https://github.com/IFL-CAMP/easy_handeye.git                         
  git clone --depth 1 https://github.com/franzesegiovanni/franka_easy_handeye.git  
  cd ..   
  catkin build 
  source devel/setup.bash  
```
# Run the calibration
``` bash
  roslaunch franka_easy_handeye easy.launch  
```
