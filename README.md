# Easy hand eye calibration for panda/fr3 and realsense camera
Movit-free hand eye calibration for panda and fr3. This repo is using the fanstastic tool [easy_hand](https://github.com/IFL-CAMP/easy_handeye). This repo just sets all the link and parameters for using the franka robot with the specified frames and uses the apriltag to track the objects. 

I have tested the code using ROS noetic, a real panda robot and a realsense camera.
If you are using a realsense, please install the driver using the command:

Install realsense drivers for ROS
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
If you need extra precision in the forward kinematics (that will also result in better results in the calibration of the camera) please give a look in the human friendly controllers on how to run the calibrated kinematic model. 

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
  cd ..   
  catkin build 
  source devel/setup.bash  
```

# Print the marker.
Print the pdf file marker.pdf. 
# If you have the camera attached at the end effector, place the marker in front and please run:
``` bash
  roslaunch franka_easy_handeye calibrate.launch  
```
# If you have the camera on the base, attach the marker at the end effector and run:

``` bash
  roslaunch franka_easy_handeye calibrate.launch  eye_on_base:=True 
```
Now, just make the robot compliant using the rqt reconfigure, and move the robot to a different position where you can see the marker in  front of the robot. When launching  the calibrate file, you have pop-up window that allows you to record the current pose and relative camera position. This is used later to compute the robot camera transformation.

# After calibrating the robot  

You can add the static transform in the launch file camera.launch. Be sure to place the right number in the position, here is the structure to use. You need also to specify the starting_frame that can be panda_hand if you are doing eye in hand or panda_link0 if you are doing eye on base.  

Example: 

```
<node pkg="tf" type="static_transform_publisher" name="camera_tf_publisher" args="translation/x translation/y translation/z rotation/x rotation/y rotation/z rotation/w starting_frame camera_color_optical_frame 100" />

```
