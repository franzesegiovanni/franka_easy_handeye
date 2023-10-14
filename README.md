# franka_easy_handeye
Movit free hand eye calibration for panda and fr3. 

I have tested the code using ROS noetic, a real panda robot and a realsense d435 camera.

I am using the apriltag library to detect the apriltag in the camera image.

## Installation
### Install the real sense camera
``` sudo apt-get install ros-noetic-realsense2-camera ```


# Create a catking workspace
``` mkdir calibration_ws ```

``` cd calibration_ws ```

``` mkdir src ```

``` cd src ```

``` git clone https://github.com/AprilRobotics/apriltag.git ```  # Clone Apriltag library

``` git clone https://github.com/AprilRobotics/apriltag_ros.git ```  # Clone Apriltag ROS 
wrapper                              

``` git clone https://github.com/franzesegiovanni/franka_easy_handeye.git ```

``` cd .. ```

``` rosdep install --from-paths src --ignore-src -r -y ```  # Install any missing packages

``` catkin build ```    # Build all packages in the workspace (catkin_make_isolated will work also)

``` source devel/setup.bash ```

# Run the calibration

``` roslaunch franka_easy_handeye handeye_calibration.launch ```