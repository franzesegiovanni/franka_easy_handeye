import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import time
import os
import argparse
import tf2_ros 
import pickle
#!/usr/bin/env python3

class DepthEstimator:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_frames = []
        self.start_time = None
        self.duration = rospy.get_param('~duration', 5)  # Default 5 seconds
        self.is_recording = False
        
        # Initialize the subscriber
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        self.rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.camera_info_callback)
        self.camera_frame = "camera_color_optical_frame"
        self.base_frame = "panda_link0"

        self.tfBuffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.tfBuffer)
        # self._tf_listener = tf2_ros.TransformListener(self.tfBuffer)
    def read_transform(self):
        try:
            # Get the transform between the camera frame and the robot base frame
            transform = self.tfBuffer.lookup_transform(self.base_frame, self.camera_frame, rospy.Time(0), rospy.Duration(2))
            return transform
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            # Create an identity transform if lookup fails
            identity_transform = tf2_ros.TransformStamped()
            identity_transform.header.frame_id = self.base_frame
            identity_transform.child_frame_id = self.camera_frame
            identity_transform.transform.rotation.w = 1.0  # Identity quaternion
            return identity_transform
    def camera_info_callback(self, msg):
        self.cx = msg.K[2]
        self.cy = msg.K[5]
        self.fx = msg.K[0]
        self.fy = msg.K[4]
    def depth_callback(self, msg):
        if not self.is_recording:
            return
            
        # Convert ROS image to OpenCV format
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            # Store the depth image
            self.depth_frames.append(depth_image)
            
            # Check if recording duration is complete
            if time.time() - self.start_time >= self.duration:
                self.is_recording = False
                self.visualize_depth()
        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        except Exception as e:
            rospy.logerr(f"Error processing RGB image: {e}")
    
    def start_recording(self, name='test'):
        self.name = name
        self.depth_frames = []
        self.start_time = time.time()
        self.is_recording = True
        rospy.loginfo(f"Started recording depth data for {self.duration} seconds")

    def normalize_for_display(self,depth_image):
        depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        return depth_image.astype(np.uint8)
    def visualize_depth(self):
        if not self.depth_frames:
            rospy.logwarn("No depth frames were recorded!")
            return
            
        rospy.loginfo(f"Recorded {len(self.depth_frames)} depth frames")
        
        # Apply median filter to reduce noise
        median_depth = np.median(self.depth_frames, axis=0)
        self.avg_depth = np.clip(median_depth, 100, 1000)
        # Normalize for display
        avg_depth_vis = np.copy(self.avg_depth).astype(np.uint8)
        
        avg_depth_vis = self.normalize_for_display(avg_depth_vis)

        # Display the average depth image
        cv2.imshow("Average Depth Image", avg_depth_vis)
        # Apply colormap for better visualization
        cv2.imshow("RGB Image", self.rgb_image)
        print("Dimension of the rgb image")
        print(self.rgb_image.shape)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
        self.save(name=self.name)
    
    def save(self, name='test'):
        parent_dir = os.path.dirname(os.path.abspath(__file__))
        save_dir = os.path.join(parent_dir, 'data')

        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            
        # Save RGB image and depth data separately for backward compatibility
        img_path = os.path.join(save_dir, f"{name}.jpg")
        cv2.imwrite(img_path, self.rgb_image)
        depth_path = os.path.join(save_dir, f"{name}.npy")
        np.save(depth_path, self.avg_depth)
        
        print("Dimension of the depth image is:", self.avg_depth.shape)
        print("Dimension of the rgb image is:", self.rgb_image.shape)
        
        # Get current transform of the camera
        camera_transform = None
        try:
            # Create an empty pose to transform
            camera_transform = self.read_transform()
        except Exception as e:
            rospy.logwarn(f"Failed to get camera transform: {e}. Is the robot running?")
        
        # Create a comprehensive object with all data
        combined_data = {
            'rgb_image': self.rgb_image,
            'depth_image': self.avg_depth,
            'camera_info': {
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy
            },
            'camera_transform': {
                'translation': {
                    'x': camera_transform.transform.translation.x,
                    'y': camera_transform.transform.translation.y,
                    'z': camera_transform.transform.translation.z
                },
                'rotation': {
                    'x': camera_transform.transform.rotation.x,
                    'y': camera_transform.transform.rotation.y,
                    'z': camera_transform.transform.rotation.z,
                    'w': camera_transform.transform.rotation.w
                }
            }
        }
        
        # Save the combined object
        save_path = os.path.join(save_dir, f"{name}.pkl")
        with open(save_path, 'wb') as f:
            pickle.dump(combined_data, f)
        rospy.loginfo(f"Saved combined data to {save_path}")
        
        # # Save camera intrinsics as YAML for backward compatibility
        # camera_path = os.path.join(save_dir, f"camera_info.yaml")
        # with open(camera_path, 'w') as f:
        #     yaml.dump(combined_data['camera_info'], f)
        
        rospy.loginfo(f"Saved data to {save_dir}")

def main(name= 'test'):
    rospy.init_node('depth_estimator')
    de = DepthEstimator()
    
    # Wait a bit to ensure topics are connected
    rospy.sleep(1.0)
    
    # Start recording
    de.start_recording(name=name)
    
    rospy.spin()
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Depth Estimation Node')
    parser.add_argument('--name', type=str, default='test2', help='Name for the recording')
    args = parser.parse_args()
    name = args.name

    # Set the name for the recording
    print("Recording name is set to:", name)
    main(name)