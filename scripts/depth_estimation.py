import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import time
import os
import argparse
import yaml
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
        # Save the depth frames
        #Save self.avg_depth_vis as jpg
        img_path = os.path.join(save_dir, f"{name}.jpg")
        cv2.imwrite(img_path, self.rgb_image)
        #Save the depth value as numpy 
        depth_path = os.path.join(save_dir, f"{name}.npy")
        np.save(depth_path, self.avg_depth)
        print("Dimension of the depth image is:", self.avg_depth.shape)
        # Save the RGB image
        print("Dimension of the rgb image is:", self.rgb_image.shape)
        rospy.loginfo(f"Saved depth data to {depth_path}")
        # Save the camera intrinsic parameters as YAML file
        camera_info = {
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy
        }
        camera_path = os.path.join(save_dir, f"camera_info.yaml")
        with open(camera_path, 'w') as f:
            yaml.dump(camera_info, f)
        rospy.loginfo(f"Saved camera intrinsics to {camera_path}")

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
    parser.add_argument('--name', type=str, default='test', help='Name for the recording')
    args = parser.parse_args()
    name = args.name

    # Set the name for the recording

    main(name)