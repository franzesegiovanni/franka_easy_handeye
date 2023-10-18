#%%
#!/usr/bin/env python
import rospy
import numpy as np
import time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from pynput.keyboard import Listener, Key
from panda_ros import Panda
from std_msgs.msg import Empty 
import rospkg

class Teach(Panda):
    def __init__(self):
        rospy.init_node('Calibration', anonymous=True)
        super(Teach, self).__init__()
        self.r=rospy.Rate(10)
        self.recorded_traj = None 
        self.save_cartesian_position=None
        self.end = False
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()
        rospy.sleep(2)
        self.take_sample_pub = rospy.Publisher('/my_eih_calib_eye_on_hand/take_sample', Empty, queue_size=1)
    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == Key.esc:
            self.end = True        
        if key == Key.space:
            self.save_cartesian_position = True    


    def cart_rec_point(self):
        print('Start Recording demonstration')
        print('Press space bar to save the current position')
        print('Press esc to end the demonstration')
        self.set_stiffness(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        self.recorded_traj = self.curr_pos
        self.recorded_ori = self.curr_ori
        self.end=False
        print('Adding cartesian position') 
        print('Recorded points: ', 1)
        while not(self.end):       
            if  self.save_cartesian_position==True: 
                self.recorded_traj = np.c_[self.recorded_traj, self.curr_pos]
                self.recorded_ori = np.c_[self.recorded_ori, self.curr_ori]
                print('Adding cartesian position') 
                print('Recorded points: ', self.recorded_traj.shape[1])
                time.sleep(0.1)
                self.save_cartesian_position=False     


        print('End of the demonstration')  

        dir_path = rospkg.RosPack().get_path('franka_easy_handeye')
        dir_path = dir_path + '/scripts/data/'
        np.savez(dir_path + 'recorded_points.npz', recorded_traj=self.recorded_traj , recorded_ori=self.recorded_ori)

    def load_file(self, file_name='recorded_points.npz'):
        dir_path = rospkg.RosPack().get_path('franka_easy_handeye')
        dir_path = dir_path + '/scripts/data/' + file_name
        goal_points = np.load(dir_path)
        self.recorded_traj = goal_points['recorded_traj']
        self.recorded_ori = goal_points['recorded_ori']

    def execute_cart_points(self):
        self.set_stiffness(600.0, 600.0, 600.0, 30.0, 30.0, 30.0, 0.0)
        for i in range (self.recorded_traj.shape[1]):
            print('Going to point number : ', i+1)
            goal = PoseStamped()

            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = self.recorded_traj[0,i]
            goal.pose.position.y = self.recorded_traj[1,i]
            goal.pose.position.z = self.recorded_traj[2,i]

            goal.pose.orientation.w = self.recorded_ori[0,i]
            goal.pose.orientation.x = self.recorded_ori[1,i]
            goal.pose.orientation.y = self.recorded_ori[2,i]
            goal.pose.orientation.z = self.recorded_ori[3,i]

            self.go_to_pose(goal)

            time.sleep(2)
            self.take_sample_pub.publish(Empty())
            time.sleep(2) 
        
        print('End of recorded calibration points')