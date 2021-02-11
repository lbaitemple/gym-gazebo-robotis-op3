import gym
import rospy
import roslaunch
import time
import numpy as np

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan

from gym.utils import seeding

op3_command_topics = [
    "/robotis_op3/head_pan_position/command",
    "/robotis_op3/head_tilt_position/command",
    "/robotis_op3/l_ank_pitch_position/command",
    "/robotis_op3/l_ank_roll_position/command",
    "/robotis_op3/l_el_position/command",
    "/robotis_op3/l_hip_pitch_position/command",
    "/robotis_op3/l_hip_roll_position/command",
    "/robotis_op3/l_hip_yaw_position/command",
    "/robotis_op3/l_knee_position/command",
    "/robotis_op3/l_sho_pitch_position/command",
    "/robotis_op3/l_sho_roll_position/command",
    "/robotis_op3/r_ank_pitch_position/command",
    "/robotis_op3/r_ank_roll_position/command",
    "/robotis_op3/r_el_position/command",
    "/robotis_op3/r_hip_pitch_position/command",
    "/robotis_op3/r_hip_roll_position/command",
    "/robotis_op3/r_hip_yaw_position/command",
    "/robotis_op3/r_knee_position/command",
    "/robotis_op3/r_sho_pitch_position/command",
    "/robotis_op3/r_sho_roll_position/command"
]

op3_state_topics = [
    "/robotis_op3/head_pan_position/state",
    "/robotis_op3/head_tilt_position/state",
    "/robotis_op3/l_ank_pitch_position/state",
    "/robotis_op3/l_ank_roll_position/state",
    "/robotis_op3/l_el_position/state",
    "/robotis_op3/l_hip_pitch_position/state",
    "/robotis_op3/l_hip_roll_position/state",
    "/robotis_op3/l_hip_yaw_position/state",
    "/robotis_op3/l_knee_position/state",
    "/robotis_op3/l_sho_pitch_position/state",
    "/robotis_op3/l_sho_roll_position/state",
    "/robotis_op3/r_ank_pitch_position/state",
    "/robotis_op3/r_ank_roll_position/state",
    "/robotis_op3/r_el_position/state",
    "/robotis_op3/r_hip_pitch_position/state",
    "/robotis_op3/r_hip_roll_position/state",
    "/robotis_op3/r_hip_yaw_position/state",
    "/robotis_op3/r_knee_position/state",
    "/robotis_op3/r_sho_pitch_position/state",
    "/robotis_op3/r_sho_roll_position/state"
]

class GazeboRobotisOp3Env(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "GazeboRobotisOp3.launch")

        self.publishers = [rospy.Publisher(topic, Float64, queue_size=5) for topic in op3_command_topics]

        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def calculate_observation(self, data):
        return data, False

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def scan_data(self):
        data = [None for _ in range(len(op3_state_topics))]
        return data
        
        data_fill = 0
        while data_fill < len(op3_state_topics):
            try:
                for index, topic in enumerate(op3_state_topics):
                    if data[index] is not None: continue
                    msg = rospy.wait_for_message(topic, JointControllerState, timeout=1)
                    if msg is not None:
                        data[index] = msg.process_value
                        data_fill += 1
            except:
                pass
        return data

    def step(self, action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        for index, pub in enumerate(self.publishers):
            value = Float64()
            value.data = action[index]
            pub.publish(value)

        data = self.scan_data()

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state, done = self.calculate_observation(data)

        if not done:
            # reward = round(15*(max_ang_speed - abs(ang_vel) +0.0335), 2)
            reward = 0
        else:
            reward = 0

        return np.asarray(state), reward, done, {}

    def reset(self):
        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        #read laser data
        data = self.scan_data()

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state,done = self.calculate_observation(data)

        return np.asarray(state)
