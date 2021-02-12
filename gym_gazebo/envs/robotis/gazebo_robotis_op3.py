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
from gazebo_msgs.msg import LinkStates
from controller_manager_msgs.srv import ListControllers
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

op3_controller_names = [
    "head_pan_position",
    "head_tilt_position",
    "l_ank_pitch_position",
    "l_ank_roll_position",
    "l_el_position",
    "l_hip_pitch_position",
    "l_hip_roll_position",
    "l_hip_yaw_position",
    "l_knee_position",
    "l_sho_pitch_position",
    "l_sho_roll_position",
    "r_ank_pitch_position",
    "r_ank_roll_position",
    "r_el_position",
    "r_hip_pitch_position",
    "r_hip_roll_position",
    "r_hip_yaw_position",
    "r_knee_position",
    "r_sho_pitch_position",
    "r_sho_roll_position"
]

class GazeboRobotisOp3Env(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "GazeboRobotisOp3.launch")

        self.publishers = [rospy.Publisher(topic, Float64, queue_size=5) for topic in op3_command_topics]

        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.listControllers = rospy.ServiceProxy('/robotis_op3/controller_manager/list_controllers', ListControllers)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.prev_action = np.zeros(20)

        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def calculate_observation(self, data):
        names = data.name
# - robotis_op3::body_link
# - robotis_op3::head_pan_link
# - robotis_op3::head_tilt_link
        for index, name in enumerate(data.name):
            if name != "robotis_op3::body_link": continue
            p = data.pose[index].position
            return (p.x, p.y, p.z), False
        return None, False

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def scan_data(self):
        while True:
            try:
                data = rospy.wait_for_message("/gazebo/link_states", LinkStates, timeout=5)
                if data is not None:
                    return data
            except:
                pass
    
    def supress_action(self, action):
        if self.prev_action is None:
            return action
        prev = np.array(self.prev_action)
        curr = np.array(action)
        delta = curr - prev

        max_speed = 0.05
        delta = np.minimum(delta, max_speed)
        delta = np.maximum(delta, -max_speed)
        supressed_action = prev + delta

        self.prev_action = supressed_action

        return supressed_action

    def step(self, action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        action = self.supress_action(action)
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

    def wait_for_controllers(self):
        rospy.wait_for_service('/robotis_op3/controller_manager/list_controllers')
        ctrls = None
        while True:
            ctrls = self.listControllers().controller
            ctrl_names = [x.name for x in ctrls]
            flag = True
            for name in op3_controller_names:
                if name not in ctrl_names:
                    flag = False
                    break
            if flag: break
        print 'Checked all controllers available'
        for ctrl in ctrls:
            print ctrl.name, ctrl.state

    def reset(self):
        self.wait_for_controllers()

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
