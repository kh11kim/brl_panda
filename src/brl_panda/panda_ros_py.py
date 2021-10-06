#!/usr/bin/env python3
import rospy
import actionlib
import numpy as np
from franka_gripper.msg import GraspAction, GraspGoal, HomingAction, HomingGoal, \
                               MoveAction, MoveGoal, StopAction, StopGoal, GraspEpsilon
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped
import dynamic_reconfigure.client

def check(test, timeout, rate=100):
    rate = rospy.Rate(rate)
    end_time = rospy.get_time() + timeout

    while not test():
        if rospy.is_shutdown():
            print("roscore is not running")
            return False
        elif rospy.get_time() >= end_time:
            rospy.loginfo("timeout")
            return False
        rate.sleep()

    return True



class Gripper:
    def __init__(self):
        ns = "/panda/franka_gripper/"
        action_dict = {"homing":HomingAction, 
                       "grasp":GraspAction,
                       "move":MoveAction,
                       "stop":StopAction}
        self._gripper_speed = 0.05
        self.MAX_WIDTH = 0.2

        # Check if gripper is loaded
        try:
            rospy.get_param(ns+"robot_ip")
        except KeyError:
            rospy.loginfo("Gripper is not loaded")
            return

        # Check if there are action servers for gripper
        rospy.loginfo("Waiting for gripper action servers... ")
        self._action_clients = {}
        for action_name, action in action_dict.items():
            self._action_clients[action_name] = actionlib.SimpleActionClient(ns+action_name, action)   
            self._action_clients[action_name].wait_for_server()
        rospy.loginfo("Gripper action servers found!")
    
    def _done_cb(self, status, result):
        rospy.logdebug("Gripper: '{}' complete. Result: \n\t{}".format(self._caller, result))
    
    def homing(self):
        self._caller = "homing"
        self._action_clients["homing"].send_goal(HomingGoal(), done_cb=self._done_cb)

        return True

    def grasp(self, width, force, speed=None, epsilon_inner=0.005, epsilon_outer=0.005, cb=None):
        if not speed:
            speed = self._gripper_speed
        if not cb:
            cb = self._done_cb

        # Make Grasp Goal
        goal = GraspGoal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon = GraspEpsilon(inner=epsilon_inner, outer=epsilon_outer)
        
        self._caller = "grasp"
        self._action_clients["grasp"].send_goal(goal, done_cb=cb)

        return True
    
    def move(self, width, speed=None):
        if not speed:
            speed = self._gripper_speed
        
        # Make Move Goal
        goal = MoveGoal()
        goal.width = width
        goal.speed = speed
        
        self._caller = "move"
        self._action_clients["move"].send_goal(goal, done_cb=self._done_cb)

        return True
        
    def stop(self):
        self._caller = "stop"
        self._action_clients["stop"].send_goal(StopGoal(), done_cb =self._done_cb)
        result = self._action_clients["stop"].wait_for_result(rospy.Duration(10))
        return result

    def open(self):
        self._caller = "open"
        return self.move(self.MAX_WIDTH)

    def close(self):
        self._caller = "close"
        def cb(_, result):
            if not result.success:
                self.stop()
        return self.grasp(0, 0.1, cb=cb)

class Arm:
    def __init__(self):
        self.ns = "/panda/"
        self._franka_state_sub = rospy.Subscriber(
            self.ns+"franka_state_controller/franka_states",
            FrankaState,
            self._franka_state_callback,
            queue_size=1,
            tcp_nodelay=True)

        self._eq_pose_pub = rospy.Publisher(
            self.ns+"cartesian_impedance_example_controller/equilibrium_pose",
            PoseStamped, queue_size=1)
        
        self._reconfig_client = dynamic_reconfigure.client.Client("/panda/cartesian_impedance_example_controllerdynamic_reconfigure_compliance_param_node")

        # Check if franka_state_controller is loaded
        try:
            rospy.get_param(self.ns+"franka_state_controller/"+"arm_id")
            rospy.loginfo("Arm is connected")
        except KeyError:
            rospy.loginfo("Arm is not connected.")
            return

        

    def _franka_state_callback(self, msg):
        self._O_T_EE = np.asarray(msg.O_T_EE).reshape(4,4, order="F")
        self._O_F_ext_hat_K = np.asarray(msg.O_F_ext_hat_K)
        self._q = np.asarray(msg.q)
        self._dq = np.asarray(msg.dq)
        self._tau = np.asarray(msg.tau_ext_hat_filtered)

    def get_EE_pose(self):
        return self._O_T_EE

    def set_eq_pose(self, pos, ori):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        msg.pose.orientation.w = ori[0]
        msg.pose.orientation.x = ori[1]
        msg.pose.orientation.y = ori[2]
        msg.pose.orientation.z = ori[3]
        self._eq_pose_pub.publish(msg)

    def set_rotational_stiffness(self, k):
        if 0 <= k <= 30:
            self._reconfig_client.update_configuration(
                                {"rotational_stiffness":k})
        else:
            rospy.loginfo("k is not in range")

    def set_translational_stiffness(self, k):
        if 0 <= k <= 400:
            self._reconfig_client.update_configuration(
                                {"translational_stiffness":k})
        else:
            rospy.loginfo("k is not in range")

    def get_current_stiffness(self):
        self._reconfig_client.update_configuration({})


    
    