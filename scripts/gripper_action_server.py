#! /usr/bin/env python3

import rospy
import actionlib
import control_msgs.msg
import std_msgs.msg

from sensor_msgs.msg import JointState


class GripperAction(object):

    def __init__(self, name):
        self._action_name = name
        self._sub = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback, queue_size=1)
        self._pub = rospy.Publisher("/joint_command", JointState, queue_size=1)
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.GripperCommandAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._joint_state = None
        self._gripper_joints = ['l_gripper_finger_joint', 'r_gripper_finger_joint']

    def joint_states_callback(self, message):
        self._joint_state = message
      
    def execute_cb(self, goal):
        if self._joint_state is None:
            print('no joint state recieved in gripper action server')
            return
        
        gripper_position = goal.command.position
        joint_commands = JointState()

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        joint_commands.header = header

        joints_dict = {}
        for i, name in enumerate(self._joint_state.name):
            # Storing arm joint names and positions
            joints_dict[name] = self._joint_state.position[i]

        # set finger positions
        joints_dict[self._gripper_joints[0]] = gripper_position * 0.5
        joints_dict[self._gripper_joints[1]] = gripper_position * 0.5

        joint_commands.name = joints_dict.keys()
        joint_commands.position = joints_dict.values()

        # Publishing combined message containing all arm and finger joints
        self._pub.publish(joint_commands)
        print(joint_commands)
        self._as.set_succeeded()
        
if __name__ == '__main__':
    rospy.init_node('gripper_action_server')
    name = 'gripper_controller/gripper_action'
    server = GripperAction(name)
    print('start gripper action server:', name)
    rospy.spin()