#!/usr/bin/python

from sensor_msgs.msg import JointState
import rospy
import numpy as np

PI = np.pi

jointName = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
    "left_joint_1", "left_joint_2", "left_joint_3", "left_joint_4", "left_joint_5", "left_joint_6",
    "right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4", "right_joint_5", "right_joint_6"]

if __name__ == '__main__':
    rospy.init_node('joint_angle_publisher')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=100)
    rate = rospy.Rate(2)
    jointState =JointState()
    jointState.name = jointName

# motion planning
    

    while not rospy.is_shutdown():
        jointState.header.stamp = rospy.Time.now()
        # jointState.position = np.zeros((18,1))
        dualBot = robot1.extend(robot2)
        jointState.position = dualBot.extend(robot3)
        pub.publish(jointState)
        rate.sleep()
