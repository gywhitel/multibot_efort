#!/usr/bin/python
import rospy
import tf2_ros
import tf2_msgs.msg
import math

left = [0,0,0]
right = [0,0,0]

def callback(tfData:tf2_msgs.msg.TFMessage):
    # print(tfData)
    ER10_end_effector = tfData.transforms[5]
    left_ER20_end_effector = tfData.transforms[10]
    right_ER20_end_effector = tfData.transforms[17]
    left[0] = left_ER20_end_effector.transform.translation.x
    left[1] = left_ER20_end_effector.transform.translation.y
    left[2] = left_ER20_end_effector.transform.translation.z
    right[0] = right_ER20_end_effector.transform.translation.x
    right[1] = right_ER20_end_effector.transform.translation.y
    right[2] = right_ER20_end_effector.transform.translation.z
    print(left_ER20_end_effector.header.frame_id, left, right_ER20_end_effector.header.frame_id, right)
    print(f"distance: {distance}")

if __name__ == '__main__':
    rospy.init_node('distance_display')
    sub = rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, callback=callback, queue_size=1)
    distance = math.sqrt((left[0]-right[0])**2 + (left[1]-right[1])**2 + (left[2]-right[2])**2 )

    while not rospy.is_shutdown():
        rospy.spin()
        