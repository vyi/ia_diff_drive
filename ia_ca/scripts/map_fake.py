#!/usr/bin/env python
import rospy

import tf_conversions
import tf2_ros

#import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped


def process():
    rospy.init_node('fake_map', anonymous=True)
    #pub = rospy.Publisher('/map', TransformStamped, queue_size=10)

    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()


    RATE = 10
    rate = rospy.Rate(RATE) # 10hz
    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "map"
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0


        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
        


if __name__=='__main__':
    try:
        process()
    except rospy.ROSInterruptException:
        pass