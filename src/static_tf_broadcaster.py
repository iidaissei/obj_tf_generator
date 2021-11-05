#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':

    rospy.init_node('static_tf_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "camera_depth_frame"
    static_transformStamped.child_frame_id = "fixed_human"

    # static_transformStamped.transform.translation.x = float(sys.argv[2])
    # static_transformStamped.transform.translation.y = float(sys.argv[3])
    # static_transformStamped.transform.translation.z = float(sys.argv[4])
    static_transformStamped.transform.translation.x = 1.0
    static_transformStamped.transform.translation.y = 0.5
    static_transformStamped.transform.translation.z = 0.0

    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()
