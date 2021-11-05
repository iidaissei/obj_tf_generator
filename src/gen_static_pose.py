#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import tf
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import String, Float64
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class GenHumanFrame():
    def __init__(self):
        self.br = tf2_ros.StaticTransformBroadcaster()
        self.static_stamp = geometry_msgs.msg.TransformStamped()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def execute(self):
        # self.static_stamp.header.stamp = rospy.Time.now()
        # self.static_stamp.header.frame_id = "camera_depth_frame"
        # self.static_stamp.child_frame_id = "static_human_frame"

        # self.static_stamp.transform.translation.x = float(1.0)
        # self.static_stamp.transform.translation.y = float(0.5)
        # self.static_stamp.transform.translation.z = float(0.0)

        # quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        # self.static_stamp.transform.rotation.x = quat[0]
        # self.static_stamp.transform.rotation.y = quat[1]
        # self.static_stamp.transform.rotation.z = quat[2]
        # self.static_stamp.transform.rotation.w = quat[3]

        # self.br.sendTransform(self.static_stamp)
        print "b"
        # rate = rospy.Rate(10.0)
        # while not rospy.is_shutdown():
            # print "c"
            # try:
                # trans = self.tfBuffer.lookup_transform('map','static_human_frame', rospy.Time(0))
        trans = self.tfBuffer.lookup_transform('map','static_human_frame', rospy.Time(0))
        quat_2 = tf.transformations.quaternion_from_euler(trans.transform.translation.x,
                # quat_2 = tf.transformations.quaternion_from_euler(trans.transform.translation.x,
                                                                  trans.transform.translation.y,
                                                                  0.0, )
                # print quat_2
                # return quat_2
            # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # rate.sleep()
                # return False
            # rate.sleep()
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform('map','static_human_frame', rospy.Time())
                quat_2 = tf.transformations.quaternion_from_euler(trans.transform.translation.x,
                                                                  trans.transform.translation.y, 0.0)
                print quat_2
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
            rate.sleep()

class NaviLocationServer():
    def __init__(self):
        self.ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # Publisher
        self.crloc_pub = rospy.Publisher('/current_location', String, queue_size = 1)
        # Service
        self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        # Value
        self.location_name = "null"

    def sendGoal(self, location_list):
        # set goal_pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = location_list[0]
        goal.target_pose.pose.position.y = location_list[1]
        goal.target_pose.pose.orientation.z = location_list[2]
        goal.target_pose.pose.orientation.w = location_list[3]
        # clearing costmap
        rospy.loginfo("Clearing costmap...")
        rospy.wait_for_service('move_base/clear_costmaps')
        self.clear_costmap()
        rospy.sleep(0.5)
        # start navigation
        # self.head_pub.publish(0)
        self.ac.wait_for_server()
        self.ac.send_goal(goal)
        # self.ac.wait_for_result()
        navi_state = self.ac.get_state()
        while not rospy.is_shutdown():
            navi_state = self.ac.get_state()
            if navi_state == 3:
                rospy.loginfo('Navigation success!!')
                self.crloc_pub.publish(self.location_name)
                return NaviLocationResponse(result = True)
            elif navi_state == 4:
                rospy.loginfo('Navigation Failed')
                return NaviLocationResponse(result = False)
            else:
                pass


if __name__ == '__main__':
    rospy.init_node('gen_static_pose')
    ghf = GenHumanFrame()
    sls = NaviLocationServer()
    result  = ghf.execute()
    if result:
        aaa = sls.sendGoal(result)

    else:
        rospy.logerr("Result false")
        pass
