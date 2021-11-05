#!/usr/bin/env python
import rospy

import actionlib
import math
import tf
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import String, Float64
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


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
                # return NaviLocationResponse(result = True)
                return True
            elif navi_state == 4:
                rospy.loginfo('Navigation Failed')
                # return NaviLocationResponse(result = False)
                return False
            else:
                pass

if __name__ == '__main__':
    rospy.init_node('obj_tf_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('map','static_human_frame', rospy.Time())
            # quat_2 = tf.transformations.quaternion_from_euler(trans.transform.translation.x,
                                                              # trans.transform.translation.y,
                                                              # trans.transform.translation.z)
            quat_2 = [0.0, 0.0, 0.0, 0.0]
            quat_2[0] = trans.transform.translation.x
            quat_2[1] = trans.transform.translation.y
            quat_2[2] = trans.transform.rotation.z
            quat_2[3] = trans.transform.rotation.w
            # print trans.transform.translation.y
            # print trans.transform.translation.z
            print quat_2
            # print trans
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        rate.sleep()
    sls = NaviLocationServer()
    result = sls.sendGoal(quat_2)
