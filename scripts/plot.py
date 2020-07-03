#!/usr/bin/python2

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from std_msgs.msgs import *
import tf2_ros
import numpy


class ros_tf_list:
    def __init__(self):
        parent_name = 'world'
        child_name = 'H01/imu_viz_link'
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        x = np.zeros((1,7))
        count = 0
        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform(parent_name, child_name, rospy.Time())
                np.append(x, [[trans.transform.translation.x],[trans.transform.translation.y],[trans.transform.translation.z], [trans.transform.rotation.x], [trans.transform.rotation.y], [trans.transform.rotation.z], [trans.transform.rotation.w]], axis=1)
                count = count + 1
                if count == 20000
                    print "Save to csv"
                    np.savetxt("/home/kyle/catkin_ws/src/cartographer_testing_cave/data/transform.csv", x, delimiter=",")           
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue       


def main():
    rospy.init_node('plot', anonymous=True)
    my_ros = ros_tf_list()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass