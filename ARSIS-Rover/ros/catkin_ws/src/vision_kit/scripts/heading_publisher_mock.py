#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32


def heading_updater():
    pub = rospy.Publisher("heading", Float32, queue_size=10)
    rospy.init_node("heading_publisher_mock", anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    i = 0
    while not rospy.is_shutdown():
        i += 1
        i %= 360
        rospy.loginfo(i)
        pub.publish(i)
        rate.sleep()


if __name__ == "__main__":
    try:
        heading_updater()
    except rospy.ROSInterruptException:
        pass
