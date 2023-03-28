#!/usr/bin/python3
import rospy
from vision_kit.msg import LLA
import random


def heading_updater():
    pub = rospy.Publisher("lla", LLA, queue_size=10)
    rospy.init_node("heading_publisher_mock", anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    latitude = 45
    longitude = 45
    altitude = 0
    while not rospy.is_shutdown():
        latitude = latitude + random.uniform(0, 0.05)
        longitude = longitude + random.uniform(0, 0.05)
        altitude = altitude + random.uniform(-5, 5)
        msg = LLA()
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        heading_updater()
    except rospy.ROSInterruptException:
        pass


