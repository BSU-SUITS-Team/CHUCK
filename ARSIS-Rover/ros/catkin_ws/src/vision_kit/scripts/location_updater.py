#!/usr/bin/python3
import os

import requests
import rospy
from std_msgs.msg import Float32
from vision_kit.msg import LLA


class LocationUpdater:
    def __init__(self):
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0
        self.heading = 0
        self.telemetry_server = os.environ.get("TELEMETRY_API", "0.0.0.0")
        self.user_id = os.environ.get("USER_ID", 1)

    def heading_callback(self, data):
        self.heading = data.data
        rospy.loginfo(rospy.get_caller_id() + " New Heading: %s", data.data)

    def lla_callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.altitude = data.altitude

    def publish(self):
        data = {
            "latitude": self.latitude,
            "longitude": self.longitude,
            "altitude": self.altitude,
            "heading": self.heading,
        }
        headers = {"Content-Type": "application/json", "accept": "application/json"}

        r = requests.post(
            f"http://{self.telemetry_server}/location/{self.user_id}/update_location",
            json=data,
            headers=headers,
        )


def main():
    location_updater = LocationUpdater()

    rospy.init_node("location_updater", anonymous=True)

    rospy.Subscriber("heading", Float32, location_updater.heading_callback)
    rospy.Subscriber("lla", LLA, location_updater.lla_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        location_updater.publish()
        rate.sleep()


if __name__ == "__main__":
    main()
