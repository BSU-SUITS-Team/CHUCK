#!/usr/bin/python3
import os
import requests
import rospy
from std_msgs.msg import String

DEVICE_NAME = os.environ.get("DEVICE_NAME", "Rover")
QUEUE_SIZE = os.environ.get("MESSAGE_QUEUE_SIZE", 10)


class GroundControlCommands:
    def __init__(self):
        self.gc = os.environ.get("GROUND_CONTROL_API", "0.0.0.0")
        self.user_id = os.environ.get("USER_ID", 1)
        self.publisher = None

    def sync(self):
        """Syncs commands from the ground control API to the ROS network"""
        data = requests.get(
            f"http://{self.gc}/devices/has_commands/{DEVICE_NAME}").json()
        while data == True:
            data = requests.get(
                f"http://{self.gc}/devices/get_commands/{DEVICE_NAME}").json()
            rospy.loginfo(rospy.get_caller_id() + " New Command: %s", data)
            self.publisher.publish(data)
            data = requests.get(
                f"http://{self.gc}/devices/has_commands/{DEVICE_NAME}").json()

    def register(self):
        """Registers the device with the ground control API"""
        rospy.init_node("ground_control_sync", anonymous=True)
        self.publisher = rospy.Publisher(
            "ground_control_commands", String, queue_size=10)
        
        # get the ip address of the device. TODO: This WILL NOT WORK on our local-only network. This is for testing only.
        my_ip = requests.get("https://api.ipify.org").text

        requests.post(
            f"http://{self.gc}/devices/configure_device/{DEVICE_NAME}?ip_address={my_ip}")


def main():
    gc = GroundControlCommands()
    gc.register()

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        gc.sync()
        rate.sleep()


if __name__ == "__main__":
    main()
