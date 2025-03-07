#!/usr/bin/env python2
import rospy
from std_msgs.msg import UInt8

MAX_VAL = 256


class Node:
    def __init__(self, node_name):
        self.initialized = False
        rospy.loginfo("Initializing subscriber node...")
        # Initialize the ROS node
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)

        # Subscriber for the data topic
        self.data_sub = rospy.Subscriber(
            "/data_raw", UInt8, self.data_cb, queue_size=10
        )

        # Subscriber rate
        self.rate = 1
        # Variable to hold the last recorded value
        self.value = -1
        # Range of possible values
        self.range = MAX_VAL
        self.initialized = True
        rospy.loginfo("Subscriber node initialized!")

    def data_cb(self, data_msg):
        rate = rospy.Rate(self.rate)
        curr_data = data_msg.data
        rospy.loginfo("Subscriber Value: " + str(curr_data))

        if curr_data != (self.value + 1) % self.range:
            rospy.logwarn(
                "Subscriber received value other than expected. Expected: "
                + str((self.value + 1) % self.range)
                + ", Received: "
                + str(curr_data)
            )
        self.value = curr_data

        # Sleep to maintain the subscribing rate
        rate.sleep()


if __name__ == "__main__":
    try:
        camera_node = Node(node_name="nodeA")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
