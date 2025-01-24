#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage


class CameraSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera subscriber node...")
        self.bridge = CvBridge()

        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_processed",
            CompressedImage,
            self.image_cb,
            buff_size=10000000,
            queue_size=1,
        )

        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("Camera subscriber node initialized!")

    def image_cb(self, data):
        if not self.initialized:
            return

        if self.first_image_received == False:
            self.first_image_received = True
            rospy.loginfo(
                "Camera subscriber captured first image from processing node."
            )
        try:
            dataArray = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(dataArray, cv2.IMREAD_COLOR)

            # Display the image
            cv2.imshow("Camera View", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))

    def cleanup(self):
        cv2.destroyAllWindows()


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("camera_subscriber_node", anonymous=True)
    camera_node = CameraSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")
    finally:
        camera_node.cleanup()
