#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage


image_height = 480
image_width = 640

K = np.array(
    [
        [336.5568779901599, 0, 329.48689927534764],
        [0, 336.83379682532745, 247.82748160875434],
        [0, 0, 1],
    ]
)

D = np.array(
    [
        -0.276707022018512,
        0.04795023713853468,
        -0.0024864989237580143,
        -0.00033282476417615016,
        0,
    ]
)


class CameraProcessingNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera processing node...")
        self.bridge = CvBridge()

        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_raw",
            CompressedImage,
            self.image_cb,
            buff_size=10000000,
            queue_size=1,
        )

        # Construct Publisher
        self.image_pub = rospy.Publisher(
            "/camera/image_processed", CompressedImage, queue_size=1
        )

        self.first_image_received = False
        self.initialized = True

        self.newK, self.regionOfInterest = cv2.getOptimalNewCameraMatrix(
            K, D, (image_width, image_height), 1, (image_width, image_height)
        )

        rospy.loginfo("Camera processing node initialized!")

    def image_cb(self, data):
        if not self.initialized:
            return

        if self.first_image_received == False:
            self.first_image_received = True
            rospy.loginfo("Camera processing captured first image from publisher.")
        try:
            # Convert the ROS Image message to an OpenCV image
            np_arr = np.fromstring(data.data, np.uint8)  # Convert to a NumPy array
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode the image
            # Display the image
            # cv2.imshow("Camera View", cv_image)
            # cv2.waitKey(1)

            # Undistort the image
            dst = cv2.undistort(cv_image, K, D, None, self.newK)

            # Crop the image (if necessary)
            # x, y, w, h = self.regionOfInterest
            # dst = dst[y : y + h, x : x + w]

            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode(".jpg", dst)[1]).tostring()

            self.image_pub.publish(msg)

        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))

    def cleanup(self):
        cv2.destroyAllWindows()


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("camera_processing_node", anonymous=True)
    camera_node = CameraProcessingNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")
    finally:
        camera_node.cleanup()
