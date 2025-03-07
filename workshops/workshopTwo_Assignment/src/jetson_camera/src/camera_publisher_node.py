#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage


class CameraPublisherNode:
    def __init__(self, node_name):
        self.initialized = False
        rospy.loginfo("Initializing camera publisher node...")
        # Initialize the ROS node
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)

        # Publisher for the image topic
        self.image_pub = rospy.Publisher(
            "/camera/image_raw", CompressedImage, queue_size=1
        )

        # Create a CvBridge object for converting images
        self.bridge = CvBridge()

        # Camera Dimensions And Properties Parameters
        config = self.load_yaml()

        # For frame capturing (calibration scripts)
        self.seqNo = 0
        self.saveImages = True

        self.sensor_id = config["sensor_id"]  # 0: CSI, 1: USB
        self.width = config["width"]
        self.height = config["height"]
        self.fps = config["fps"]
        self.flip_method = config["flip_method"]

        # GStreamer pipeline for the Jetson CSI camera
        self.pipeline = self.gstreamer_pipeline()
        if self.pipeline is None:
            rospy.logerr("Pipeline could not be initialized!")

        # OpenCV video capture with the GStreamer pipeline
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            rospy.logerr("Unable to open camera")

        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("Camera publisher node initialized!")

    def gstreamer_pipeline(self):
        if self.sensor_id == 0:
            # Define the GStreamer pipeline for the Jetson camera
            return (
                "nvarguscamerasrc sensor-id=0 ! "
                "video/x-raw(memory:NVMM), "
                "width=(int)%d, height=(int)%d, "
                "format=(string)NV12, framerate=(fraction)%d/1 ! "
                "nvvidconv flip-method=%d ! "
                "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! appsink"
                % (
                    self.width,
                    self.height,
                    self.fps,
                    self.flip_method,
                    self.width,
                    self.height,
                )
            )
        elif self.sensor_id == 1:
            return (
                "v4l2src device=/dev/video1 ! "
                "video/x-raw, "
                "width=(int)%d, height=(int)%d, "
                "format=(string)YUY2, framerate=(fraction)%d/1 ! "
                "videoconvert ! "
                "video/x-raw, format=BGR ! "
                "appsink" % (self.width, self.height, self.fps)
            )
        else:
            return None

    def start_publishing(self):
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Failed to capture image")
                continue
            if self.first_image_received == False:
                self.first_image_received = True
                rospy.loginfo(
                    "Camera publisher captured first image from physical device."
                )
            try:
                # Convert the OpenCV image to a ROS CompressedImage message
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.format = "jpeg"
                msg.data = np.array(cv2.imencode(".jpg", frame)[1]).tostring()

                # Publish the image
                self.image_pub.publish(msg)

                if self.saveImages:
                    # rospy.loginfo("Printing frame to {}".format(os.getcwd()))
                    cv2.imwrite(
                        "/home/jetbot/workshops/workshopTwo_Assignment/calibration/frames/frame_{seqNo}.jpg".format(
                            seqNo=self.seqNo
                        ),
                        frame,
                    )
                    self.seqNo += 1

            except CvBridgeError as e:
                rospy.logerr("Error converting image: {}".format(e))

            # Sleep to maintain the publishing rate
            rate.sleep()

    def cleanup(self):
        self.cap.release()

    def load_yaml(self):
        config = {}
        config["sensor_id"] = rospy.get_param("~sensor_id")
        config["width"] = rospy.get_param("~width")
        config["height"] = rospy.get_param("~height")
        config["fps"] = rospy.get_param("~fps")
        config["flip_method"] = rospy.get_param("~flip_method")

        # Log the loaded configuration
        rospy.loginfo("Loaded config: %s", config)

        return config


if __name__ == "__main__":
    node_name = "camera_publisher_node"
    camera_pub = None
    try:
        camera_pub = CameraPublisherNode(node_name)
        camera_pub.start_publishing()
    except rospy.ROSInterruptException:
        pass
    finally:
        if camera_pub is not None:
            camera_pub.cleanup()
