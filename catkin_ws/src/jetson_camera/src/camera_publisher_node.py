#!/usr/bin/env python2

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class CameraPublisherNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera publisher node...")
        # Initialize the ROS node
        rospy.init_node('camera_publisher_node', anonymous=True)

        # Publisher for the image topic
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

        # Create a CvBridge object for converting images
        self.bridge = CvBridge()

        # Camera Dimensions And Properties Parameters
        self.sensor_id = 0
        self.width = 640
        self.height = 480
        self.fps = 30
        self.flip_method = 0
        
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
            return ('nvarguscamerasrc sensor-id=0 ! '
                    'video/x-raw(memory:NVMM), '
                    'width=(int)%d, height=(int)%d, '
                    'format=(string)NV12, framerate=(fraction)%d/1 ! '
                    'nvvidconv flip-method=%d ! '
                    'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
                    'videoconvert ! '
                    'video/x-raw, format=(string)BGR ! appsink' % (self.width, self.height, self.fps, 
                                                                    self.flip_method, self.width, self.height))
        elif self.sensor_id == 1:
            return ('v4l2src device=/dev/video1 ! '
                    'video/x-raw, '
                    'width=(int)%d, height=(int)%d, '
                    'format=(string)YUY2, framerate=(fraction)%d/1 ! '
                    'videoconvert ! '
                    'video/x-raw, format=BGR ! '
                    'appsink' % (self.width, self.height, self.fps))
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
                rospy.loginfo("Camera publisher captured first image from physical device.")
            try:
                # Convert the OpenCV image to a ROS Image message
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")

                # Publish the image
                self.image_pub.publish(img_msg)

            except CvBridgeError as e:
                rospy.logerr("Error converting image: {}".format(e))

            # Sleep to maintain the publishing rate
            rate.sleep()

    def cleanup(self):
        self.cap.release()

if __name__ == "__main__":
    # Initialize the node
    try:
        camera_pub = CameraPublisherNode()
        camera_pub.start_publishing()
    except rospy.ROSInterruptException:
        pass
    finally:
        camera_pub.cleanup()