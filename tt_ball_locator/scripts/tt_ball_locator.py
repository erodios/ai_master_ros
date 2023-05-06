#!/usr/bin/env python3
import rclpy
import math
# Lifecycle imports
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
# Messages imports
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
# TF imports
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
# Import for two topics in one subscriber
import message_filters
# Computer Vision imports
from cv_bridge import CvBridge
import cv2
import numpy as np


class BallLocatorNode(Node):
    def __init__(self):
        self.bridge = CvBridge()
        self.ball_radius = 0.02 # Tabel-Tenis Ball Radius in Meters
        super().__init__('tt_ball_locator')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating...')
        # One topic subscriber it is not needed but i keep it for reference
        # self.subscription = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.camera_callback, 10)

        # Create two subscriber
        image_sub = message_filters.Subscriber(self, Image, '/head_front_camera/rgb/image_raw')
        point_sub = message_filters.Subscriber(self, PointCloud2, '/head_front_camera/depth_registered/points')

        # Combine the messages from the two subscribers into on message
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, point_sub], 10, 0.1, allow_headerless=True)
        # Call the Image processing function with the previous message as input
        ts.registerCallback(self.camera_callback)
        
        return super().on_activate(state)

    def on_deactivate(self):
        self.get_logger().info('Deactivating...')
        #return super().on_deactivate()

    def on_cleanup(self):
        self.get_logger().info('Cleaning up...')
        #return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self):
        self.get_logger().info('Shutting down...')
        #return TransitionCallbackReturn.SUCCESS
        self.destroy_node()

    def camera_callback(self, msg, points):
        # Create a bridge between the ROS Image msg and OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # Image Pre Processing
        grayImg = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurredImg2 = cv2.GaussianBlur(grayImg, (5, 5), 0)

        # Find the circles in the Processed Image
        circles = cv2.HoughCircles(blurredImg2, cv2.HOUGH_GRADIENT, 1.2, 400,
                                   param1=100,param2=15,minRadius=1,maxRadius=10)
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            chosen = None

            for i in circles[0,:]:
                if chosen is None or i[2] > chosen[2]:
                    chosen = i

            # Calculate the Distance of the ball from the camera
            pixelToMM = 1.5625  # Width of the Table-Tennis table divided by the width of the camera.
            horiResMM = 640 * pixelToMM 
            FOVRad = 0.506145483    # FOV for the half part of the camera.
            FocalLength = (horiResMM/2)/math.tan(FOVRad)    # Distance from the camera to the picture
            diameterMM = (chosen[2]+1)/2 * pixelToMM    
            distanceMM = (FocalLength * horiResMM) / (diameterMM)
            distanceToBall = distanceMM / 100

        # Informations from the Camera Info Topic 
        fx = 522.1910329546544
        fy = 522.1910329546544
        cx = 320    # half of the width
        cy = 240    # half of the height

        # Store the values of position of the center of the ball
        u = chosen[0]
        v = chosen[1]
        z = distanceToBall/1000
        
        # Update the values of X and Y with the camera info
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # Create a Broadcaster and a Transformer to connect the ball with the camera
        broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "head_front_camera_rgb_optical_frame"
        transform.child_frame_id = "tt_ball_center_link"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.x = 1.0
        transform.transform.rotation.y = 1.0
        transform.transform.rotation.z = 1.0
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BallLocatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()