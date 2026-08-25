#!/usr/bin/env python

# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np

class ColourContoursDetector(Node):
    def __init__(self):
        super().__init__('colour_contours_detector')
        self.object_pub = self.create_publisher(PolygonStamped, '/object_polygon', 10)
        self.create_subscription(Image, '/limo_camera/image', self.camera_callback, 10)

        self.countour_color = (255, 255, 0) # cyan
        self.countour_width = 1 # in pixels
        self.br = CvBridge()

    def camera_callback(self, data):
        #self.get_logger().info("camera_callback")

        try:
            bgr_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # using the BGR colour space, create a mask for everything
        # that is in a certain range
        bgr_thresh = cv2.inRange(bgr_image,
                                 np.array((0, 120, 0)),
                                 np.array((140, 255, 140)))

        # It often is better to use another colour space, that is
        # less sensitive to illumination (brightness) changes.
        # The HSV colour space is often a good choice. 
        # So, we first change the colour space here...
        hsv_img = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # ... and now let's create a binary (mask) image, looking for 
        # any hue (range: 0-255), but for something brightly
        # colours (high saturation: > 150)
        # provide the right range values for each BGR channel (set to red bright objects)
        # Create mask for range of colours (HSV low values, HSV high values)
        # Onine colour picker - https://redketchup.io/color-picker
        hsv_thresh = cv2.inRange(hsv_img,
                                 np.array((155, 25, 0)),
                                 np.array((179, 255, 255)))

        # This is how we could find actual contours in
        # the BGR image, but we won't do this now.
        # _, bgr_contours, hierachy = cv2.findContours(
        #     bgr_thresh.copy(),
        #     cv2.RETR_TREE,
        #     cv2.CHAIN_APPROX_SIMPLE)

        # Instead find the contours in the mask generated from the
        # HSV image.
        hsv_contours, hierachy = cv2.findContours(
            hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
        
        # in hsv_contours we now have an array of individual
        # closed contours (basically a polgon around the 
        # blobs in the mask). Let's iterate over all those found 
        # contours.
        detected_objects = []
        for contour in hsv_contours:
            # This allows to compute the area (in pixels) of a contour
            area = cv2.contourArea(contour)
            # and if the area is big enough, we draw the outline
            # of the contour (in blue)
            if area > 100.0:
                cv2.drawContours(bgr_image, contour, -1, (0, 0, 255), 10)
                bbx, bby, bbw, bbh = cv2.boundingRect(contour)
                # append the bounding box of the region into a list
                detected_objects.append(Polygon(points = [Point32(x=float(bbx), y=float(bby)), Point32(x=float(bbw), y=float(bbh))]))
                #if self.visualisation:
                cv2.rectangle(bgr_image, (bbx, bby), (bbx+bbw, bby+bbh), self.countour_color,  self.countour_width)
        # publish individual objects from the list
        # the header information is taken from the Image message
        for polygon in detected_objects:
            self.object_pub.publish(PolygonStamped(polygon=polygon, header=data.header))

        # visualise the image processing results    
        #if self.visualisation:
        cv2.imshow("colour image", bgr_image)
        cv2.imshow("detection mask", hsv_thresh)
        cv2.waitKey(1)
        
        #cv_image_small = cv2.resize(bgr_image, (0,0), fx=0.4, fy=0.4) # reduce image size
        #cv2.imshow("Image window", cv_image_small)
        #cv2.waitKey(1)

def main(args=None):
    print('Starting colour_contours.py.')

    rclpy.init(args=args)

    colour_contours = ColourContoursDetector()

    rclpy.spin(colour_contours)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_contours.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()