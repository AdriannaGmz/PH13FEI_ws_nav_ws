#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/img_out",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camThermal/image_raw_16",Image,self.callback)

  def callback(self,data):
    try:
      # bgr8: CV_8UC3, color image with blue-green-red color order 
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      # cv_image = self.bridge.imgmsg_to_cv2(data, "bgra8")

      # DO NOT USE mono8 or Mono16
      # cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)