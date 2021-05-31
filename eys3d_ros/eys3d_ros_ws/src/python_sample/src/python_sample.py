#! /usr/bin/python
import rospy 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge = CvBridge()

def image_callback(color_msg):
    print("Received an image!")
    try:
        # Convert ROS Image message to OpenCV2
        color_img = bridge.imgmsg_to_cv2(color_msg, "bgr8")

    except CvBridgeError, e:
        print(e)
    else:
        # Save OpenCV2 image as a jpeg 
        # cv2.imwrite('camera_image.jpeg', color_img)

        # show image from topic
        cv2.imshow('color', color_img)
        cv2.waitKey(1)
        rospy.sleep(0.01)

def convert_depth_image(depth_msg):
    print("Received an depth image!")
    try:
        depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

    except CvBridgeError, e:
        print (e)

    else:
        depth_array = np.array(depth_img, dtype=np.float32)
        np.save("depth_img.npy", depth_array)
        rospy.loginfo(depth_array)

        cv2.imshow('depth', depth_img)
        cv2.waitKey(1)
        rospy.sleep(0.1)

def main():
    rospy.init_node('python_sample')
    image_topic = "/dm_preview/left/image_color"
    rospy.Subscriber(image_topic, Image, image_callback)
    # depth_image_topic = "/dm_preview/depth/image_raw"
    # rospy.Subscriber(depth_image_topic, Image, convert_depth_image)
    rospy.spin()

if __name__ == '__main__':
     main()
