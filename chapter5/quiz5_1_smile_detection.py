# Quiz 5.1 Smile Detection
# 
# Copyright (c) 2022-2025, TAN Jeffrey Too Chuan (ai-robot-book@googlegroups.com)
# All rights reserved.
# This source code is licensed under the Apache License 2.0 found in the LICENSE file in the root directory of this project.

# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

# Load the cascade
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
smile_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_smile.xml')

from rclpy.qos import qos_profile_sensor_data
 
class SmileDetection(Node):
  """
  Create a SmileDetection class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('quiz5_1_smile_detection')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/image_raw',
      self.listener_callback, 
      qos_profile_sensor_data)

    self.publisher = self.create_publisher(Image, 'smile_detection_result', 10)

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving image')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
    
    # Convert to grayscale
    gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    
    # Detect the faces
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    
    # Draw the rectangle around each face
    for (x,y,w,h) in faces:
        cv2.rectangle(current_frame,(x,y),(x+w,y+h),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = current_frame[y:y+h, x:x+w]
        smiles = smile_cascade.detectMultiScale(roi_gray, 1.8, 20)
        for (sx,sy,sw,sh) in smiles:
            cv2.rectangle(roi_color,(sx,sy),(sx+sw,sy+sh),(0,255,0),2)
    
    # Publish the result in ROS
    smile_detection_result = self.br.cv2_to_imgmsg(current_frame, "bgr8")
    self.publisher.publish(smile_detection_result)

    # Display image
    cv2.imshow("Camera", current_frame)
    
    cv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  quiz5_1_smile_detection = SmileDetection()
  
  # Spin the node so the callback function is called.
  rclpy.spin(quiz5_1_smile_detection)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  quiz5_1_smile_detection.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
