'''
    CameraNode.py

    written by Quan Zhou on 3/20

    Manages the Camera Node and captures images
'''
import rospy
from sensor_msgs import msg

class camera_node:
    def __init__(self):
        self.sub = rospy.Subscriber('/camera/raw_image/', msg.Image, self.handle_image)
        self.current_image = msg.Image()

    def handle_image

