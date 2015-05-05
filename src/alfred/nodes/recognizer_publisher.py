#!/usr/bin/env python
'''
    recognizer_publisher.py

    Takes inputs from sys.stdin and publishes them
'''
import rospy
import sys
from  std_msgs.msg import String
from termcolor import colored

class recognizer_publisher():
    def __init__(self):
        rospy.init_node('recognizer_publisher')
        self.pub = rospy.Publisher('/recognizer/output', String, queue_size=1)

        rospy.loginfo('Type in commands to echo to /recognizer/output. Send in blank lines to exit')
        self.publish_lines()
    
    def publish_lines(self):
        string = sys.stdin.readline().strip()

        while string != "":
            print colored('sent: "%s"' % string, 'white')
            self.pub.publish(string)
            string = sys.stdin.readline().strip()

if __name__ == '__main__':
    rp = recognizer_publisher()




        
