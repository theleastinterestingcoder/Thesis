'''
    mission_thread.py

    written by Quan Zhou on 3/28

    A thread for managing a mission
'''
import rospy
from std_msgs.msg import String
import pdb

import threading


class mission_thread(threading.Thread):
    def __init__(self, name, start_node):
        threading.Thread.__init__(self) 
        self.name = name
        self.start_node = start_node

        # Publisher to mission control
        self.pub = None
    
    # Execute thread. When completed, ask the mission manager to execute the next one
    def run(self):
        self.pub =  rospy.Publisher('/alfred/mission_control/', String, queue_size=1)

        # Execute thread
        rospy.loginfo('Starting thread with ID = %s' % self.name)
        self.start_node.execute()
        rospy.loginfo('Finished thread with ID = %s' % self.name)
        
    # Stop by publishing 'cancel'  
    def stop(self):
        return self.pub.publish('cancel')

    # Check if the thread is currently executing a task
    def is_active(self):
        return self.is_alive()

    def __repr__(self):
        return 'name=%s, start_node=%s' % (self.name, self.start_node)

