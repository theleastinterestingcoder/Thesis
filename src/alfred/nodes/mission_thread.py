'''
    mission_thread.py

    written by Quan Zhou on 3/28

    A thread for managing a mission
'''
import threading
import rospy
import pdb

class mission_thread(threading.Thread):
    def __init__(self, name, cb):
        threading.Thread.__init__(self)
        self.name = name
        self.cb = cb

    def run(self):
        rospy.loginfo('Starting Thread with ID = %s' % self.name)
        self.cb.callback()
        rospy.loginfo('Finished Thread with ID = %s' % self.name)
        
