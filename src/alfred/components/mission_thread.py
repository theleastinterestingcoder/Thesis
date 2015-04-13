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
    mission_manager = None

    def __init__(self, name, cb):
        threading.Thread.__init__(self) 
        self.name = name
        self.cb = cb

        # Publisher to mission control
        self.pub = rospy.Publisher('/alfred/mission_control/', String, queue_size=1)
    
    
    # Execute thread. When completed, ask the mission manager to execute the next one
    def run(self):
        # Execute thread
        rospy.loginfo('Starting thread with ID = %s' % self.name)
        self.cb.callback()
        rospy.loginfo('Finished thread with ID = %s' % self.name)
        
        # Execute next mission in mission manager's queue
        if mission_thread.mission_manager:
            mission_thread.mission_manager.execute_next_mission(forced=True)
    
    # Is this mission_thread alive?
    def is_alive(self):
        return self.thread.is_alive() 
    
    def join(self):
        return self.thread.join()

    def stop(self):
        return self.pub.Publish('cancel')
        
    def __repr__(self):
        return 'name=%s, cb=%s' % (self.name, self.cb)

# Sets the mission manager for all mission_threades
def mp_set_mission_manager(mm):
    mission_thread.mission_manager = mm
