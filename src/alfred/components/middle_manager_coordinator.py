'''
    middle_manager_coordinator.py

    written by Quan Zhou on May 5th, 2015

    A module for managing resources via primitive actions
'''
import rospy
from std_msgs.msg import String

class coordinator():
    def __init__(self, cc):
        self.name = 'aux_manager'
        self.pub = rospy.Publisher('/alfred/mission_control', String, queue_size=1)
        self.core_component = cc

    # handles the cancel command
    def cancel(self):
        self.pub.publish('cancel')
        self.stop_motion()
        rospy.sleep(0.1)
        return True

    # Stops the base
    def stop_motion(self):
        self.core_component.rvc.stop()
        self.core_component.ngm.cancel_goals()
        rospy.sleep(0.1)
        return True
    
    # Cleans up all the resources
    def reset(self):
        self.cancel()
        self.core_component.frs.cleanup()
        return True
    
    # Cleanup
    def cleanup(self):
        self.reset()
