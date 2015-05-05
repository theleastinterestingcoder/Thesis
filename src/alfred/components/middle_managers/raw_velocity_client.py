'''
    raw_velocity_client.py

    written by Quan Zhou

    Wraps calls to the raw_velocity_commander.py node so that
    the methods are primitive actions
'''
import time
import rospy

from sleep_module import sleep_module
from std_msgs.msg import String

class raw_velocity_client():
    # Constructor for this module
    def __init__(self, coordinator):
        self.name = 'raw_velocity_client'
        self.pub = rospy.Publisher('/alfred/raw_vel_commander/', String, queue_size=1)
        self.sleep_module = sleep_module(self)
        self.coordinator = coordinator
    
    # ---- primitive actions ----- 

    #If no duration is specified, then spin forever until it is canceled
    def move_foward(self, duration=None):
        self.coordinator.stop_motion()
        self.pub.publish('move foward')
        # Let the turtlebot move for the specified time. Then stop it
        is_interrupted = self.sleep_module.interruptable_sleep(duration)
        self.stop()

        if is_interrupted:
            return None
        return True

    def move_backward(self, duration=None):
        self.coordinator.stop_motion()
        self.pub.publish('move backward')
        is_interrupted = self.sleep_module.interruptable_sleep(duration)
        self.stop()

        if is_interrupted:
            return None
        return True

    def turn_left(self, duration=None):
        self.coordinator.stop_motion()
        self.pub.publish('move backward')
        is_interrupted = self.sleep_module.interruptable_sleep(duration)
        self.stop()

        if is_interrupted:
            return None
        return True

    def turn_right(self, duration=None):
        self.coordinator.stop_motion()
        self.pub.publish('move backward')
        is_interrupted = self.sleep_module.interruptable_sleep(duration)
        self.stop()

        if is_interrupted:
            return None
        return True

    def stop(self):
        self.pub.publish('stop')
        return True

if __name__ == "__main__":
    rvc = raw_velocity_client()
    rvc.move_foward()
