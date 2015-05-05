'''
    raw_velocity_client.py

    written by Quan Zhou

    Wraps calls to the raw_velocity_commander.py node so that
    the methods are primitive actions
'''
import time
import rospy

from sleep_module import sleep_module

class raw_velocity_client():
    def __init__(self):
        self.name = 'raw_velocity_client'
        self.pub.publish = rospy.Publisher('/alfred/raw_vel_commander/', String, queue_size=1)
        self.sleep_module = sleep_module()
    
    def move_foward(self, duration=None):
        self.pub.publish('move foward')
        if duration:
            is_interrupted = self.sleep_module.interruptable_sleep(self, duration)
        self.stop()

        if is_interrupted:
            return None
        return True

    def move_backward(self, duration=None):
        self.pub.publish('move backward')
        if duration:
            is_interrupted = self.sleep_module.interruptable_sleep(self, duration)
        self.stop()

        if is_interrupted:
            return None
        return True

    def turn_left(self, duration=None):
        self.pub.publish('move backward')
        if duration:
            is_interrupted = self.sleep_module.interruptable_sleep(self, duration)
        self.stop()

        if is_interrupted:
            return None
        return True

    def turn_right(self, duration=None):
        self.pub.publish('move backward')
        if duration:
            is_interrupted = self.sleep_module.interruptable_sleep(self, duration)
        self.stop()

        if is_interrupted:
            return None
        return True

    def stop(self):
        self.pub.publish('stop')
        return True

    # ---- Methods that handle mission control stuff------

