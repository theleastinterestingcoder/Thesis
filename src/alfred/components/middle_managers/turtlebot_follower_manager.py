'''
    turtlebot_follower_manager.py

    Written by Quan Zhou on May 11, 2015

    encapsulates calls to turtlebot follower.launch
'''

import time, rospy, os, subprocess, time, signal
from sleep_module import sleep_module

class turtlebot_follower_manager():
    def __init__(self):
        self.name = 'raw_velocity_client'
        self.subprocess = None
        self.sleep_module = sleep_module(self)


    def follow(self, duration=15):
        if self.subprocess:
            raise Exception('TFM Error: follow command was sent but turtlebot is already set to follow')
        
        self.subprocess = subprocess.Popen("roslaunch alfred follower.launch",
                                            stdout=subprocess.PIPE, 
                                            preexec_fn=os.setsid, 
                                            shell=True) 
        # Pause and wait for interrupt
        if self.sleep_module.interruptable_sleep(duration):
            ans = None
        else:
            ans = True
        
        # Checks if process has been killed already
        if self.unfollow():
            return ans
        return None

    def unfollow(self):
        if not self.subprocess:
            return None
        os.killpg(self.subprocess.pid, signal.SIGTERM)
        self.subprocess = None

if __name__ == '__main__':
    rospy.init_node('follower_tutorial')
    tfm = turtlebot_follower_manager()
    tfm.follow()
