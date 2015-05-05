'''
    kobuki_sound_manager.py

    written by Quan Zhou on 3/19/15

    Contains a set of wrapper functions for sound. 

    Sound cannot be canceled by 'cancel' to /alfred/command, since
    this should not be interruptable
'''
import rospy
from kobuki_msgs import msg
import time

class kobuki_sound_manager:
    def __init__(self):
        self.pub = rospy.Publisher('/mobile_base/commands/sound', msg.Sound, queue_size=10, latch=True)


    def beep(self, val=1, done_cb=None):
        m = msg.Sound()
        m.value = val
        self.pub.publish(m)

        return True

if __name__=='__main__':
    rospy.init_node('ksm_tutorial')
    ksm = kobuki_sound_manager()
    rospy.loginfo('Asking Kobuki Base to Beep')
    
    ksm.beep(1)
    rospy.spin()  #Puts the main in an infinite loop so it doesn't exit too quickly
