'''
    sleep_module.py

    written by Quan Zhou

    A component of middle managers that contains methods for interruptable sleep. 

    ----- Usage -----

    class middlemanager:
        def __init__(self):
            self.sleep_module = sleep_module()

        def my_func():
            if interruptable_sleep(10):
                return "I was interrupted"
            else:
                return "I was not interrupted"
'''
import rospy
from std_msgs.msg import String

class sleep_module:
    def __init__(self, manager):
        self.manager = manager
    
    # Handle that triggers on messages published to '/alfred/mission_control'
    def handle_mission_control(self, data):
        if data.data == 'cancel':
            self.is_interrupted=True
        else:
             rospy.loginfo("'%s'Sleep Module: Unrecognized command '%s' " % (self.manager.name, data.data))   

    # Sleep for the specified timeout. If duration=None is specified, then goes into infinite loop.  Returns true if interrupted, false if not
    def interruptable_sleep(self, duration):
        self.is_interrupted = False
        self.sub = rospy.Subscriber('/alfred/mission_control', String, self.handle_mission_control)
        ans = self.wait_and_listen_for_interrupt(duration)
        self.sub.unregister()
        self.is_interrupted = False
        return ans

    # -- Private functions -----
    # Listens for interrupts
    def wait_and_listen_for_interrupt(self, duration):
        rate = rospy.Rate(10) # 10 Hz
        start = rospy.Time.now()
        elapsed = (rospy.Time.now() - start).to_sec()
        # If no duration is specified, go to infinite sleep
        if duration == None:
            while True:
                if self.is_interrupted:
                    return True
                rate.sleep()
        else:           
            while (elapsed < duration):
                if self.is_interrupted:
                    return True
                elapsed = (rospy.Time.now() - start).to_sec()
                rate.sleep()
        return False


