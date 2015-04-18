'''
    control_thread.py

    written by: Quan Zhou on April 8th, 2015

    Watches and times out a mission_thread. Takes two parameters:
    - a mission_thread to watch
    - A timeout:
        'time': time in seconds (int)
        'mission': 
            'name': String
            'start_node' : node
            'do_now' : Boolean
'''
import time, threading
import rospy

class control_thread(threading.Thread):
    mission_manager = None
    
    def __init__(self, mission_thread, timeout):
        threading.Thread.__init__(self) 
        self.mission_thread = mission_thread
        self.timeout = timeout

        self.is_interrupted = False
    

    def run(self):
        # Set the timer
        rospy.loginfo('Setting timeout with time=%s seconds' % self.timeout['time'])

        start = rospy.Time.now()
        
        # Check for interrupts on the Control Thread
        while ((rospy.Time.now() - start).to_sec() < self.timeout['time']):
            if self.is_interrupted:
                self.is_interrupted = False
                return None
            time.sleep(0.1)

    
        # Kill mission process if it is still running
        if self.mission_thread.is_alive():
            self.mission_manager.reset()
            rospy.loginfo('mission "%s" has been timed out (%s s)' % (self.mission_thread.name, self.timeout['time']))
        else:
            return None

        # Execute next mission
        if self.timeout['mission']:
            control_thread.mission_manager.handle_request(**self.timeout['mission'])

    def stop(self):
        rospy.loginfo('mission_manager.control_thread: timeout %s has been canceled' % self.timeout['mission']['name'])
        self.is_interrupted = True


# Sets the mission manager for all control_threades
def cp_set_mission_manager(mm):
    control_thread.mission_manager = mm
