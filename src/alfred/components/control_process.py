'''
    control_process.py

    written by: Quan Zhou on April 8th, 2015

    Watches and times out a missionprocess 
'''
import time, multiprocessing
import rospy

class control_process():
    mission_manager = None
    
    def __init__(self, mission_process, timeout):
        self.process = multiprocessing.Process(target=cb.callback) 
        self.mission_process = mission_process
        self.timeout = timeout
    

    def run(self):
        # Set the timer
        rospy.loginfo('Setting timeout with time=%s seconds')
        time.sleep(timeout['time'])
    
        # Kill mission process if it is still running
        if mission_process.is_alive():
            self.mission_process.stop()
            self.mission_manager.cleanup()
            rospy.loginfo('mission "%s" has been timed out (%s s)' % (self.mission_process.name, timeout['time']))
        else:
            return None

        # Execute next mission
        if timeout['mission']:
            control_process.mission_manager.handle_request(*timeout['mission'], do_now=True)

    def is_alive(self):
        return self.process.is_alive()

    def start(self):
        return self.process.start()
# Sets the mission manager for all control_processes
def cp_set_mission_manager(mm):
    control_process.mission_manager = mm
