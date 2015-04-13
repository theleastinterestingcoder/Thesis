'''
    mission_manager.py

    written by Quan Zhou on April 8th, 2015

    Mission manager contains a list of mission_queue and executes them one by one. 
    
    Missions contain:
        - a chained callback function
        - an optional timeout for the callback function

'''
import rospy

from mission_thread import mission_thread, mp_set_mission_manager
from control_process import control_process, cp_set_mission_manager

class mission_manager():
    def __init__(self, core_module):
        # Instance Variables
        self.mission_thread = None  # executes the mission 
        self.control_process = None  # contains timers
        self.core_module = core_module
        
        # Assign this mission manager to fields of the followign classes
        mp_set_mission_manager(self)
        cp_set_mission_manager(self)

        # List of (mission_t, timeout) tuples
        self.mission_queue =[]           

        # A pubisher to mission_control

   # Handle requests from the core module
    def handle_request(self, name, cb, timeout=None, do_now=False): 
       mission_t = mission_thread(name, cb)
       # Drop everything and do this mission
       if do_now or not self.mission_queue:
           self.mission_queue.insert(0, (mission_t, timeout))
           self.execute_next_mission()
       else:
            # Else, add it to the mission queue
            self.mission_queue.append((mission_t, timeout))

    # start the mission on the queue
    def execute_next_mission(self, forced=False):
        # Do not execute mission_queue on empty queue or active queue
        if not self.mission_queue:
            return None
        # Avoid executing more than one mission at a time
        if not forced and self.mission_thread and self.mission_thread.is_active():
            return None
       
       #Execute the mission
#         self.cleanup()
        mission_t, timeout = self.mission_queue.pop(0)
        self.mission_thread = mission_t
        self.mission_thread.start()
        

        # If there is a timeout, then wait 
        if timeout and timeout['time'] > 0:
            self.control_process = control_process(mission_t, timeout)
            self.control_process.start()

        return True
   
    # Start the mission manager
    def start(self):
        if self.is_active() or not self.mission_queue:
            return
        else:
            self.execute_next_mission()
    
    # Kill current processes 
    def stop(self):
        if self.mission_thread:
            self.mission_thread.stop()
        if self.control_process:
            self.control_process.stop()
            
    def is_active(self):
        return self.mission_thread.is_alive()

    def pause(self):
        self.mission_thread.join()
        self.control_process.join()

    # Clear resources (be careful of infinite loop here)
    def cleanup(self):
        self.stop()
        self.core_module.cleanup()
        self.mission_thread = None
        self.control_process = None







        
