'''
    mission_manager.py

    written by Quan Zhou on April 8th, 2015

    Mission manager contains a list of mission_queue and executes them one by one. 
    
    Constructor takes:
    - coordinator as the input

    Missions contain:
        'name' : String
        'start_node': starting node

    Timeout contains:
        'time': time in seconds (int)
        'mission':
            'name': String
            'start_node': starting node
            

    IMPORTANT: Mission manager needs the core module to call coordinator.reset()

'''
import rospy, pdb

from control_thread import control_thread, cp_set_mission_manager

class mission_manager():
    def __init__(self, coordinator):
        # Instance Variables
        self.mission_thread = None  # executes the mission 
        self.control_thread = None  # contains timers
        self.coordinator = coordinator
        
        # Give the pointer to the thread manager to allow thread manager to manage missiosn
        cp_set_mission_manager(self)

        # List of (mission_t, timeout) tuples
        self.mission_queue =[]           
    
    # Asks mission manager to constantly poll itself for new missinos 
    def mm_thread_init(self):
        r = rospy.Rate(10)
        while (True):
            self.start()
            r.sleep()

   # Handle requests from the core module. This function is exposed ot the coordinator
    def handle_request(self, name, start_node, timeout=None, do_now=False): 
       mission_t = mission_thread(name, start_node)
       # Drop everything and do this mission
       if do_now or not self.mission_queue:
           self.mission_queue.insert(0, (mission_t, timeout))
           self.execute_next_mission()
       else:
            # Else, add it to the mission queue
            self.mission_queue.append((mission_t, timeout))

    # start the mission on the queue
    def execute_next_mission(self):
        # Do not execute mission_queue on empty queue or if there is an active mission
        if not self.mission_queue:
            rospy.loginfo('Mission Manager: mission queue is empty')
            return None
        if self.mission_thread and self.mission_thread.is_active():
            rospy.loginfo('Mission thread is current active with mission name=%s and start_node=%s' %\
                          (self.mission_thread.name, self.mission_thread.start_node)) 
            return None
       
       #Execute the mission
        self.reset()
        mission_t, timeout = self.mission_queue.pop(0)
        self.mission_thread = mission_t
        self.mission_thread.start()
        
        # If there is a timeout, then wait 
        if timeout and timeout['time'] > 0:
            self.control_thread = control_thread(mission_t, timeout)
            self.control_thread.start()

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
        if self.control_thread:
            self.control_thread.stop()

    # Is there an active mission being managed?     
    def is_active(self):
        return self.mission_thread.is_alive()


    # Clear resources (be careful of infinite loop here)
    def reset(self):
        self.stop()
        self.coordinator.reset()
        self.mission_thread = None
        self.control_thread = None







        
