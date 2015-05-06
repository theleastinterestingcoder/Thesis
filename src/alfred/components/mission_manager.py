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
import rospy, pdb, threading

from control_thread import control_thread, cp_set_mission_manager
from mission_thread import mission_thread, mp_set_mission_manager

class mission_manager():
    def __init__(self, coordinator):
        # Instance Variables
        self.mission_thread = None  # executes the mission 
        self.control_thread = None  # contains timers
        self.coordinator = coordinator
        
        # Give the pointer to the thread manager to allow thread manager to manage missiosn
        cp_set_mission_manager(self)
        mp_set_mission_manager(self)

        # List of (mission_t, timeout) tuples
        self.mission_queue =[]           

        self.queue_watcher = threading.Thread(target=self.mm_thread_init)
        self.is_watching = True
        self.queue_watcher.start()

        rospy.on_shutdown(self.cleanup)

    
    # If a thread is not running, then pull the next one
    def mm_thread_init(self):
        r = rospy.Rate(5)
        while (self.is_watching):
            # rospy.loginfo('mission_queue=%s' % self.mission_queue)
            if self.mission_thread or not self.mission_queue:
                # rospy.loginfo('mission_thread = %s' % str(self.mission_thread))
                # rospy.loginfo('control_thread = %s' % str(self.control_thread))
                pass
            else:
                self.execute_next_mission()
            r.sleep()

   # Handle requests from the core module. This function is exposed ot the coordinator
    def handle_request(self, name, start_node, timeout=None, do_now=False): 
       mission_t = mission_thread(name, start_node)
       # Drop everything and do this mission
       if do_now or not self.mission_queue:
           if self.is_active():
                self.stop()
           self.mission_queue.insert(0, (mission_t, timeout))
       else:
            # Else, add it to the mission queue
            self.mission_queue.append((mission_t, timeout))

    # Clears active missions and clear the queue
    def clear_mission_queue(self):
        self.stop()
        self.mission_queue = []


    # start the mission on the queue
    def execute_next_mission(self):
        # Do not execute mission_queue on empty queue or if there is an active mission
        if not self.mission_queue:
            rospy.loginfo('Mission Manager: mission queue is empty')
            return None
        if self.mission_thread and self.mission_thread.is_active():
            rospy.loginfo('Mission thread is current active with mission name=%s' % self.mission_thread.name) 

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
    
    # Kill current processes 
    def stop(self):
        if self.mission_thread:
            self.mission_thread.stop()
            self.mission_thread = None
        if self.control_thread:
            self.control_thread.stop()
            self.control_thread = None

    # Is there an active mission being managed?     
    def is_active(self):
        if not self.mission_thread:
            return False
        return self.mission_thread.is_alive()


    # Clear resources (be careful of infinite loop here)
    def reset(self):
        self.stop()
        self.coordinator.reset()
        self.mission_thread = None
        self.control_thread = None

    def cleanup(self):
        self.reset()
        self.is_watching = False







        
