#!/usr/bin/env python

"""
  face_detection_spawner.py

   Written by Quan Zhou on 3/19

  spawns face the suite of face detection packages.   
  
"""

import roslib; roslib.load_manifest('pi_speech_tutorial')
import rospy
import pdb
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import copysign

from nav_goal_manager import nav_goal_manager
from sleep_module import sleep_module
import os, subprocess, time, signal

class face_recognition_spawner:
    def __init__(self, coordinator):
        self.name = 'face_recognition_spawner'
        # Set core parameters, functions, and subscriptions 
        self.is_busy = False
        rospy.on_shutdown(self.cleanup)
        
        # For canceling and cleaning out resources
        self.coordinator = coordinator
        self.sleep_module = sleep_module(self)
    
    # Initialize all the nodes required for core processes
    def init_nodes(self):
        if not self.is_busy:
            self.gscam = self.launch_gscam()
            self.fclient = self.launch_fclient()
            self.fquan = self.launch_fquan()
            self.is_busy=True
        else:
            rospy.info('face_detection_spawner tried to initialize nodes but was locked! [failed]')
    
    # Blocks this function until we see a name of a person
    def spin_and_monitor(self, names, duration):
        # everytime you see a face, add it to the queue
        fq = self.face_queue()
        sub = rospy.Subscriber('recognized_face', String, fq.add)

        # clear the queue at a rate of 10hz
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        found = False
        
        # at every tick, pull items off the queue and examine it
        while (rospy.Time.now() - start).to_sec() < duration:
            # Check for interrupt flag
            if self.sleep_module.interruptable_sleep(0.5):
                sub.unregister()
                return None

            while len(fq.queue) > 0:
                item = fq.queue[0]
                if item[0] in names:
                    sub.unregister()# unsubscribe from the feed
                    return item[0]
                else:
                    del fq.queue[0]
            rate.sleep()
        # nothing has been found in the time duration      
        sub.unregister() 
        return ""

    # Primitive Action: a wrapped version of spin and monitor that has the properties of a primitive action
    def look_for_face(self, names = ['Quan'], duration=10, done_cb=None):
        # Allow only face recognition function to be working at a time
        if not self.is_busy:
            self.init_nodes() # Locks the fds
            self.start_pub()
         
            # Check if a face has been seen
            person = self.spin_and_monitor(names, duration)
            if person:                                                                                                                                
                rospy.loginfo('FRS: %s has been found' % person)
                ans = True
            elif person == "":
                rospy.loginfo('FRS: waited for %s, but did not find anyone in %s' % (duration, names))
                ans = False
            else:
                rospy.loginfo('FRS: look_for_face was canceled')
                ans = None


            self.stop_pub()
            self.kill_nodes() # Releases the lock
        else:
            rospy.info('Tried to look for face, but Face Detection Service is busy!')
        return ans
    
#     # Primitive Action: Trains a face upon the message 'go'
#     def train_face(self, name):
#         # Start the face recognition
#         if not self.is_busy:
#             self.init_nodes() 
#             self.start_pub()
#             
#             # Start face recognition upon hearing the command 'start'
#             sub = rospy.Subscriber('/recognizer/output', String, self.recognizer_cb)
#             rospy.loginfo('FRS: Ready to record face. Please say "start" to start training your face')
# 
#             self.training_flag = False
#             # Block until recognizer_cb sets the training flag to be true. Also checks the interrupt flag. 
#             while (self.training_flag is not True):
#                 # Check for interrupts
#                 if self.is_interrupted:
#                     self.is_interrupted = False
#                     self.stop_pub()
#                     self.unregister()
#                     self.kill_nodes()
#                     return None
#                 rospy.sleep(0.1)
#             
#             # Train and block for the duration of the trianing (can't be interrupted)
#             subprocess.call("rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 4 \"none\"", shell=True )
#             rospy.sleep(2.5)
# 
#             self.stop_pub()
#             self.kill_nodes()
#             sub.unregister()
#             rospy.loginfo('FRS: Done recording face.')
#             return True

    # Call back function 
    def recognizer_cb(self, data):
        if data.data == 'start':
            self.training_flag = True

    # get the camera up and running
    def launch_gscam(self):
        pwd = os.getcwd()
        # setup gscam (kinda hacky and awkward, but it'll work)
        os.chdir("/bin")
        if os.path.isfile('/home/alfred/quan_ws/src/gscam/bin/gscam'):
            subprocess.call('mv /home/alfred/quan_ws/src/gscam/bin/gscam /home/alfred/quan_ws/src/gscam/bin/gscam_tmp', shell=True)
            rospy.loginfo('temporarily renamed gscam')
        subprocess.call("cd /bin/", shell=True)
        new_env = os.environ.copy()
        new_env['GSCAM_CONFIG']= "v4l2src device=/dev/video0 ! video/x-ra w-rgb ! ffmpegcolorspace"
        
        
        # get the webcam up
        gscam = subprocess.Popen("rosrun gscam gscam", stdout=subprocess.PIPE, preexec_fn=os.setsid, shell=True, env=new_env)
        
        # restore the hacky thingy, after a small time delay
        time.sleep(0.5)
        if os.path.isfile('/home/alfred/quan_ws/src/gscam/bin/gscam_tmp'):
            subprocess.call('mv /home/alfred/quan_ws/src/gscam/bin/gscam_tmp /home/alfred/quan_ws/src/gscam/bin/gscam', shell=True)
            rospy.loginfo('undid rename change to gscam')
        return gscam

    # get face recognition server up and running
    def launch_fclient(self):
        fclient = subprocess.Popen("rosrun face_recognition Fserver", stdout=subprocess.PIPE, preexec_fn=os.setsid, shell=True)
        return fclient

    # get the face recognition client up and running
    def launch_fquan(self):
        fquan   = subprocess.Popen("rosrun face_recognition Fpublisher", stdout=subprocess.PIPE, preexec_fn=os.setsid, shell=True)
        return fquan

    def launch_response(self):
        response = subprocess.Popen("rosrun alfred face_recognizer.py", stdout=subprocess.PIPE, preexec_fn=os.setsid, shell=True)
        return response

    # start publishing, which launches the publishing window
    def start_pub(self):
        subprocess.call("rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 1 \"none\"", shell=True )

    # stop publishing
    def stop_pub(self):
        subprocess.call("rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 4 \"none\"", shell=True )


    # stop all the processes
    def kill_nodes(self):
        if self.is_busy:
            rospy.loginfo('Shutting down gscam, fclient, fquan, and response')
            os.killpg(self.gscam.pid, signal.SIGTERM)  
            os.killpg(self.fclient.pid, signal.SIGTERM)  
            os.killpg(self.fquan.pid, signal.SIGTERM)  
#             os.killpg(self.response.pid, signal.SIGTERM)  
            self.is_busy=False

    # Cleanup
    def cleanup(self):
        self.kill_nodes()
        self.is_interrupted = False

    # ---- data structure for remembering faces-----
    class face_queue():
        def __init__(self):
            self.queue = []
            self.i = 0 # iterable
        
        def add(self, data):
            self.queue.append((data.data, rospy.Time.now()))

 
if __name__=="__main__":
    frs = face_recognition_spawner()
    frs.init_nodes() # launches all the necessary processes
    frs.start_pub()  # starts scanning for faces by publishing to fr_order -- 1 "none"

    if frs.spin_and_monitor(['Quan'], 10):
        rospy.loginfo('Quan Found')
    else:
        rospy.loginfo('Quan Not found in 10 seconds')

    frs.stop_pub()
    frs.kill_nodes() # 


    

