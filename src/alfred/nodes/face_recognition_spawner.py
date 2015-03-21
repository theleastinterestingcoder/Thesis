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

from NavGoalManager import NavGoalManager
import os, subprocess, time, signal

class face_recognition_spawner:
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node("Face_Detection_Package_Manager")
            rospy.loginfo("Created node")
            
        self.is_busy = False
        # launch everything as needed
        rospy.on_shutdown(self.cleanup)
    
    # start all the processes
    def init_nodes(self):
        if not self.is_busy:
            self.gscam = self.launch_gscam()
            self.fclient = self.launch_fclient()
            self.fquan = self.launch_fquan()
#             self.response = self.launch_response()
            self.is_busy=True
        else:
            rospy.info('face_detection_spawner tried to initialize nodes but was locked! [failed]')
    
    # pause and see if we can see a list of people 
    def watch_for(self, names, duration):
        # everytime you see a face, add it to the queue
        fq = self.face_queue()
        sub = rospy.Subscriber('recognized_face', String, fq.add)

        # clear the queue at a rate of 10hz
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        found = False

        # at every tick, pull items off the queue and examine it
        while (rospy.Time.now() - start).to_sec() < duration:
            while len(fq.queue) > 0:
                item = fq.queue[0]
                if item[0] in names:
                    sub.unregister()# unsubscribe from the feed
                    return True
                else:
                    del fq.queue[0]
        # nothing has been found in the time duration      
        sub.unregister() 
        return False

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
    def kill_everything(self):
        if self.is_busy:
            rospy.loginfo('Shutting down gscam, fclient, fquan, and response')
            os.killpg(self.gscam.pid, signal.SIGTERM)  
            os.killpg(self.fclient.pid, signal.SIGTERM)  
            os.killpg(self.fquan.pid, signal.SIGTERM)  
#             os.killpg(self.response.pid, signal.SIGTERM)  
            self.is_busy=False
        else:
            rospy.loginfo("face_detection_spawner tried to kill nodes that were already dead")

    # get the face recognition client up and running
    def cleanup(self):
        # when shutting down be sure to stop the robot! 
        self.kill_everything()
    
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

    if frs.watch_for(['Quan'], 10):
        rospy.loginfo('Quan Found')
    else:
        rospy.loginfo('Quan Not found in 10 seconds')

    frs.stop_pub()
    frs.kill_everything() # 
# if __name__== "__main__":
#     frs = face_recognition_spawner()
#     frs.launch_gscam()
#     fclient = subprocess.Popen("rosrun face_recognition Fserver", stdout=subprocess.PIPE, preexec_fn=os.setsid, shell=True)
#     fquan   = subprocess.Popen("rosrun face_recognition Fpublisher", stdout=subprocess.PIPE, preexec_fn=os.setsid, shell=True)
#     subprocess.call("rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 1 \"none\"", shell=True )


    

