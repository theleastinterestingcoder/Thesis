'''
  face_recognizer.py

  written by Quan Zhou on 3/17

  Makes the turtlebot chirp everytime a person's face is recognized. 

  Subscribes to 
'''
import rospy
from std_msgs.msg import String
from kobuki_msgs import msg


class face_recognizer:
    def __init__(self, name='Quan', sub='recognized_faces'):
        rospy.init_node('alfred_face_response');
        rospy.Subscriber(sub, String, self.face_response)
        self.pub = rospy.Publisher('/mobile_base/commands/sound', msg.Sound, queue_size=10)

        self.name = name


    def face_response(self, data):
        if data.data == self.name:
            rospy.loginfo("%s has been found!" % self.name)
            m = msg.Sound()
            m.value = 1
            self.pub.publish(m)

if __name__ == '__main__':
    fr = face_recognizer()
    rospy.spin()







