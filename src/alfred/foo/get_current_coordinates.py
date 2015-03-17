from tf import TransformListener
from tf.transformations import euler_from_quaternion


import rospy, time

rospy.init_node('foo_get_current_coordinates')
tfl = TransformListener()

base_f = rospy.get_param('~base_frame', '/base_link')
map_f  = rospy.get_param('~map_frame', '/map')
# now = rospy.Time.now()
# tfl.waitForTransform(map_f, base_f, now, rospy.Duration(4.0))    
#time.sleep(2)
# pos, quat = tfl.lookupTransform(map_f, base_f, rospy.Time(0))


#tfl.waitForTransform(map_f, base_f, rospy.Time(), rospy.Duration(4.0))

now = rospy.Time(0)
tfl.waitForTransform(map_f, base_f, now, rospy.Duration(4.0))
(pos,quat) = tfl.lookupTransform(map_f, base_f, now)
# 
euler = euler_from_quaternion(quat)
# 
print "Turtlebot location: (%s, %s, %s)" % (pos[0], pos[1], euler[2])

