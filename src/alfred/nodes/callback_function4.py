'''
  callback_function.py
  
  written by Quan Zhou on 3/19/15

  A wrapper class that is a special callback function with parameters
'''

import pdb
import rospy

class cb_func:
    def __init__(self, *args, **kwargs):
        self.function = kwargs.pop('function')
        self.p_kwargs = kwargs
        self.p_args = args

    def call_back(self, *args, **kwargs):
        if args and args[0] != 3:
            rospy.loginfo('Callback sequence suppressed since goal was not achieved: %s' % self)
            return False
        return self.function(*self.p_args, **self.p_kwargs)

    def __repr__(self):
        return "{function: %s\nargs: %s\n kwargs: %s}\n" % (self.function, self.p_args, self.p_kwargs)

