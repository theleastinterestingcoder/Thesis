'''
  callback_function.py
  
  written by Quan Zhou on 3/19/15

  A wrapper class that is a special callback function with parameters
'''

import pdb

class cb_func:
    def __init__(self, function, **kwargs):
        self.function = function
        self.param = kwargs

    def call_back(self, *args, **kwargs):
        return self.function(**self.param)

