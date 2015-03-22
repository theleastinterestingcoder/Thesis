'''
  callback_function.py
  
  written by Quan Zhou on 3/19/15

  A wrapper class that is a special callback function with parameters
'''

import pdb

class cb_func:
    def __init__(self, *args, **kwargs):
        self.function = kwargs.pop('function')
        self.p_kwargs = kwargs
        self.p_args = args

    def call_back(self, *args, **kwargs):
        return self.function(*self.p_args, **self.p_kwargs)

