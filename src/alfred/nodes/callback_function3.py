'''
  callback_function.py
  
  written by Quan Zhou on 3/22/15

  A wrapper class that is a special callback function with parameters
'''

import pdb
import NavGoalManager
# A normal function with callback
class normal:
    def __init__(self, *args, **kwargs):
        self.function = kwargs.pop('function')
        self.p_kwargs = kwargs
        self.p_args = args
        
        self.success_cb = kwargs.pop('success_cb', None)
        self.fail_cb = kwargs.pop('fail_cb', None)
        self.done_cb = kwargs.pop('done_cb', None)

    def call_back(self, *args, **kwargs):
        ans = self.function(*self.p_args, **self.p_kwargs)
        
        if hasattr(self, 'done_cb'):
            return self.done_cb.callback()
        elif ans == True:
            self.success_cb.callback()
        else:
            self.fail_cb.callback()
# A normal callback function class
class cb_func:
    server_cb_func = [NavGoalManager.go_to_location]


    class server(normal):
        def __init__(

