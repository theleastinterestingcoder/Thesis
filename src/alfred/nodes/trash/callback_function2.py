'''
    callback_function2.py

    A more complicated version of a callback function. 
    Chains callback functions together
'''

def cb_func:
    def __init__(self, *args, **kwargs):
        # Initialize the payload
        self.success_func = None
        self.ps_args = None
        self.ps_kwargs = None

        self.fail_func = None
        self.pf_args = None
        self.pf_kwargs = None
        
        self.function = kwargs.pop('function')
        self.p_args = args
        self.kwargs = kwargs
        
        self.next_cb = kwargs.pop('next_cb')
        # Change the previous cb_func's based on values passed in
        if 'done_cb' in kwargs:
            pass
        if 'success_cb' in kwargs:


             

    def callback(self, *args, **kwargs):
        # Execute self based on precondition
        if self.precondition == True or self.precondition == None and cond == True:
            ans = self.success_function(*self.ps_args, **self.ps_kwargs)
        if self.precondition == False and cond == False:
            ans = self.fail_function(*self.pf_args, **self.pf_kwargs)

        # pass answer to next callback function
        if ans == True:
            self.next_cb.callback(cond==ans)


