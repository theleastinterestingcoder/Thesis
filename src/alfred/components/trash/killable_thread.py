'''
    killable_thread.py

    mimics the instance of a thread using subprocesses. 

    Limitations: need to pass a cleanup method
'''
import signal, process
from multiprocessing import pool

def killable_thread():

    def __init__(name, func, args, kwargs, cleanup_func):
#         self.func = func
#         self.args = args
#         self.kwargs = kwargs
        if args and kwargs:
            self.subprocess = Process(target=func, args=args, kwargs=kwargs)
        elif args and not kwargs:
            self.subprocess = Process(target=func, args=args)
        elif not args and kwargs:
            self.subprocess = Process(target=func, kwargs=kwargs)
        elif not args and not kwargs:
            self.subprocess = Process(target=func)
        
    # Executes the function
    def run():    
        self.subprocess.start()

    def suspend():
        self.subprocess.

    def resume():
        pass

    def kill():
        pass

    def cleanup():
        pass

    def is_alive():
        pass
