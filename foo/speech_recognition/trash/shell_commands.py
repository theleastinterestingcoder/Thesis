from subprocess import call
import os
stream = os.popen(" emacs -nw foo.txt")
stream.print("hello world")
