#from subprocess import Popen, PIPE, STDOUT
#p = Popen('[echo]', stdout=PIPE, stdin=PIPE, stderr=PIPE)
#stdout_data = p.communicate(input='hello world')[0]
#print stdout_data

from subprocess import Popen, PIPE, STDOUT

p = Popen(['emacs', '-nw'], stdout=PIPE, stdin=PIPE, stderr=STDOUT)    
grep_stdout = p.communicate(input=b'one\ntwo\nthree\nfour\nfive\nsix\n')[0]
print(grep_stdout.decode())
