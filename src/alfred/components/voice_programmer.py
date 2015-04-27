data = '''
Define Program 'Patrol Two Points'
    Define Comment
        This is an example of a program that tells the turtlebot to 
        patrol between two points
    End Comment

    Define Mission 'first mission'
        Start Node 'go to first location'
            Action 'go to location'
            Arguments [5,6,7]
            If success, execute Node 'go to second location'
            If fail, execute Node 'fail beep'
        End Node

        Define Node 'go to second location'
            Action 'go to location'
            Arguments eight comma one point twenty one comma three point forty one
            If success, execute Node 'go to second location'
            If fail, execute Node 'fail beep'
        End Node

        Edit Node 'go to first location':
            Arguments one comma two comma three comma four
        End node
    End Mission

    Define Modifier
        Do Now equals True
        Clear Mission Queue equals true
    End Modifier
    
        Define Timeout 'go home'
        Start Node 'go home':
        End Node
        Time 10 minutes
    End Timeout
End Program 
'''

# data = '''
# define program
# define mission patrol
#     new node go to point a
#         action 'go to alpha'
#         if success 'go to point b'
#     end node

#     start node 'go home'
#         action 'go home'
#         if success 'go to point a'
#     end node


#     new node go to point b
#         action 'go to beta'
#         if success 'go to point a'
#     end node
# end mission
# end program

# # '''
import re, math, pdb, json
from semantic import numbers
from mission_node import node

from json import JSONEncoder

ns = numbers.NumberService()

class foo_component:
    def __init__(self):
        self.ngm = foo_component.ngm()
        self.loc = {}
        self.loc['alpha'] = [1,1,1]
        self.loc['beta']  = [2,2,2]
        self.loc['home']  = [0,0,0]

    class ngm:
        def go_to_location(self):
            pass
    class ksm:
        def beep(self):
            pass
class voice_programmer:
    keywords = {}
    keywords['definition'] = ['define program']
    keywords['program'] = ['define mission', 'define timeout', 'define modifier', 'end program']
    keywords['node']    = ['action', 'if success', 'if failure', 'if fail' , 'arguments', 'argument', 'keyword arguments','keyword argument', 'end node']
    keywords['mission'] = ['new node', 'start node', 'edit node', 'edit start node', 'end mission', ]
    keywords['modifier'] = ['do now', 'do now is', 'end modifier']
    keywords['timeout'] = ['time', 'new node', 'start node',  'edit node', 'edit start node', 'end timeout']


    temp = [val for key, val in keywords.iteritems()]
    keywords['all'] = [item for sublist in temp for item in sublist] # Flattens the list


    def __init__(self, core_component=foo_component()):
        # instantiates local variables
        self.reset()

        # Do the subscriber thing
        # self.sub = rospy.Subscriber('/alfred/voice_programmer', Some sthring thingy, stuff)
        self.core_component = core_component

        self.action_to_node_dic = {
            'go to alpha' : node(function=self.core_component.ngm.go_to_location, *self.core_component.loc['alpha']),
            'go to beta' :  node(function=self.core_component.ngm.go_to_location, *self.core_component.loc['beta']), 
            'go to home' :  node(function=self.core_component.ngm.go_to_location, *self.core_component.loc['home']),
            'go home' :     node(function=self.core_component.ngm.go_to_location, *self.core_component.loc['home']),
            'abort goals': None, 
            'move foward': None, 
            'move right': None, 
            'turn left': None, 
            'turn right': None, 
            'stop': None, 
            'stop broadcast': None, 
            'start broadcast': None, 
            'cancel' : None, 
            'set mark alpha' : None, 
            'set mark beta' : None, 
            'pause speech' : None, 
            'continue speech' : None, 
            'start face recognition' : None, 
            'stop face recognition' : None, 
            'start psychotherapist' : None, 
            'success beep':   node(function=self.core_component.ksm.beep, val = 1, done_cb=None),  
            'fail beep':      node(function=self.core_component.ksm.beep, val = 2, done_cb=None), 
            'go to location': node(function=self.core_component.ngm.go_to_location), 
        }    
    # cleans up the variables in this function
    def reset(self):
        # State Variables
        self.state = ['definition']
        self.is_compiled = False

        # Metadata
        self.node_c = None
        self.raw_speech = ""
        self.speech_chunks = []

        # What we're interested in
        self.modifier = {}
        self.timeout = {}
        self.mission_nodes = []
        self.timeout_nodes = []
        self.mission_start = None
        self.timeout_start = None
        

    # Returns the current model as a string
    def get_model_as_string(self):
        print json.dumps(self.get_model_as_dict())

    # Returns the current model of oneself
    def get_model_as_dict(self):
        if self.is_compiled:
            ans = {}
            ans['modifier']           = self.modifier
            ans['timeout']            = self.timeout
            ans['mission_nodes']      = self.mission_nodes  
            ans['timeout_nodes']      = self.timeout_nodes  
            ans['mission_start_node'] = self.mission_start
            ans['timeout_start_node'] = self.timeout_start     
        else:
            throw Exception("VP Error: Program has not been compiled. Please publish 'end program' to compile.")

        return ans

    def input_buffer(self, data):
        self.raw_speech = data.replace('time out', 'timeout')
        reg = re.compile('\w+')
        self.speech_chunks += reg.findall(data.lower()) # Remember to clean the data
        
    def advance_state(self):
        self.digest_chunks()

    def digest_chunks(self):
        words = self.speech_chunks
        raw = words

        while raw:
            keyword, argument, raw = vp.get_keyword_argument(raw)
            if not keyword and not argument:
                return
            print "keyword=%s\nargument=%s\n" % (keyword, argument)
            self.is_compiled = False
            self.update_state(keyword, argument)


    def update_state(self, keyword, argument):
        # Check if keyword argument is valid
        if keyword not in voice_programmer.keywords[self.state[-1]]:
            res =  'VP Error: Invalid Syntax - keyword "%s" is not recognized in state "%s"' % (keyword, self.state[-1])
            res += '\n  Error occured when processing:\n"%s" ' % self.raw_speech
            raise Exception(res)

        # ========= Switch statement triggered on keyword =======
        keyword_to_func = {
            'define program'  : self.handle_start_program,
            'end program'     : self.handle_end_program,
            'new node'        : self.handle_new_node,
            'start node'      : self.handle_start_node,
            'edit node'       : self.handle_edit_node,
            'edit start node' : self.handle_edit_start_node,
            'end node'        : self.handle_end_node, 
        }

        substr_to_func = {
            'define' : self.handle_state_entrance,
            'end'    : self.handle_state_exit,
        }

        state_to_func = {
            'definition'    : self.handle_state_definition,
            'node'          : self.handle_state_node,
            'mission'       : self.handle_state_mission,
            'modifier'      : self.handle_state_modifier,
            'timeout'       : self.handle_state_timeout,
            'program'       : self.handle_state_program,
        }


        # Check triggered keywords first
        if keyword in keyword_to_func:
            return keyword_to_func[keyword](keyword, argument)

        # Check substrings next
        for substr in substr_to_func.keys():
            if substr in keyword:
                return substr_to_func[substr](keyword, argument)

        # Then do generalized case
        return state_to_func[self.state[-1]](keyword, argument)
        

    def get_keyword_argument(self, words):
        phrase = " ".join(words)
        keyword, loc1 = self.get_keyword(words)

        if not keyword:
            return None, None, None
        else:
            idx1 = phrase.index(keyword)

            # Check for arguments
            nextkey, loc2 = self.get_keyword(words[loc1+1:])
        
            if nextkey:
                idx2 = phrase[idx1+len(keyword):].index(nextkey)
            else:
                return keyword, "", words[loc1+1:]
            argument = phrase[idx1+len(keyword):idx1+idx2+len(keyword)]
        
        return keyword.strip(), argument.strip(), words[loc1+loc2:]


    # Go through the list of words until you hit a key word
    def get_keyword(self, words):
        phrase = ""

        # Pick out words
        for i,w in enumerate(words):
            phrase = "%s %s" % (phrase, w)
            for m in voice_programmer.keywords['all']:
                reg = re.compile(r'\b' + m + r'\b')
                if reg.search(phrase):
                    return m, i

        return "", -1

    # Finds the start node with the matching name
    def find_node(self, name, node_space, include_library=False):
        for n in node_space:
            if n.name == name:
                return n
        if include_library and name in self.action_to_node_dic:
            return self.action_to_node_dic[name]
        return None

    def find_start_node(self, node_space):
        for n in self.mission_nodes:
            if 'start_node' in n.__dict__:
                return n
        return None



    # ---- Converts keyword+arguments into real meaning -----
    def handle_start_program(self, keyword, argument):
        self.reset()
        self.state.append('program')

    def handle_end_program(self, keyword, argument):
        self.compile_nodes(self.mission_nodes)
        self.compile_nodes(self.timeout_nodes)


        # Check some invariances
        if not self.mission_nodes:
            raise Exception('VP Error: No mission_nodes were created')
        if not self.mission_start:
            raise Exception('VP Error: No start node was specified for mission')
        if not self.modifier:
            self.modifier = {'do now': False}


        self.is_compiled=True

    # Compile string->references
    def compile_nodes(self, node_space):
        if not node_space:
            return 

        # For each node, do a name lookup in the node space and the library
        for n in node_space:
            if n.ps_nd:
                found = self.find_node(n.ps_nd, node_space, include_library=True)
                if not found:
                    raise Exception('Could not not dereference the pointer "%s" for node "%s"' % (n.ps_nd, n.name))
                else:
                    n.ps_nd = found
            if n.pf_nd:
                found = self.find_node(n.pf_nd,  node_space, include_library=True)
                if not found:
                    raise Exception('Could not not dereference the pointer "%s" for node "%s"' % (n.pf_nd, n.name))
                else:
                    n.pf_nd = found
        

    def handle_start_node(self, keyword, argument):
        self.handle_new_node(keyword, argument, is_start=True)

    def handle_new_node(self, keyword, argument, is_start=False):
        self.node_c = node(function=None, name="")
        self.node_c.name = argument

        node_space = self.get_node_space()
        if self.find_node(argument, node_space):
            raise Exception("VP Error: node name '%s' already exists in node space '%s'" % self.state[-2])
        self.state.append('node')

        if is_start:
            self.node_c.start_node = True

    def handle_edit_node(self, keyword, argument, is_start=False):
        node_space = self.get_node_space()
        if is_start:
            self.node_c = self.find_start_node(node_space)
        else:
            self.node_c = self.find_node(argument, node_space)
        if not self.node_c:
            raise Exception('Error: Node "%s" could not be found' % self.node_c.name)
        self.state.append('node')

    def handle_edit_start_node(self, keyword, argument):
        self.handle_edit_node(keyword, argument, is_start=True)

    def handle_state_entrance(self, keyword, argument):
        new_s = keyword.replace('define ', '')
        if new_s in self.keywords:
            return self.state.append(new_s)
        else:
            raise Exception('VP Error: Unrecognized state "%s" in keyword "%s"' % (new_s, keyword))

    def handle_state_exit(self, keyword, argument):
        curr_s = keyword.replace('end ', '')
        if curr_s == self.state[-1]:
            self.state.pop(-1)
        else:
            raise Exception('VP Error: Trying to exit state "%s" when in state "%s"' % (self.state[-1], curr_s))

    def handle_end_node(self, keyword, argument):
        node_space = self.get_node_space()
        self.handle_state_exit(keyword, argument)

        if not self.find_node(self.node_c.name, node_space):
            node_space.append(self.node_c)

        # Case for the start node
        if 'start_node' in self.node_c.__dict__:
            if self.state[-1] == 'timeout':
                self.timeout_start = self.node_c
            else:
                self.mission_start = self.node_c

    def handle_state_definition(self, keyword, argument):
        if keyword == 'define program':
            self.state.append('program')
        else:
            raise Exception('Unrecognized keyword "%s" in handle_state_definition')

    def handle_state_node(self, keyword, argument):
        if keyword == 'action':
            # copy over the function and kwargs
            n = self.action_to_node(keyword, argument)
            self.node_c.function = n.function
            self.node_c.p_args   = n.p_args
            self.node_c.p_kwargs = n.p_kwargs
        # Keep as string; need to resolve as pointer later
        elif keyword == 'if success':
            self.node_c.ps_nd = self.parse_node_alias_from_argument(keyword, argument)
        elif keyword == 'if fail':
            self.node_c.pf_nd = self.parse_node_alias_from_argument(keyword, argument)
        elif keyword in ['arguments', 'argument']:
            self.node_c.p_args = parse_arg_as_num(argument) 
        elif keyword in ['keyword arguments', 'keyword argument']:
            self.node_c.p_kwargs = self.parse_keyword(argument)
        else:
            raise Exception('Unrecognized keyword "%s" in handle_state_node')

    # Nothing happens here
    def handle_state_program(self, keyword, argument):
        raise Exception('No arguments to be had here')
    
    # Nothing happens here
    def handle_state_mission(self, keyword, argument):
        raise Exception('No arguments to be had here')

    def handle_state_modifier(self, keyword, argument):
        if keyword in ['do now', 'dow no is']:
            if argument in ['true', 'equals true']:
                self.modifier['do_now'] = True
            elif argument in ['false', 'equals false']:
                self.modifier['do_now'] = False
            else:
                raise Exception('Invalid argument "%s" for "%s"' % (argument, keyword))
        else:
            raise Exception('Unrecognized keyword "%s" in handle_state_mission')

    def handle_state_timeout(self, keyword, argument):
        if keyword == 'time':
            self.timeout['time'] = parse_arg_as_seconds(argument)
        else:
            raise Exception('Unrecognized keyword "%s" in handle_state_mission')

    def parse_node_alias_from_argument(self, keyword, argument):
        ans = argument.replace('execute node', '').replace('node', '')
        return ans.strip()

    def action_to_node(self, keyword, argument):
        if argument not in self.action_to_node_dic:
            raise Exception('VP Error: Cannot identify argument="%s" as an action' % argument)
        return self.action_to_node_dic[argument]

    def get_node_space(self):
        prev_state = self.state[-1]
        if prev_state == 'node':
            prev_state = self.state[-2]

        if prev_state == 'mission':
            return self.mission_nodes
        elif prev_state == 'timeout':
            return self.timeout_nodes
        raise Exception('VP Error: there is no node space in state "%s"' % prev_state)


def parse_arg_as_num(argument):
    # Protect some tokens
    temp = argument.replace('and', '').replace(' hundred', '-hundred')\
                   .replace(' thousand', '-thousand').replace(' million', '-million')\
                   .replace(' billion', '-billion').replace(' trillion', '-trillion')
    for m in re.findall(r'(\w+ty) (\w+)', temp):
        if 'ty' not in m[1]:
            temp = temp.replace('%s %s' % (m[0], m[1]), '%s-%s' % (m[0], m[1]))

    tokens = re.split(r'comma|,', temp)
    values = []

    # Parse each chunk as a number
    for chunk in tokens:
        # Two consecutive commas -> interpet as zero
        if not chunk:
            values.append(0)
            continue

        # Parse each side of point
        if 'point' in chunk:
            split = chunk.split('point')
            first = split[0]
            second = split[1]
        else:
            first = chunk
            second= 'zero'


        # Now interpet all the values
        first_val  = [ns.parse(word) for word in first.split() ]
        second_val = [ns.parse(word) for word in second.split()]
        
        # Now figure out the value
        total = 0
        for i,f in enumerate(first_val):
            exp = get_num_digits(f)
            total = total * 10 ** exp + f

        dpoint = 0
        for i,s in enumerate(second_val):
            exp = get_num_digits(s)
            dpoint += exp
            total = total + 10 ** (-(dpoint))*s
        

        values.append(total)

    return values

def parse_arg_as_seconds(argument):
    val = argument.replace('and', '')
    vals = re.split('comma|,', val)

    ans = []
    for v in vals:
        if 'second' in argument:
            mult = 1
            v = v.replace('seconds', '').replace('second', '')
        if 'minute' in argument:
            mult = 60
            v = v.replace('minutes', '').replace('minute', '')
        if 'hour' in argument:
            v = v.replace('hour', '').replace('hours', '')
            mult = 3600
        ans.append(parse_arg_as_num(v)[0])
    return sum(ans)


def get_num_digits(num):
    if num == 0:
        return 1
    elif num < 1:
        raise Exception('Error: Function does not work for decimal numbers or negative number (num=%s'%num)
    return math.floor(math.log(num)/math.log(10)) + 1


if __name__=="__main__":
    vp = voice_programmer() 
    print voice_programmer.keywords['all']
    vp.input_buffer(data.lower())
    print vp.get_model_as_dict()


