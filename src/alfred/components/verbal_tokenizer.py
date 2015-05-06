'''
    verbal_tokenizer.py

    written by Quan Zhou

    breaks text into tuplets of (modifier, statement, argument)
'''

class verbal_tokenizer():
    def __init__(core_component):
        self.core_component = core_component
        
        # Update keywords from core_component
        self.keywords = []
        for module, dic in core_component.keyword_to_node:
            self.keywords.extend(dic.keys())

        self.time_words = {
            'milliseconds' : 0.001,
            'millisecond'  : 0.001,
            'seconds'      : 1,
            'second'       : 1,
            'minutes'      : 60,
            'minute'       : 60,
            'hours'        : 60*60,
            'hour'         : 60*60,
            'days'         : 60*60*24,
            'day'          : 60*60*24,
            'weeks'        : 60*60*24*7,
            'week'         : 60*60*24*7,   
        }

        self.modifier = ['now', 'then', 'for']
        for_regex = r'\bfor\s+.+(' + '|'.join(self.time_words.keys()) + r')\b'
        self.modifier.append(for_regex)

    def tokenize_string(self, string):
        ans_t = []
        ans_c = []
        all_tokens     = self.keywords + self.modifier
        tokenized      = self.tokenize_by_list(string, all_tokens)
        colored_tokens = self.color_tokens(tokens, string):

        chunk_t = tokenized
        chunk_c = colored_tokens
        
        # While there is a keyword in the tokens, try to tokenize
        while 'keyword' in chunk_c:
            idx = chunk_c.index('keyword')

            left_cs   = chunk_c[:idx]
            left_ts   = chunk_t[:idx]
            middle_cs = chunk_c[idx]  
            middle_ts = chunk_t[idx]
            right_cs  = chunk_c[idx:]
            right_ts  = chunk_t[:idx]

            # Check parity
            if len(left_c) * len(middle_c) * len(right_c) != 1:
                raise Exception('Error: tuplet (left, middle, right) = (%s, %s, %s) has multiple tokens' % (left_t, middle_t, right_c))
            
            # Dereference
            left_c  = left_cs[0]
            left_t  = left_ts[0]
            middle_c  = middle_cs[0]
            middle_t  = middle_ts[0]
            right_c  = right_cs[0]
            right_t  = right_ts[0]

            # Look at the right token and adjust chunking sie
            if right_c != 'argument':
                chunk_c = chunk_c[idx+1:]
                chunk_t = chunk_t[idx+1:]
                right_t = None
                right_c = None
            else:
                chunk_c = chunk_c[idx+2:]
                chunk_t = chunk_t[idx+2:]

            # Check the parities
            if left_c != 'modifier':
                raise Exception('Error: %s is not a modifier that precedes keyword (%s) ' % (left_c, middle_c))

            ans_t.append((left_t, middle_t, right_t))
            ans_c.append((left_c, middle_c, right_c))

        if chunk_c or chunk_t:
            raise Exception('Error: Undigested chunks %s as %s' % (chunk_c, chunk_t))

        return ans_t, ans_c

    def parse_command(self, string):
        # Convert a string into a command
        for tyype, mapping in self.keyword_to_node.iteritems():
            for keyword in mapping.keys():
                if keyword in data.data:
                    return tyype, keyword
        rospy.loginfo('Warning: command not recognized "%s"' % data)
        return None, None
                
    def parse_modifier(data):
        ans = {'do_now' : True}
        chunk = data


        # Check for 'now' and 'then'
        if re.search(r'\bnow\b', chunk):
            chunk = re.sub(r'\bnow\b', '', chunk)
        if re.search(r'\bthen\b', chunk):
            ans['do_now'] = False
            chunk = re.sub(r'\bthen\b', '', chunk)
        # "For ____ <time>" signals a timeout
        if 'for' in chunk:
            flag = True
            for key, val in self.time_words.iteritems():
                if key in data:
                    flag=False
                    break
            if flag:
                raise Exception("Parsing Error: Keyword 'for' is not followed by a expression of time")

            chunk = re.sub(r'\bfor\b', '', chunk)
            idx = chunk.index(key)
            chunk = chunk[:idx].strip()
        

            print "chunk %s" % chunk


            ans['timeout'] = {
                'time' : ns.parseInt(chunk) * val,
                'mission' : {
                    'name' : data,
                    'start_node': "fail_beep", # IMPORTANT FIX.
                    'do_now':  True,
                }
            }

        return ans

    def color_tokens(tokens, data):
        color = []
        for token in tokenized:
            if token in self.keywords:
                color.append('keyword')
            if token  in self.keywords:
                color.append('modifier')
            else:
                color.append('argument')
        # color in the keywords
        return color

    def tokenize_by_list(string, list_of_keywords):
        ans = []
        chunk = string

        keyword = self.find_keyword(chunk, list_of_keywords)

        while keyword != "":
            reg = re.compile(r'\b' + keyword + r'\b')
            # Split it into three parts
            left, mid, right = self.split_one(chunk, reg)

            # If we have a match, then add it to our list
            if mid:
                ans.append(left)
                ans.append(mid)
                chunk = right

            # prime the next loop 
            keyword = self.find_keyword(chunk, list_of_keywords)
        ans.append(chunk)

        return [a.strip() for a in ans if a.strip()]

    def split_one(string, reg):
        match = reg.search(string)

        if match:
            return string[:match.start()], string[match.start():match.end()], string[match.end():]
        else:
            return None, None, None

    def find_keyword(string, list_of_keywords):
        pos = []

        for keyword in list_of_keywords:
            reg = re.compile(r'\b' + keyword + r'\b')
            match = reg.search(string)
            if match:
                pos.append((match.start(), match.end()))

        if not pos:
            return ""
        idx = sorted(pos, key=lambda x: x[0])[0]
        return string[idx[0]:idx[1]]

            
    
