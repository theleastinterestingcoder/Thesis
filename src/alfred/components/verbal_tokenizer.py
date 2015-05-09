#!/usr/bin/env python
'''
    verbal_tokenizer.py

    written by Quan Zhou

    breaks text into tuplets of (modifier, keyword)
'''

import re, pdb
from semantic import numbers
ns = numbers.NumberService()

class verbal_tokenizer():
    time_words = {
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
    def __init__(self, core_component):
        self.core_component = core_component
        
        # Update keywords from core_component
        self.keywords = []
        for module, dic in core_component.keyword_to_node.iteritems():
            self.keywords.extend(dic.keys())


        self.modifiers = []
        # use a non-greedy capture group 
        for_regex = r'\bfor\s+(?:.+?)(' + '|'.join(verbal_tokenizer.time_words.keys()) + r')\b'
        self.modifiers.append("then " + for_regex) 
        self.modifiers.append("now " + for_regex) 
        self.modifiers.append(for_regex)
        self.modifiers.extend(["now", "then"])
    # def __init__(self):
    #     self.keywords =  [
    #      r'cancel',
    #      r'stop',
    #      r'move fowards',
    #      r'move foward',
    #      r'move backwards',
    #      r'move backward',
    #      r'move right',
    #      r'turn left',
    #      r'turn right',
    #      r'stop',
    #      r'go to home',
    #      r'go to alpha',
    #      r'go to beta',
    #      r'go home',
    #      r'fail beep',
    #      r'success beep',
    #      r'do nothing',
    #      r'look for']

    #     verbal_tokenizer.time_words = {
    #         'milliseconds' : 0.001,
    #         'millisecond'  : 0.001,
    #         'seconds'      : 1,
    #         'second'       : 1,
    #         'minutes'      : 60,
    #         'minute'       : 60,
    #         'hours'        : 60*60,
    #         'hour'         : 60*60,
    #         'days'         : 60*60*24,
    #         'day'          : 60*60*24,
    #         'weeks'        : 60*60*24*7,
    #         'week'         : 60*60*24*7,   
    #     }

    #     self.modifiers = ['now', 'then']
    #     for_regex = r'\bfor\s+.+(' + '|'.join(verbal_tokenizer.time_words.keys()) + r')\b'
        self.modifiers.append(for_regex)

    def tokenize_string(self, string):
        ans_t = []
        ans_c = []
        all_tokens     = self.keywords + self.modifiers
        tokenized      = self.tokenize_by_list(string, all_tokens)
        colored_tokens = self.color_tokens(tokenized)


        print tokenized
        print colored_tokens

        chunk_t = tokenized
        chunk_c = colored_tokens
        
        # While there is a keyword in the tokens, try to tokenize
        while 'keyword' in chunk_c:

            idx = chunk_c.index('keyword')
            # make the chunks here
            if idx > 1:
                raise Exception('Error in token arrangement in the phrase %s' % chunk_t)
            if idx == 0:
                left_c = None
                left_t = None
                right_c = chunk_c[0]
                right_t = chunk_t[0]
            elif idx == 1:
                left_c = chunk_c[0]
                left_t = chunk_t[0]
                right_c = chunk_c[1]
                right_t = chunk_t[1]


            # Check the parities
            if left_c not in ['modifier', None]:
                raise Exception('Error: %s is not a modifier that precedes keyword (%s) ' % (left_c, right_c))

            ans_t.append((left_t, right_t))
            ans_c.append((left_c, right_c))

            chunk_c = chunk_c[idx+1:]
            chunk_t = chunk_t[idx+1:]

        if chunk_c or chunk_t:
            raise Exception('Error: Undigested chunks %s as %s' % (chunk_c, chunk_t))

        return ans_t, ans_c

    def color_tokens(self, tokenized):
        color = []

        for token in tokenized:
            if self.find_keyword_regex_from_token(token, self.keywords):
                color.append('keyword')
            elif self.find_keyword_regex_from_token(token, self.modifiers):
                color.append('modifier')
            else:
                raise Exception('unrecognized token "%s" ' % token)
        # color in the keywords
        return color

    def tokenize_by_list(self, string, list_of_keywords):
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

    def split_one(self, string, reg):
        match = reg.search(string)

        if match:
            return string[:match.start()], string[match.start():match.end()], string[match.end():]
        else:
            return None, None, None

    def find_keyword(self, string, list_of_keywords):
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

    def find_keyword_regex_from_token(self, token, list_of_regex):
        for keyword in list_of_regex:
            reg = re.compile(r'\b' + keyword + r'\b')
            match = reg.search(token)
            if match:
                return keyword 
        return ""


    def parse_numeric(self, group):
        # This function is built only for one grouop
        if not group:
            raise Exception('verbal_tokenizer: expected group but got none')
        return ns.parse(group)
    
    def parse_time(self, group):
        # This function is built only for one grouop
        if not group:
            raise Exception('verbal_tokenizer: expected group but got none')

        # Now parse the time
        flag = True
        for key, val in verbal_tokenizer.time_words.iteritems():
            if key in group:
                flag=False
                break
        if flag:
            raise Exception("Parsing Error: no expression of time found in '%s'" % string)

        return ns.parse(group.replace(key, '').strip()) * val

    def parse_string(self, group):
        # This function is built only for one grouop
        if not group:
            raise Exception('verbal_tokenizer: expected group but got none')

        return group.strip()

    def parse_tuple(self, group):
        if not group:
            raise Exception('verbal_tokenizer: expected group but got none')
        return tuple([g.strip() for g in group])

# if __name__=='__main__':
#     vt = verbal_tokenizer()
#     # text='go home now look for Quan'
#     # text='go home for ten seconds go home'
#     text = 'go to alpha then look for quan then go home'
#     ans = vt.tokenize_string(text)
#     print ans[0]
#     print ans[1]
