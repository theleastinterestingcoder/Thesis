'''
    text_cacher.py

    Written by Quan Zhou on May 9th 2015

    caches text that has been sent to it
'''
import rospy

class text_cacher():
    def __init__(self):
        self.cache_s = []
        self.is_active = False

    def update(self, string):
        if not self.is_active:
            raise Exception("Text Cacher Warning: Need to activate text cacher")
        self.cache_s.append(string)
        self.print_cache()
    
    def get_cache(self):
        return " ".join(self.cache_s)

    def print_cache(self):
        rospy.loginfo('  Current string in cache: "%s"' % self.get_cache())

    def undo_insert(self):
        if not self.cache_s:
            rospy.loginfo("No more undo's left")
        else:
            self.cache_s.pop(-1)
            self.print_cache()

    def activate(self):
        self.is_active = True
        self.clear()
    
    def clear(self):
        self.cache_s = []
        self.print_cache()

    def deactivate(self):
        self.is_active = False
