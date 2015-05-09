'''
    text_cacher.py

    Written by Quan Zhou on May 9th 2015

    caches text that has been sent to it
'''
import rospy

class text_cacher():
    def __init__(self):
        self.cache_s = ""
        self.is_active = False

    def update(self, string):
        if not self.is_active:
            raise Exception("Text Cacher Warning: Need to activate text cacher")
        self.cache_s += " " + string
        self.cache_s = self.cache_s.replace('  ', '')

        rospy.loginfo('  Current strings in cache: "%s"' % self.cache_s)
    
    def get_cache(self):
        return self.cache_s

    def print_cache(self):
        rospy.loginfo('  Current string in cache: "%s"' % self.get_cache())

    def activate(self):
        self.is_active = True
        self.clear()
    
    def clear(self):
        self.cache_s = ""
        rospy.loginfo('  Current strings in cache: ""')

    def deactivate(self):
        self.is_active = False
