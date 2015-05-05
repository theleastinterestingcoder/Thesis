'''
    middle_manager_coordinator.py

    written by Quan Zhou on May 5th, 2015

    A module for managing resources via primitive actions
'''

class coordinator():
    def __init__(self, cc):
        self.name = 'aux_manager'
        self.core_component = cc

    # handles the cancel command
    def cancel(self):
        cc.cancel()
        return True

    # Stops the base
    def stop_motion(self):
        self.core_component.rvc.stop()
        return True
    
    # Cleans up all the resources
    def reset(self):
        self.stop_motion()
        return True
