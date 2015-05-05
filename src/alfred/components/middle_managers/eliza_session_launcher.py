'''
    eliza_session_launcher

    uses eliza.py (a pythonic version of eliza) as a chatbot 
    and is integrated with google voice

    speech_recognition : github: https://github.com/Uberi/speech_recognition

'''
import sys, time

import rospy

from eliza import eliza
import speech_recognition as sr
from termcolor import colored

class eliza_session_launcher():
    def __init__(self,core_component):
        self.is_interrupted = False
        
        self.interrupt_sub = rospy.Subscriber('alfred/mission_control', self.handle_mission_control)
    
    # Primitive action: gives a therapist session
    def init_session(self, duration):
        ans = initiate_doctor()
        core_component.pause_speech = True

        if ans:
            rospy.loginfo('ESL: Successful session')
        elif ans == False:
            rospy.loginfo('ESL: Unsuccessful session')
        else:
            rospy.loginfo('ESL: Session canceled')
        
        core_component.pause_speech = False
        return ans
        
    # Handle cancels from the outside 
    def handle_mission_control(self, data):
        if data.data == 'cancel':
            self._is_interrupted = True

    def initiate_doctor(self):
        # Initiates the doctor
        therapist = eliza()
        rospy.loginfo(colored("-"* 30, 'green'))
        rospy.loginfo(colored("Hello! I am Eliza, your personal psychoanalyst. Tell me about your problems.", 'green'))
        rospy.loginfo(colored("When you are done, say 'I am satisfied with my care'", 'green')+colored("\n>", "red"))
        rospy.loginfo(colored("Or, you can say 'exit' to leave this session", 'green'))

        # Microphone settings
        r = sr.Recognizer()
        r.energy_threshold = 4000 # quiet room, medium distance, normal volume
        
        # Loop and give session
        query = ''
        while True:
            # Check for interrupt
            if self.is_interrupt:
                self.is_interrupt
                return None
            # Record and get response
            query = mic_to_text(r, verbose=True)
            if query == "": 
                continue

            # Display  results
            rospy.loginfo(colored("you said> %s" % (query), 'red'))
            
            # Check if patient is satisifed 
            if query.lower() in ['i am satisfied with my care', "i'm satisfied with my care", "satisifed with my care"]:
                return True
            if query.lower() in ['exit']:
                return False

            resp  = therapist.respond(query)
            time.sleep(1)    
            rospy.loginfo(colored("eliza responded> %s" % (resp), 'green'))
            rospy.loginfo(colored(">", "red"))

        rospy.loginfo(colored('Thank you for the visit.', 'green'))
    

# ------ Some microphone stuff here -----
def mic_to_text(r, verbose=False):
    #Translate what is on the audio to text
    with sr.Microphone() as source:                
        audio = r.listen(source)                   
    try:
        if verbose and audio:
            rospy.loginfo(colored("Sending recording to google", 'grey'))
        return r.recognize(audio)                 
    except LookupError:                          
        if verbose:
            rospy.loginfo(colored("Speech is unintelligible.", 'grey'))
        return ""

if __name__== '__main__':
    rospy.init_node("alfred")
    esl = eliza_session_launcher()
    esl.init_session()



