'''
    eliza_bot.py

    uses eliza.py (a pythonic version of eliza) as a chatbot

    speech_recognition : github: https://github.com/Uberi/speech_recognition

'''
import sys, time
sys.path.append('/home/alfred/quan_ws/foo/eliza/')
sys.path.append('/home/alfred/quan_ws/foo/speech_recognition/')

from eliza import eliza
import speech_recognition as sr
from termcolor import colored

def initiate_doctor():
    # Initiates the doctor
    therapist = eliza()
    print colored("-"* 30, 'green')
    print colored("Hello! I am Eliza, your personal psychoanalyst. Tell me about your problems.", 'green')
    print colored("When you are done, say 'I am satisfied with my care'", 'green'), colored("\n>", "red"),

    # Microphone settings
    r = sr.Recognizer()
    r.energy_threshold = 4000 # quiet room, medium distance, normal volume

    query = ''
    while query != 'I am satisfied with my care':
        # Record and get response
        query = mic_to_text(r, verbose=True)
        if query == "": continue

        resp  = therapist.respond(query)

        # Displayh results
        print colored("you said> %s" % (query), 'red')
        time.sleep(1)    
        print colored("eliza responded> %s" % (resp), 'green')
        print colored(">", "red"),

    print colored('Thank you for the visit.', 'green')
    

# ------ Some microphone stuff here -----
def mic_to_text(r, verbose=False):
    #Translate what is on the audio to text
    with sr.Microphone() as source:                # use the default microphone as the audio source
        #        if verbose: print colored(">Recording Voice...", 'grey'),
        audio = r.listen(source)                   # listen for the first phrase and extract it into audio data
    try:
        if verbose and audio:
            print colored("Sending recording to google", 'grey')
        return r.recognize(audio)                  # recognize speech using Google Speech Recognition
    except LookupError:                            # speech is unintelligible
        if verbose:
            print colored("Speech is unintelligible.", 'grey')
        return ""

if __name__== '__main__':
    initiate_doctor()
