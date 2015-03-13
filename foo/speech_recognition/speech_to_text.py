'''
    Written by Quan Zhou on 3/11

    Uses the package speech_recognition to make connections
      to Google's speech recognition software

    github: https://github.com/Uberi/speech_recognition
'''
from ctypes import *
import contextlib
import sys

# NOTE: this requires PyAudio because it uses the Microphone class
import speech_recognition as sr

# Create the environment variables
def main():
    r = sr.Recognizer()
    r.energy_threshold = 3000 # quiet room, medium distance, normal volume

    print mic_to_text(r, verbose=True)

# Returns a null string if not intelligable 
def mic_to_text(r, verbose=False):
    with sr.Microphone() as source:                # use the default microphone as the audio source
        if verbose: print "Recording Voice...",
        audio = r.listen(source)                   # listen for the first phrase and extract it into audio data
    try:
        if verbose:
            print "sending recording to google"
        return r.recognize(audio)                  # recognize speech using Google Speech Recognition
    except LookupError:                            # speech is unintelligible
        return ""

@contextlib .contextmanager
def nostderr():
    savestderr = sys.stderr
    savestdout = sys.stdout

    class Devnull(object):
        def write(self, _): pass
    sys.stderr = Devnull()
    sys.stdout = Devnull()
    try:
        yield
    finally:
        stderr = savestderr
        stdout = savestdout

if __name__ == "__main__":
    main()
