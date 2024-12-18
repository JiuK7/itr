#!/usr/bin/env python3

import rospy
from second_coursework.srv import Speak, SpeakResponse

def handle_say_text(req):
    # In a real implementation, integrate with a TTS engine here.
    # For example:
    #   engine = pyttsx3.init()
    #   engine.say(req.text)
    #   engine.runAndWait()
    #
    # Or call a shell command:
    #   subprocess.call(["espeak", req.text])

    rospy.loginfo("[TTS] Speaking: %s", req.text)
    # Simulate success
    success = True
    return SpeakResponse(success=success)

def text_to_speech_server():
    rospy.init_node('text_to_speech_server', anonymous=True)
    rospy.Service('/text_to_speech', Speak, handle_say_text)
    rospy.loginfo("[TEXT_TO_SPEECH_SERVER] Service /text_to_speech is ready.")
    rospy.spin()

if __name__ == "__main__":
    text_to_speech_server()
