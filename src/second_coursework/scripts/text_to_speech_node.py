#!/usr/bin/env python3

import rospy
import pyttsx3
from second_coursework.srv import Speak, SpeakResponse


def handle_say_text(req):
    text = req.text
    rospy.loginfo("[TTS] Received text to speak: %s", text)

    try:
        engine = pyttsx3.init()

        # Optional: Configure voice properties
        # engine.setProperty('rate', 150)    # Speech rate
        # engine.setProperty('volume', 0.9)  # Volume (0.0 to 1.0)

        engine.say(text)
        engine.runAndWait()

        rospy.loginfo("[TTS] Successfully spoke the text.")
        return SpeakResponse(success=True)

    except Exception as e:
        rospy.logerr("[TTS] Failed to speak text: %s", str(e))
        return SpeakResponse(success=False)


def text_to_speech_server():
    rospy.init_node('text_to_speech_server', anonymous=True)
    rospy.Service('/text_to_speech', Speak, handle_say_text)
    rospy.loginfo("[TEXT_TO_SPEECH_SERVER] Service /text_to_speech is ready.")
    rospy.spin()


if __name__ == "__main__":
    text_to_speech_server()

