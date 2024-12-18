#!/usr/bin/env python3

import rospy
import smach
from geometry_msgs.msg import Pose
from your_custom_package.srv import SayText, SayTextRequest  # Update with your actual package and srv name


def report_and_feedback_callback(userdata, request):
    """
    Callback for REPORT_AND_FEEDBACK state.

    This function:
    - Identifies new detections not seen previously.
    - If new people are found, triggers TTS to announce them.
    - Updates main detected lists with newly found entities.
    - Publishes feedback through the action server (if `userdata.publish_feedback` is available).
    """

    # Find new detections by comparing current lists with previously known lists
    new_people = [p for p in userdata.current_detected_people if p not in userdata.detected_people]
    new_cats = [c for c in userdata.current_detected_cats if c not in userdata.detected_cats]
    new_dogs = [d for d in userdata.current_detected_dogs if d not in userdata.detected_dogs]

    # Announce new people via TTS if any
    if new_people:
        try:
            rospy.wait_for_service('/text_to_speech', timeout=5.0)
            tts_srv = rospy.ServiceProxy('/text_to_speech', SayText)
            # Construct a message - for simplicity, just count how many people
            text_msg = "I see " + str(len(new_people)) + " new person" + ("" if len(new_people) == 1 else "s")
            resp = tts_srv(SayTextRequest(text=text_msg))
            if resp:
                rospy.loginfo("[REPORT_AND_FEEDBACK] TTS announcement made: %s", text_msg)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("[REPORT_AND_FEEDBACK] Failed to call TTS service: %s", e)

    # Update the master lists with newly detected entities
    userdata.detected_people.extend(new_people)
    userdata.detected_cats.extend(new_cats)
    userdata.detected_dogs.extend(new_dogs)

    # If you have a feedback function or action server interface:
    # For example, let's assume userdata.publish_feedback is a callable that takes a dictionary:
    if hasattr(userdata, 'publish_feedback') and callable(userdata.publish_feedback):
        feedback_msg = {
            "new_people_count": len(new_people),
            "new_cats_count": len(new_cats),
            "new_dogs_count": len(new_dogs),
            "total_people_count": len(userdata.detected_people),
            "total_cats_count": len(userdata.detected_cats),
            "total_dogs_count": len(userdata.detected_dogs)
        }
        userdata.publish_feedback(feedback_msg)
        rospy.loginfo("[REPORT_AND_FEEDBACK] Feedback published: %s", feedback_msg)

    # Clear current detection lists after handling
    userdata.current_detected_people = []
    userdata.current_detected_cats = []
    userdata.current_detected_dogs = []

    rospy.loginfo("[REPORT_AND_FEEDBACK] Reporting and feedback complete.")
    return 'done'


def main():
    rospy.init_node('report_and_feedback_state_test')

    sm = smach.StateMachine(outcomes=['finished'])
    # Initialize userdata with some known detections for testing
    sm.userdata.detected_people = []
    sm.userdata.detected_cats = []
    sm.userdata.detected_dogs = []

    # Current detection lists - for testing, let's say we found one new person.
    sm.userdata.current_detected_people = [Pose()]
    sm.userdata.current_detected_cats = []
    sm.userdata.current_detected_dogs = []

    # If you had a feedback function, you'd add it here:
    # sm.userdata.publish_feedback = some_feedback_function

    with sm:
        smach.StateMachine.add('REPORT_AND_FEEDBACK',
                               smach.CBState(report_and_feedback_callback,
                                             outcomes=['done'],
                                             input_keys=['detected_people',
                                                         'detected_cats',
                                                         'detected_dogs',
                                                         'current_detected_people',
                                                         'current_detected_cats',
                                                         'current_detected_dogs'
                                                         # 'publish_feedback' if available
                                                         ],
                                             output_keys=['detected_people',
                                                          'detected_cats',
                                                          'detected_dogs',
                                                          'current_detected_people',
                                                          'current_detected_cats',
                                                          'current_detected_dogs']),
                               transitions={'done': 'finished'})

    outcome = sm.execute()
    rospy.loginfo("State Machine finished with outcome: %s", outcome)
    rospy.loginfo("Updated known detections - People: %d, Cats: %d, Dogs: %d",
                  len(sm.userdata.detected_people),
                  len(sm.userdata.detected_cats),
                  len(sm.userdata.detected_dogs))


if __name__ == '__main__':
    main()
