#!/usr/bin/env python3

import rospy
import smach
import time
from geometry_msgs.msg import Pose
from your_custom_package.srv import YoloDetect, YoloDetectRequest  # Update with your actual package and srv name


def detect_objects_callback(userdata, request):
    """
    Callback for DETECT_OBJECTS state.
    This function:
    - Calls a YOLO detection service to get currently visible people, cats, and dogs.
    - Stores the results in userdata so that the next state can compare them with previously detected entities.
    """

    rospy.loginfo("[DETECT_OBJECTS] Calling YOLO detection service...")

    # Wait for the YOLO detection service to be available
    rospy.wait_for_service('/yolo_detect')

    try:
        detect_service = rospy.ServiceProxy('/yolo_detect', YoloDetect)
        response = detect_service()  # No request fields assumed, adjust if necessary

        # Store results in userdata
        userdata.current_detected_people = response.people_positions
        userdata.current_detected_cats = response.cats_positions
        userdata.current_detected_dogs = response.dogs_positions

        rospy.loginfo("[DETECT_OBJECTS] YOLO detection complete. Found: %d people, %d cats, %d dogs",
                      len(response.people_positions),
                      len(response.cats_positions),
                      len(response.dogs_positions))

        return 'done'
    except rospy.ServiceException as e:
        rospy.logerr("[DETECT_OBJECTS] YOLO detection service call failed: %s" % e)
        # If the service call fails, you might decide to retry or set a different outcome.
        return 'done'  # Assuming we continue anyway; adjust if needed.


def main():
    rospy.init_node('detect_objects_state_test')

    # For testing, we'll just run this state in isolation
    sm = smach.StateMachine(outcomes=['finished'])
    sm.userdata.current_detected_people = []
    sm.userdata.current_detected_cats = []
    sm.userdata.current_detected_dogs = []

    with sm:
        smach.StateMachine.add('DETECT_OBJECTS',
                               smach.CBState(detect_objects_callback,
                                             outcomes=['done'],
                                             input_keys=[],
                                             output_keys=['current_detected_people',
                                                          'current_detected_cats',
                                                          'current_detected_dogs']),
                               transitions={'done': 'finished'})

    outcome = sm.execute()
    rospy.loginfo("State Machine finished with outcome: %s" % outcome)
    rospy.loginfo("Detected people: %s" % sm.userdata.current_detected_people)
    rospy.loginfo("Detected cats: %s" % sm.userdata.current_detected_cats)
    rospy.loginfo("Detected dogs: %s" % sm.userdata.current_detected_dogs)


if __name__ == '__main__':
    main()
