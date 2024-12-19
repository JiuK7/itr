#!/usr/bin/env python3

import rospy
import smach
import time


def init_callback(userdata, request):
    """
    Callback function for INIT state.
    This function will:
    - Initialize lists to store detected people, cats, and dogs.
    - Record the start time of the patrol.
    - Prepare any other initial conditions needed by the state machine.
    """
    # Clear previously detected entities
    userdata.detected_people = []
    userdata.detected_cats = []
    userdata.detected_dogs = []

    # Record the current time as start time
    userdata.start_time = rospy.Time.now()

    # Log info (optional)
    rospy.loginfo("[INIT] State: Initialization complete.")

    # Return the outcome
    return 'done'


def main():
    # This main function is just an example of how you might test this state in isolation.
    # Normally, you would import this state into your main state machine file.
    rospy.init_node('init_state_test')

    # Create a StateMachine for testing
    sm = smach.StateMachine(outcomes=['finished'])
    sm.userdata.detected_people = None
    sm.userdata.detected_cats = None
    sm.userdata.detected_dogs = None
    sm.userdata.start_time = None

    with sm:
        # Add the INIT state as a CBState
        smach.StateMachine.add('INIT',
                               smach.CBState(init_callback,
                                             outcomes=['done'],
                                             input_keys=[],
                                             output_keys=['detected_people',
                                                          'detected_cats',
                                                          'detected_dogs',
                                                          'start_time']),
                               transitions={'done': 'finished'})

    # Execute state machine
    outcome = sm.execute()
    rospy.loginfo("State Machine finished with outcome: %s" % outcome)


if __name__ == '__main__':
    main()
