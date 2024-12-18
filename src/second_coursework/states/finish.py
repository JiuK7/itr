#!/usr/bin/env python3

import rospy
import smach
from geometry_msgs.msg import Pose
from second_coursework.msg import PatrolActionResult  # Update with your actual action package and result msg

def finish_callback(userdata, request):
    """
    Callback for FINISH state.

    This function:
    - Gathers the final list of detected people, cats, and dogs from userdata.
    - Prepares a PatrolActionResult message.
    - Calls set_succeeded(result) on the action server to signal completion.
    """
    rospy.loginfo("[FINISH] Preparing final result...")

    result = PatrolActionResult()
    # Assuming these arrays are geometry_msgs/Pose[] as defined in your action
    result.people_positions = userdata.detected_people
    result.cats_positions = userdata.detected_cats
    result.dogs_positions = userdata.detected_dogs

    # Set the action as succeeded
    if hasattr(userdata, 'set_succeeded') and callable(userdata.set_succeeded):
        userdata.set_succeeded(result)
        rospy.loginfo("[FINISH] Action succeeded with final counts: People=%d, Cats=%d, Dogs=%d",
                      len(userdata.detected_people),
                      len(userdata.detected_cats),
                      len(userdata.detected_dogs))
    else:
        rospy.logwarn("[FINISH] No set_succeeded callable found in userdata. Cannot set action result here.")

    return 'done'

def main():
    rospy.init_node('finish_state_test')

    # For testing, we simulate known detected entities
    sm = smach.StateMachine(outcomes=['finished'])
    sm.userdata.detected_people = [Pose(), Pose()]  # Example: 2 people
    sm.userdata.detected_cats = []
    sm.userdata.detected_dogs = [Pose()]  # Example: 1 dog

    # Here we mock a function for set_succeeded
    def mock_set_succeeded(result):
        rospy.loginfo("[TEST MOCK] Action succeeded called with: %d people, %d cats, %d dogs",
                      len(result.people_positions),
                      len(result.cats_positions),
                      len(result.dogs_positions))

    sm.userdata.set_succeeded = mock_set_succeeded

    with sm:
        smach.StateMachine.add('FINISH',
            smach.CBState(finish_callback,
                          outcomes=['done'],
                          input_keys=['detected_people','detected_cats','detected_dogs','set_succeeded']),
            transitions={'done':'finished'})

    outcome = sm.execute()
    rospy.loginfo("State Machine finished with outcome: %s" % outcome)

if __name__ == '__main__':
    main()

