#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

def goal_callback(userdata, goal):
    """
    Callback to transform userdata.e_pose (PoseStamped)
    into a MoveBaseGoal for navigation.
    """
    move_goal = MoveBaseGoal()
    move_goal.target_pose = userdata.e_pose
    return move_goal

def main():
    rospy.init_node('return_to_e_state_test')

    # Example: We'll create a state machine that just runs this state.
    sm = smach.StateMachine(outcomes=['finished','failed'])
    # Provide a test pose for Room E
    sm.userdata.e_pose = PoseStamped()
    sm.userdata.e_pose.header.frame_id = "map"
    sm.userdata.e_pose.pose.position.x = 0.0
    sm.userdata.e_pose.pose.position.y = 0.0
    sm.userdata.e_pose.pose.orientation.w = 1.0

    with sm:
        # Add the RETURN_TO_E state
        smach.StateMachine.add('RETURN_TO_E',
            smach_ros.SimpleActionState('/move_base', MoveBaseAction,
                                        goal_cb=goal_callback,
                                        input_keys=['e_pose']),
            transitions={'succeeded':'finished',
                         'aborted':'failed',
                         'preempted':'failed'})

    outcome = sm.execute()
    rospy.loginfo("State Machine finished with outcome: %s" % outcome)

if __name__ == '__main__':
    main()
