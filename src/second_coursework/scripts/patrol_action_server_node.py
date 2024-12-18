#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
import smach
import smach_ros
import actionlib

# Import the action definitions
from second_coursework.msg import (
    PatrolActionAction,
    PatrolActionGoal,
    PatrolActionResult,
    PatrolActionFeedback,
    Emergency,
)

# Import your states (assuming they are in the same package or in the PYTHONPATH)
# Make sure the file names and the callback symbols match what you defined previously.
from states.initialization import init_callback
from states.select_next_room import select_next_room_callback
from states.detect_objects import detect_objects_callback
from states.report_and_feedback import report_and_feedback_callback
# For SimpleActionStates, we just imported the code. Ensure navigate_room_state.py and return_to_e_state.py are accessible.
# The navigate and return states were SimpleActionStates; we can define them inline here or just import their goal_cb if needed.
# Assuming goal_callbacks defined as `goal_callback` in navigate_room_state.py and return_to_e_state.py:
from states.navigate_room import nav_goal_cb as navigate_goal_cb
from states.return_to_e import goal_callback as return_goal_cb
from states.finish import finish_callback

# Messages
from geometry_msgs.msg import PoseStamped

# MoveBaseAction is needed for navigation SimpleActionStates
from move_base_msgs.msg import MoveBaseAction

def main():
    rospy.init_node('patrol_action_server_node')

    # Load parameters from the parameter server
    # Example parameters:
    # - /patrol/waypoints: list of dictionaries [{'x':..., 'y':...}, ...]
    # - /patrol/patrol_duration: float (seconds)
    # - /patrol/e_pose: dictionary {'x':..., 'y':..., 'w':...} for the return-to-E pose
    waypoints_param = rospy.get_param('/patrol/waypoints', [{'x': 1.0, 'y': 2.0}, {'x': 2.0, 'y': 3.0}])
    patrol_duration_default = rospy.get_param('/patrol/patrol_duration', 60.0)
    e_pose_param = rospy.get_param('/patrol/e_pose', {'x':0.0, 'y':0.0, 'w':1.0})

    # Convert waypoints to PoseStamped
    waypoints = []
    for wp in waypoints_param:
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose.position.x = wp['x']
        ps.pose.position.y = wp['y']
        ps.pose.orientation.w = 1.0
        waypoints.append(ps)

    # Convert e_pose to PoseStamped
    e_pose = PoseStamped()
    e_pose.header.frame_id = "map"
    e_pose.pose.position.x = e_pose_param['x']
    e_pose.pose.position.y = e_pose_param['y']
    e_pose.pose.orientation.w = e_pose_param['w']

    # Create a top-level state machine
    sm = smach.StateMachine(outcomes=['ACTION_SUCCEEDED','ACTION_PREEMPTED','ACTION_ABORTED'],
                            input_keys=['patrol_time'],  # from action goal
                            output_keys=[])

    # Initialize userdata fields
    sm.userdata.detected_people = []
    sm.userdata.detected_cats = []
    sm.userdata.detected_dogs = []

    sm.userdata.current_detected_people = []
    sm.userdata.current_detected_cats = []
    sm.userdata.current_detected_dogs = []

    sm.userdata.start_time = None
    sm.userdata.patrol_duration = patrol_duration_default  # Will be overwritten by goal if provided
    sm.userdata.waypoints = waypoints
    sm.userdata.waypoints_index = 0
    sm.userdata.target_pose = None
    sm.userdata.e_pose = e_pose

    # The ActionServerWrapper will pass `goal` (PatrolGoal) into the SM via userdata.
    # We'll handle that in INIT to set patrol_duration based on the goal.

    with sm:
        # INIT
        smach.StateMachine.add('INIT',
                               smach.CBState(init_callback,
                                             outcomes=['done'],
                                             output_keys=['detected_people','detected_cats','detected_dogs','start_time','patrol_duration'],
                                             input_keys=['patrol_time','patrol_duration']),
                               transitions={'done':'SELECT_NEXT_ROOM'})

        # SELECT_NEXT_ROOM
        smach.StateMachine.add('SELECT_NEXT_ROOM',
                               smach.CBState(select_next_room_callback,
                                             outcomes=['next_room','time_up'],
                                             input_keys=['start_time','patrol_duration','waypoints','waypoints_index'],
                                             output_keys=['target_pose','waypoints_index']),
                               transitions={'next_room':'NAVIGATE_ROOM',
                                            'time_up':'RETURN_TO_E'})

        # NAVIGATE_ROOM (SimpleActionState)
        smach.StateMachine.add('NAVIGATE_ROOM',
                               smach_ros.SimpleActionState('/move_base', MoveBaseAction,
                                                           goal_cb=navigate_goal_cb,
                                                           input_keys=['target_pose']),
                               transitions={'succeeded':'DETECT_OBJECTS',
                                            'aborted':'SELECT_NEXT_ROOM',  # retry or handle differently
                                            'preempted':'ACTION_PREEMPTED'})

        # DETECT_OBJECTS
        smach.StateMachine.add('DETECT_OBJECTS',
                               smach.CBState(detect_objects_callback,
                                             outcomes=['done'],
                                             output_keys=['current_detected_people','current_detected_cats','current_detected_dogs']),
                               transitions={'done':'REPORT_AND_FEEDBACK'})

        # REPORT_AND_FEEDBACK
        smach.StateMachine.add('REPORT_AND_FEEDBACK',
                               smach.CBState(report_and_feedback_callback,
                                             outcomes=['done'],
                                             input_keys=['detected_people','detected_cats','detected_dogs',
                                                         'current_detected_people','current_detected_cats','current_detected_dogs',
                                                         'publish_feedback'],
                                             output_keys=['detected_people','detected_cats','detected_dogs',
                                                          'current_detected_people','current_detected_cats','current_detected_dogs']),
                               transitions={'done':'SELECT_NEXT_ROOM'})

        # RETURN_TO_E (SimpleActionState)
        smach.StateMachine.add('RETURN_TO_E',
                               smach_ros.SimpleActionState('/move_base', MoveBaseAction,
                                                           goal_cb=return_goal_cb,
                                                           input_keys=['e_pose']),
                               transitions={'succeeded':'FINISH',
                                            'aborted':'ACTION_ABORTED',
                                            'preempted':'ACTION_PREEMPTED'})

        # FINISH
        smach.StateMachine.add('FINISH',
                               smach.CBState(finish_callback,
                                             outcomes=['done'],
                                             input_keys=['detected_people','detected_cats','detected_dogs','set_succeeded']),
                               transitions={'done':'ACTION_SUCCEEDED'})

    # Wrap the state machine in an ActionServerWrapper to expose it as a PatrolActionAction
    # The ActionServerWrapper will:
    # - Convert action goals into userdata (patrol_time)
    # - Provide set_succeeded, publish_feedback callbacks through userdata
    asw = smach_ros.ActionServerWrapper(
        server_name='patrol_action_server',
        action_spec=PatrolActionAction,
        wrapped_container=sm,
        succeeded_outcomes=['ACTION_SUCCEEDED'],
        aborted_outcomes=['ACTION_ABORTED'],
        preempted_outcomes=['ACTION_PREEMPTED'],
        goal_key='patrol_time',  # The goal has a field 'patrol_time'
        result_key=None,  # We provide the result in FINISH state via set_succeeded
    )

    # Add the ability for states to publish feedback and set result
    # By default, ActionServerWrapper provides set_succeeded and set_preempted methods
    # We can store them in userdata at runtime. For SMACH, we can do:
    # When the action starts, these functions will be available.
    # Using ActionServerWrapper > 0.1.0, publish_feedback is accessible in userdata.
    # If not, we can wrap it:
    def set_succeeded_cb(ud, result):
        asw.set_succeeded(result)
    def publish_feedback_cb(ud, feedback):
        asw.publish_feedback(feedback)

    # Inject these into the top-level sm userdata (optional approach)
    sm.userdata.set_succeeded = set_succeeded_cb
    sm.userdata.publish_feedback = publish_feedback_cb

    # Start the action server
    asw.run_server()

    rospy.loginfo("Patrol action server is running and ready.")
    rospy.spin()

if __name__ == '__main__':
    main()
