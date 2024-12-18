#!/usr/bin/env python3

import rospy
import smach
from geometry_msgs.msg import PoseStamped

def select_next_room_callback(userdata, request):
    """
    Callback for SELECT_NEXT_ROOM state.
    - Checks if patrol duration time has elapsed.
    - If not elapsed, selects the next waypoint from userdata.
    - If no more waypoints or time exceeded, returns time_up.
    """

    # Current time
    current_time = rospy.Time.now()
    elapsed_time = (current_time - userdata.start_time).to_sec()

    # Check if we've reached or exceeded the patrol duration
    if elapsed_time >= userdata.patrol_duration:
        rospy.loginfo("[SELECT_NEXT_ROOM] Time is up. No more rooms will be visited.")
        return 'time_up'

    # Check if we have any waypoints left
    if userdata.waypoints_index >= len(userdata.waypoints):
        rospy.loginfo("[SELECT_NEXT_ROOM] No more rooms (waypoints) to visit.")
        return 'time_up'

    # Select the next waypoint
    target_pose = userdata.waypoints[userdata.waypoints_index]
    userdata.waypoints_index += 1

    # Store the target pose for NAVIGATE_ROOM state
    userdata.target_pose = target_pose

    rospy.loginfo("[SELECT_NEXT_ROOM] Next room selected. Waypoint index: %d" % (userdata.waypoints_index-1))
    return 'next_room'


def main():
    rospy.init_node('select_next_room_state_test')

    # Example initialization
    test_waypoints = []
    # Populate with some example waypoints
    # In practice, these would be loaded from a parameter server, config file, etc.
    for i in range(3):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 1.0 * i
        pose.pose.position.y = 2.0 * i
        pose.pose.orientation.w = 1.0
        test_waypoints.append(pose)

    sm = smach.StateMachine(outcomes=['finished', 'time_up'])
    sm.userdata.start_time = rospy.Time.now()
    sm.userdata.patrol_duration = 60.0  # seconds
    sm.userdata.waypoints = test_waypoints
    sm.userdata.waypoints_index = 0
    sm.userdata.target_pose = None

    with sm:
        smach.StateMachine.add('SELECT_NEXT_ROOM',
            smach.CBState(select_next_room_callback,
                          outcomes=['next_room','time_up'],
                          input_keys=['start_time','patrol_duration','waypoints','waypoints_index'],
                          output_keys=['target_pose','waypoints_index']),
            transitions={'next_room':'finished',  # For testing, we just end after one room
                         'time_up':'time_up'})

    outcome = sm.execute()
    rospy.loginfo("State Machine finished with outcome: %s" % outcome)


if __name__ == '__main__':
    main()
