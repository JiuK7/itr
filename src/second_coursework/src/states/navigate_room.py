#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

# Dictionary of subpoints per room
room_subpoints = {
    'A': [(0.5, 9.5), (3.5, 9.5), (0.5, 7.5), (3.5, 7.5), (2.0, 8.5)],  # Room A
    'B': [(4.5, 9.5), (7.5, 9.5), (4.5, 7.5), (7.5, 7.5), (6.0, 8.5)],  # Room B
    'C': [(8.5, 9.5), (11.5, 9.5), (8.5, 7.5), (11.5, 7.5), (10.0, 8.5)],  # Room C
    'D': [(0.5, 2.5), (3.5, 2.5), (0.5, 0.5), (3.5, 0.5), (2.0, 1.5)],  # Room D
    'E': [(4.5, 2.5), (7.5, 2.5), (4.5, 0.5), (7.5, 0.5), (6.0, 1.5)],  # Room E
    'F': [(8.5, 2.5), (11.5, 2.5), (8.5, 0.5), (11.5, 0.5), (10.0, 1.5)],  # Room F
}


def nav_goal_cb(userdata, goal):
    """Creates a MoveBaseGoal from userdata.x and userdata.y."""
    move_goal = MoveBaseGoal()
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = userdata.x
    pose.pose.position.y = userdata.y
    pose.pose.orientation.w = 1.0  # Face forward
    move_goal.target_pose = pose
    return move_goal


def create_room_submachine(room_points):
    """
    Create a sub-state-machine that visits all waypoints in `room_points` sequentially.
    room_points: list of (x,y) tuples
    """
    sm_room = smach.StateMachine(outcomes=['finished', 'failed'])
    sm_room.userdata.waypoints = room_points

    with sm_room:
        # Dynamically add states for each waypoint
        for i, (x, y) in enumerate(room_points):
            state_name = f'NAV_TO_POINT_{i}'

            # Add the navigation state
            smach.StateMachine.add(
                state_name,
                smach_ros.SimpleActionState('/move_base',
                                            MoveBaseAction,
                                            goal_cb=nav_goal_cb,
                                            input_keys=['x', 'y']),
                transitions={'succeeded': f'NAV_TO_POINT_{i + 1}' if i + 1 < len(room_points) else 'finished',
                             'aborted': 'failed',
                             'preempted': 'failed'}
            )

            # Set the userdata for this state (x,y) before execution
            sm_room.set_initial_state([state_name])
            sm_room.userdata.x = x
            sm_room.userdata.y = y

    return sm_room


def main():
    rospy.init_node('navigate_all_rooms')

    # Top-level state machine to go through rooms A-F sequentially
    sm_top = smach.StateMachine(outcomes=['all_rooms_visited', 'failed'])

    # Define the order of rooms to visit
    room_order = ['A', 'B', 'C', 'D', 'E', 'F']

    with sm_top:
        # Create sub-state-machines for each room and add them in sequence
        prev_state = None
        for i, room_id in enumerate(room_order):
            state_name = f'VISIT_ROOM_{room_id}'
            sm_room = create_room_submachine(room_subpoints[room_id])

            # If it's the first room, add directly
            if i == 0:
                smach.StateMachine.add(
                    state_name,
                    sm_room,
                    transitions={'finished': f'VISIT_ROOM_{room_order[i + 1]}' if i + 1 < len(
                        room_order) else 'all_rooms_visited',
                                 'failed': 'failed'}
                )
            else:
                # For subsequent rooms, chain from the previous
                smach.StateMachine.add(
                    state_name,
                    sm_room,
                    transitions={'finished': f'VISIT_ROOM_{room_order[i + 1]}' if i + 1 < len(
                        room_order) else 'all_rooms_visited',
                                 'failed': 'failed'}
                )

            prev_state = state_name

    outcome = sm_top.execute()
    rospy.loginfo("Top-level State Machine finished with outcome: %s" % outcome)


if __name__ == '__main__':
    main()
