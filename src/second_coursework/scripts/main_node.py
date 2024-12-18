#!/usr/bin/env python3

import rospy
import actionlib
from second_coursework.msg import (
    PatrolActionAction,
    PatrolActionGoal,
    PatrolActionResult,
    PatrolActionFeedback,
    Emergency,
)


from geometry_msgs.msg import Pose

class MainNode:
    def __init__(self):
        # Subscribe to the /emergency topic to receive emergency messages
        self.emergency_sub = rospy.Subscriber('/emergency', Emergency, self.emergency_callback, queue_size=1)

        # Create an action client for the PatrolActionAction server
        # Ensure that "patrol_action_server" matches the name used in actionlib_node.py
        self.client = actionlib.SimpleActionClient('patrol_action_server', PatrolActionAction)

        rospy.loginfo("Main Node initialized. Waiting for /emergency messages...")

    def emergency_callback(self, msg):
        # Extract information from the emergency message
        emergency_description = msg.description
        patrol_time = msg.patrol_time
        rospy.loginfo("Emergency received: '%s'. Patrol duration: %.1f seconds" % (emergency_description, patrol_time.to_sec()))

        # Once an emergency is received, run the action
        self.run_action(patrol_time)

    def run_action(self, patrol_time):
        rospy.loginfo("Waiting for the patrol action server to start...")
        self.client.wait_for_server()  # Wait until the action server is up

        # Create a goal to send to the action server
        goal = PatrolActionGoal()
        goal.patrol_time = patrol_time

        rospy.loginfo("Sending goal to the patrol action server...")
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)

        # Wait for the action result, allowing some extra time beyond patrol_time
        timeout = patrol_time.to_sec() + 30.0
        finished_before_timeout = self.client.wait_for_result(rospy.Duration(timeout))

        if finished_before_timeout:
            result = self.client.get_result()
            self.process_result(result)
        else:
            rospy.logwarn("Action did not finish before the %.1f-second timeout." % timeout)

    def feedback_callback(self, feedback: PatrolActionFeedback):
        # Throttle feedback prints to once every 5 seconds to avoid spam
        rospy.loginfo_throttle(5,
            f"[Feedback] People: {feedback.people_found_since_last}, "
            f"Cats: {feedback.cats_found_since_last}, "
            f"Dogs: {feedback.dogs_found_since_last}, "
            f"Last detected position: (x={feedback.last_detection_position.position.x:.2f}, "
            f"y={feedback.last_detection_position.position.y:.2f})"
        )

    def process_result(self, result: PatrolActionResult):
        # Process the final results from the action server
        people_count = len(result.people_positions)
        cats_count = len(result.cat_positions)
        dogs_count = len(result.dog_positions)

        rospy.loginfo("---------- Patrol Complete ----------")
        rospy.loginfo(f"Total People Found: {people_count}")
        for i, pose in enumerate(result.people_positions):
            rospy.loginfo(f"Person {i+1}: (x={pose.position.x:.2f}, y={pose.position.y:.2f})")

        rospy.loginfo(f"Total Cats Found: {cats_count}")
        for i, pose in enumerate(result.cat_positions):
            rospy.loginfo(f"Cat {i+1}: (x={pose.position.x:.2f}, y={pose.position.y:.2f})")

        rospy.loginfo(f"Total Dogs Found: {dogs_count}")
        for i, pose in enumerate(result.dog_positions):
            rospy.loginfo(f"Dog {i+1}: (x={pose.position.x:.2f}, y={pose.position.y:.2f})")

        rospy.loginfo("-------------------------------------")


def main():
    rospy.init_node('main_node', anonymous=False)
    MainNode()
    rospy.spin()

if __name__ == '__main__':
    main()
