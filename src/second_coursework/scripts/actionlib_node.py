#!/usr/bin/env python3
import rospy
import actionlib
from second_coursework.msg import (
    PatrolActionAction,
    PatrolActionGoal,
    PatrolActionResult,
    PatrolActionFeedback,
)


from geometry_msgs.msg import Pose

class PatrolServer:
    def __init__(self):
        # The server is named "patrol_action". Make sure the client uses the same name.
        self.server = actionlib.SimpleActionServer("patrol_action", PatrolActionAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        rospy.loginfo("Patrol started. Duration: %.1f seconds" % goal.patrol_time.to_sec())

        feedback = PatrolActionFeedback()
        result = PatrolActionResult()

        # Initialize feedback fields if needed
        feedback.people_found_since_last = 0
        feedback.cats_found_since_last = 0
        feedback.dogs_found_since_last = 0
        feedback.last_detection_position = Pose()

        # For demonstration, we can send periodic feedback
        start_time = rospy.Time.now()
        patrol_duration = goal.patrol_time.to_sec()

        # Simulate a patrol by sleeping and sending feedback periodically
        rate = rospy.Rate(1)  # 1 Hz
        while (rospy.Time.now() - start_time).to_sec() < patrol_duration:
            # Update feedback (just fake data for now)
            feedback.people_found_since_last += 1
            feedback.cats_found_since_last += 0
            feedback.dogs_found_since_last += 0
            feedback.last_detection_position.position.x += 0.1  # Just simulating some movement

            # Publish feedback
            self.server.publish_feedback(feedback)

            # Sleep
            rate.sleep()

        # After patrol, set the result (empty arrays in this example)
        result.people_positions = []
        result.cat_positions = []
        result.dog_positions = []

        # If everything is done, set the action as succeeded
        rospy.loginfo("Patrol completed successfully.")
        self.server.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("patrol_server")
    server = PatrolServer()
    rospy.spin()
