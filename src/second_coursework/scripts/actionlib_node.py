#!/usr/bin/env python3
import rospy
import actionlib
from second_coursework.msg import (
    PatrolActionAction,
    PatrolActionGoal,
    PatrolActionResult,
    PatrolActionFeedback,
    YoloDetect,
    YoloDetectRequest
)
from geometry_msgs.msg import Pose


class PatrolServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("patrol_action", PatrolActionAction, self.execute, False)

        # Initialize variables for tracking detections
        self.people_since_last = 0
        self.cats_since_last = 0
        self.dogs_since_last = 0
        self.last_detection_position = Pose()

        # Start the server
        self.server.start()

    def execute(self, goal):
        rospy.loginfo("Patrol started. Duration: %.1f seconds" % goal.patrol_time.to_sec())

        # Initialize feedback and result
        feedback = PatrolActionFeedback()
        result = PatrolActionResult()

        # Initialize the start time and loop rate
        start_time = rospy.Time.now()
        patrol_duration = goal.patrol_time.to_sec()
        rate = rospy.Rate(5)  # check more frequently, but not necessarily publish

        # Main loop until the specified patrol duration ends
        while (rospy.Time.now() - start_time).to_sec() < patrol_duration and not rospy.is_shutdown():

            # Check for new detections (in a real system, this might be handled by callbacks)
            new_people, new_cats, new_dogs, new_position = self.check_for_new_detections()

            if new_people > 0 or new_cats > 0 or new_dogs > 0:
                # Update counters and pose
                self.people_since_last += new_people
                self.cats_since_last += new_cats
                self.dogs_since_last += new_dogs
                self.last_detection_position = new_position

                # Update the feedback message
                feedback.people_found_since_last = self.people_since_last
                feedback.cats_found_since_last = self.cats_since_last
                feedback.dogs_found_since_last = self.dogs_since_last
                feedback.last_detection_position = self.last_detection_position

                # Publish feedback only when new detections are found
                self.server.publish_feedback(feedback)

                # Reset counters after publishing feedback so that next time we only report new discoveries
                self.people_since_last = 0
                self.cats_since_last = 0
                self.dogs_since_last = 0

            rate.sleep()

        # After patrol, fill in the result
        # In a real scenario, you'd have recorded positions of all unique detections
        result.people_positions = []
        result.cat_positions = []
        result.dog_positions = []

        # Complete the action
        rospy.loginfo("Patrol completed successfully.")
        self.server.set_succeeded(result)

    def check_for_new_detections(self):

        # In your execute or patrol loop:
        yolo_client = rospy.ServiceProxy('/yolo_detect', YoloDetect)
        detections = yolo_client(YoloDetectRequest())  # Assuming an empty request

        # Check if any new objects are detected compared to last known detections.
        if len(detections.people_positions) > 0 or len(detections.cats_positions) > 0 or len(
                detections.dogs_positions) > 0:

        # Update feedback and publish it.
        # Only publish if new detections have appeared since the last time you published feedback.
        # You might keep a record of previously reported positions to filter out duplicates.

        # No new detections
        return (0, 0, 0, Pose())


if __name__ == "__main__":
    rospy.init_node("patrol_server")
    server = PatrolServer()
    rospy.spin()
