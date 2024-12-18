#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from second_coursework.srv import YoloDetect, YoloDetectResponse

def run_yolo_detection():
    """
    Placeholder function for YOLO detection.
    This should:
    - Grab the latest camera image.
    - Run YOLO model inference.
    - Extract bounding boxes for people, cats, and dogs.
    - Convert bounding boxes to world or robot-centric coordinates and return them as arrays of Poses.

    Here we just return dummy data for demonstration.
    """

    # In a real implementation, you'd run YOLO inference and populate these arrays.
    # For demonstration, we pretend we found one person, one cat, and no dogs.
    people = []
    cats = []
    dogs = []

    # Example: A single person at (x=1.0, y=0.5)
    p = Pose()
    p.position.x = 1.0
    p.position.y = 0.5
    p.position.z = 0.0
    p.orientation.w = 1.0
    people.append(p)

    # Example: A single cat at (x=2.0, y=-0.5)
    c = Pose()
    c.position.x = 2.0
    c.position.y = -0.5
    c.position.z = 0.0
    c.orientation.w = 1.0
    cats.append(c)

    # No dogs detected
    return people, cats, dogs

def handle_yolo_detect(req):
    # Run YOLO detection and get results
    people, cats, dogs = run_yolo_detection()

    # Prepare response
    resp = YoloDetectResponse()
    resp.people_positions = people
    resp.cats_positions = cats
    resp.dogs_positions = dogs
    return resp

def yolo_detection_server():
    rospy.init_node('yolo_detection_server', anonymous=True)
    rospy.Service('/yolo_detect', YoloDetect, handle_yolo_detect)
    rospy.loginfo("[YOLO_DETECTION_SERVER] Service /yolo_detect is ready.")
    rospy.spin()

if __name__ == "__main__":
    yolo_detection_server()
