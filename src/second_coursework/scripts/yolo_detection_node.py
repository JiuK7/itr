#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from second_coursework.srv import YoloDetect, YoloDetectResponse

import torch

# Classes for reference (COCO): person=0, cat=15, dog=16
TARGET_CLASSES = {0: 'person', 15: 'cat', 16: 'dog'}

class YoloDetectionServer:
    def __init__(self):
        rospy.init_node('yolo_detection_server', anonymous=True)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribe to camera image
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.current_frame = None

        # Subscribe to robot odometry to get robot pose
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_pose = Pose()

        # Load YOLO model
        rospy.loginfo("[YOLO_DETECTION_SERVER] Loading YOLO model...")
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.conf = 0.5
        rospy.loginfo("[YOLO_DETECTION_SERVER] YOLO model loaded.")

        # Service
        self.service = rospy.Service('/yolo_detect', YoloDetect, self.handle_yolo_detect)
        rospy.loginfo("[YOLO_DETECTION_SERVER] Service /yolo_detect is ready.")

        # Store previously reported detections
        # Each is a list of Poses for that category
        self.reported_people = []
        self.reported_cats = []
        self.reported_dogs = []

        self.duplicate_distance = 1.0  # 1 meter threshold

    def image_callback(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CV Bridge error: %s", e)
            self.current_frame = None

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def run_yolo_detection(self):
        """
        Run YOLO detection on the current frame.
        Returns lists of new unique detections for people, cats, and dogs.
        """
        if self.current_frame is None:
            return [], [], []

        results = self.model(self.current_frame)
        detections = results.xyxy[0].cpu().numpy()

        # We'll return the robot's pose for each detected object as the "location"
        # Check uniqueness against previously reported positions
        new_people = []
        new_cats = []
        new_dogs = []

        # Current robot pose at detection time
        # (If you had actual object coordinates, you would use them instead.)
        detection_pose = Pose()
        detection_pose.position.x = self.current_pose.position.x
        detection_pose.position.y = self.current_pose.position.y
        detection_pose.position.z = self.current_pose.position.z
        detection_pose.orientation = self.current_pose.orientation

        # First, gather what we found
        found_people = False
        found_cats = False
        found_dogs = False

        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            cls = int(cls)
            if cls in TARGET_CLASSES and conf > 0.5:
                # We detected a target object
                obj_type = TARGET_CLASSES[cls]

                # Check if within 1 meter of previously reported same-type detection
                if obj_type == 'person':
                    if self.is_unique(detection_pose, self.reported_people):
                        new_people.append(detection_pose)
                        self.reported_people.append(detection_pose)
                        found_people = True
                elif obj_type == 'cat':
                    if self.is_unique(detection_pose, self.reported_cats):
                        new_cats.append(detection_pose)
                        self.reported_cats.append(detection_pose)
                        found_cats = True
                elif obj_type == 'dog':
                    if self.is_unique(detection_pose, self.reported_dogs):
                        new_dogs.append(detection_pose)
                        self.reported_dogs.append(detection_pose)
                        found_dogs = True

        return new_people, new_cats, new_dogs

    def is_unique(self, pose, reported_list):
        """
        Check if the given pose is at least 1 meter away from all poses in reported_list.
        """
        for r_pose in reported_list:
            dist = self.distance(pose, r_pose)
            if dist < self.duplicate_distance:
                return False
        return True

    def distance(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def handle_yolo_detect(self, req):
        # Run YOLO detection and get unique results
        people, cats, dogs = self.run_yolo_detection()

        # Prepare response
        resp = YoloDetectResponse()
        resp.people_positions = people
        resp.cats_positions = cats
        resp.dogs_positions = dogs

        # Log what we found
        total = len(people) + len(cats) + len(dogs)
        if total > 0:
            rospy.loginfo("[YOLO_DETECTION_SERVER] New unique detections:")
            rospy.loginfo(f"People: {len(people)}, Cats: {len(cats)}, Dogs: {len(dogs)}")
        else:
            rospy.loginfo("[YOLO_DETECTION_SERVER] No new unique detections.")

        return resp

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    server = YoloDetectionServer()
    server.spin()
