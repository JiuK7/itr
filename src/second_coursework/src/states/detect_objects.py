# states/detect_objects.py

import rospy
from second_coursework.srv import YoloDetect

MAX_RETRIES = 3
RETRY_DELAY = 1  # seconds

def detect_objects_callback(ud, request):
    """
    Callback function for the DETECT_OBJECTS state.
    Calls the /yolo_detect service and stores detected objects in userdata.
    """
    service_name = rospy.get_param('~yolo_service', '/yolo_detect')
    max_retries = rospy.get_param('~yolo_max_retries', MAX_RETRIES)
    retry_delay = rospy.get_param('~yolo_retry_delay', RETRY_DELAY)  # seconds

    for attempt in range(max_retries):
        try:
            rospy.loginfo("[DETECT_OBJECTS] Attempting to call /yolo_detect service (Attempt {}/{})".format(attempt+1, max_retries))
            rospy.wait_for_service(service_name, timeout=5)
            yolo_detect = rospy.ServiceProxy(service_name, YoloDetect)
            detect_response = yolo_detect()

            # Store detected objects in userdata
            ud.current_detected_people = detect_response.people_positions
            ud.current_detected_cats = detect_response.cats_positions
            ud.current_detected_dogs = detect_response.dogs_positions

            rospy.loginfo("[DETECT_OBJECTS] YOLO detection complete.")
            rospy.loginfo(f"Detected people: {len(ud.current_detected_people)}")
            rospy.loginfo(f"Detected cats: {len(ud.current_detected_cats)}")
            rospy.loginfo(f"Detected dogs: {len(ud.current_detected_dogs)}")

            return 'done'

        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(f"[DETECT_OBJECTS] Service call failed: {e}")
            if attempt < max_retries - 1:
                rospy.logwarn(f"[DETECT_OBJECTS] Retrying in {retry_delay} seconds...")
                rospy.sleep(retry_delay)
            else:
                rospy.logerr("[DETECT_OBJECTS] All retry attempts failed.")
                return 'aborted'
