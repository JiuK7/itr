#!/usr/bin/env python3

import rospy
import smach
from geometry_msgs.msg import Pose
from your_custom_package.srv import SayText, SayTextRequest  # Update with your actual package and srv name
from visualization_msgs.msg import Marker

# Marker types
PEOPLE_MARKER_TYPE = Marker.SPHERE
CATS_MARKER_TYPE = Marker.CUBE
DOGS_MARKER_TYPE = Marker.CYLINDER

# Colors (R,G,B)
# People: new = red, old = blue
PEOPLE_NEW_COLOR = (1.0, 0.0, 0.0)   # red
PEOPLE_OLD_COLOR = (0.0, 0.0, 1.0)   # blue

# Cats: new = green, old = darker green
CATS_NEW_COLOR = (0.0, 1.0, 0.0)     # green
CATS_OLD_COLOR = (0.0, 0.5, 0.0)     # dark green

# Dogs: new = yellow, old = darker yellow
DOGS_NEW_COLOR = (1.0, 1.0, 0.0)     # yellow
DOGS_OLD_COLOR = (0.5, 0.5, 0.0)     # dark yellow


def report_and_feedback_callback(userdata, request):
    """
    Callback for REPORT_AND_FEEDBACK state.

    This function:
    - Identifies new detections not seen previously.
    - If new people are found, triggers TTS to announce them.
    - Updates main detected lists with newly found entities.
    - Publishes feedback.
    - Publishes visualization markers for all currently known detections,
      with different shapes and colors for newly found vs. previously known.
    """

    # Store previously known lists for comparison
    previously_known_people = userdata.detected_people
    previously_known_cats = userdata.detected_cats
    previously_known_dogs = userdata.detected_dogs

    # Find new detections by comparing current lists with previously known lists
    new_people = [p for p in userdata.current_detected_people if p not in previously_known_people]
    new_cats = [c for c in userdata.current_detected_cats if c not in previously_known_cats]
    new_dogs = [d for d in userdata.current_detected_dogs if d not in previously_known_dogs]

    # Announce new people via TTS if any
    if new_people:
        try:
            rospy.wait_for_service('/text_to_speech', timeout=5.0)
            tts_srv = rospy.ServiceProxy('/text_to_speech', SayText)
            # Construct a message - for simplicity, just count how many people
            text_msg = "I see " + str(len(new_people)) + " new person" + ("" if len(new_people) == 1 else "s")
            resp = tts_srv(SayTextRequest(text=text_msg))
            if resp:
                rospy.loginfo("[REPORT_AND_FEEDBACK] TTS announcement made: %s", text_msg)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("[REPORT_AND_FEEDBACK] Failed to call TTS service: %s", e)

    # Update the master lists with newly detected entities
    userdata.detected_people.extend(new_people)
    userdata.detected_cats.extend(new_cats)
    userdata.detected_dogs.extend(new_dogs)

    # Publish feedback if function is available
    if hasattr(userdata, 'publish_feedback') and callable(userdata.publish_feedback):
        feedback_msg = {
            "new_people_count": len(new_people),
            "new_cats_count": len(new_cats),
            "new_dogs_count": len(new_dogs),
            "total_people_count": len(userdata.detected_people),
            "total_cats_count": len(userdata.detected_cats),
            "total_dogs_count": len(userdata.detected_dogs)
        }
        userdata.publish_feedback(feedback_msg)
        rospy.loginfo("[REPORT_AND_FEEDBACK] Feedback published: %s", feedback_msg)

    # Publish visualization markers
    # Markers will be published for all known detections (both old and newly found)
    # New detections are shown in 'new' colors, old in 'old' colors.
    marker_pub = userdata.marker_pub  # Access the publisher from userdata

    def publish_markers(detections, marker_type, new_set, color_new, color_old, start_id):
        current_id = start_id
        for p in detections:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "detections"
            marker.id = current_id
            marker.type = marker_type
            marker.action = Marker.ADD
            marker.pose = p
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            if p in new_set:
                # New detection color
                marker.color.a = 1.0
                marker.color.r = color_new[0]
                marker.color.g = color_new[1]
                marker.color.b = color_new[2]
            else:
                # Previously known detection color
                marker.color.a = 1.0
                marker.color.r = color_old[0]
                marker.color.g = color_old[1]
                marker.color.b = color_old[2]

            marker_pub.publish(marker)
            current_id += 1
        return current_id

    # Publish people markers
    marker_id = 0
    marker_id = publish_markers(userdata.detected_people, PEOPLE_MARKER_TYPE, new_people,
                                PEOPLE_NEW_COLOR, PEOPLE_OLD_COLOR, marker_id)

    # Publish cat markers
    marker_id = publish_markers(userdata.detected_cats, CATS_MARKER_TYPE, new_cats,
                                CATS_NEW_COLOR, CATS_OLD_COLOR, marker_id)

    # Publish dog markers
    marker_id = publish_markers(userdata.detected_dogs, DOGS_MARKER_TYPE, new_dogs,
                                DOGS_NEW_COLOR, DOGS_OLD_COLOR, marker_id)

    # Clear current detection lists after handling
    userdata.current_detected_people = []
    userdata.current_detected_cats = []
    userdata.current_detected_dogs = []

    rospy.loginfo("[REPORT_AND_FEEDBACK] Reporting, feedback, and marker publishing complete.")
    return 'done'


def main():
    rospy.init_node('report_and_feedback_state_test')

    sm = smach.StateMachine(outcomes=['finished'])
    # Initialize userdata with some known detections
    sm.userdata.detected_people = []
    sm.userdata.detected_cats = []
    sm.userdata.detected_dogs = []

    # For testing, we found one new person
    sm.userdata.current_detected_people = [Pose(position=dict(x=1.0, y=2.0, z=0.0))]
    sm.userdata.current_detected_cats = []
    sm.userdata.current_detected_dogs = []

    # Create a marker publisher and store in userdata so it can be accessed in the callback
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    sm.userdata.marker_pub = marker_pub

    with sm:
        smach.StateMachine.add('REPORT_AND_FEEDBACK',
                               smach.CBState(report_and_feedback_callback,
                                             outcomes=['done'],
                                             input_keys=['detected_people',
                                                         'detected_cats',
                                                         'detected_dogs',
                                                         'current_detected_people',
                                                         'current_detected_cats',
                                                         'current_detected_dogs',
                                                         'marker_pub'],  # include marker_pub
                                             output_keys=['detected_people',
                                                          'detected_cats',
                                                          'detected_dogs',
                                                          'current_detected_people',
                                                          'current_detected_cats',
                                                          'current_detected_dogs']),
                               transitions={'done': 'finished'})

    outcome = sm.execute()
    rospy.loginfo("State Machine finished with outcome: %s", outcome)
    rospy.loginfo("Updated known detections - People: %d, Cats: %d, Dogs: %d",
                  len(sm.userdata.detected_people),
                  len(sm.userdata.detected_cats),
                  len(sm.userdata.detected_dogs))


if __name__ == '__main__':
    main()
