#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped
# from mavros_msgs.msg import VisionPoseStamped

# Quaternion for transforming camera frame to pixhawk frame
#q_camera_to_pixhawk = [0.0, 0.707, 0.0, 0.707]

def transform_callback(tf_msg):
    # Lookup transform from camera to apriltag
    try:
        transform = tf_buffer.lookup_transform("usb_cam0", "drone", rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return

    # Convert transform to pose in camera frame
    # Transform pose to pixhawk frame
    # Publish pose on mavros topic
    vision_pose = PoseStamped()
    vision_pose.header.stamp = rospy.Time.now()
    vision_pose.header.frame_id = 'map'
    # vision_pose.pose = pose_pixhawk.pose
    # vision_pose.pose.covariance[0] = 0.01  # set covariance of x to 0.01
    vision_pose.pose.position = transform.transform.translation
    vision_pose.pose.position.z = 0.0
    vision_pose.pose.orientation = transform.transform.rotation
    vision_pose_pub.publish(vision_pose)
    vision_pose.header.stamp = rospy.Time.now()
    vision_pose_pub.publish(vision_pose)
    vision_pose.header.stamp = rospy.Time.now()
    vision_pose_pub.publish(vision_pose)

if __name__ == '__main__':
    rospy.init_node('apriltag_tf_to_vision_pose')

    # Create TF2 buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Subscribe to "/tf" topic
    tf_sub = rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, transform_callback)

    # Publish on "/mavros/vision_pose/pose" topic
    vision_pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=10)
    #vision_pose_pub = rospy.Publisher("/mavros/mocap/pose", PoseStamped, queue_size=10)

    rospy.spin()

