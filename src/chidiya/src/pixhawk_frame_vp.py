#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped
# from mavros_msgs.msg import VisionPoseStamped

# Quaternion for transforming camera frame to pixhawk frame
# q_camera_to_pixhawk = [0.0, 0.707, 0.0, 0.707]

def transform_callback(tf_msg):
    # Lookup transform from camera to apriltag
    try:
        transform = tf_buffer.lookup_transform("usb_cam0", "drone", rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return

    # Convert transform to pose in camera frame
    pose_camera = PoseStamped()
    pose_camera.header.stamp = transform.header.stamp
    pose_camera.header.frame_id = "camera"
    pose_camera.pose.position = transform.transform.translation
    pose_camera.pose.orientation = transform.transform.rotation

    # Transform pose to pixhawk frame
    pose_pixhawk = tf2_geometry_msgs.do_transform_pose(pose_camera, TransformStamped(
        header=transform.header,
        child_frame_id="pixhawk",
        transform=transform.transform
    ))

    # Publish pose on mavros topic
    vision_pose = PoseStamped()
    vision_pose.header.stamp = rospy.Time.now()
    vision_pose.header.frame_id = "pixhawk"
    vision_pose.pose = pose_pixhawk.pose
    # vision_pose.pose.covariance[0] = 0.01  # set covariance of x to 0.01
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

    rospy.spin()

