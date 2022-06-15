#!/usr/bin/python3.8
import rospy
import numpy as np
import tf

from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from pose_estimation.msg import Pose, PoseArray


KEYPOINTS = [
    "nose",
    "left_eye_inner",
    "left_eye",
    "left_eye_outer",
    "right_eye_inner",
    "right_eye",
    "right_eye_outer",
    "left_ear",
    "right_ear",
    "mouth_left",
    "mouth_right",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_pinky",
    "right_pinky",
    "left_index",
    "right_index",
    "left_thumb",
    "right_thumb",
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
    "left_heel",
    "right_heel",
    "left_foot_index",
    "right_foot_index"
]


class CameraNode:
    def __init__(self):
        self.br = CvBridge()
        self.img_pub = rospy.Publisher('global_camera/compressed', CompressedImage, queue_size=10)
        self.body_pub = rospy.Publisher('body_pose', PoseArray, queue_size = 10)
        self.viz_pub  = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

        self.tf_broadcaster = tf.TransformBroadcaster()

    def publish_image(self, img):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
        
        self.img_pub.publish(msg)

    def publish_body(self, body, marker):
        self.viz_pub.publish(marker)
        
        pose_array = PoseArray()

        for i in range(33):
            pose = Pose()
            pose.x = body.landmarks_world[i][0]
            pose.y = body.landmarks_world[i][1]
            pose.z = body.landmarks_world[i][2]

            pose.confidence = body.presence[i]
            pose.name = KEYPOINTS[i]

            pose_array.array.append(pose)

        self.body_pub.publish(pose_array)

        self.tf_broadcaster.sendTransform((-3.70, 14.2, 0.5), #(-0.671, 2.77, 0.5)
                                          tf.transformations.quaternion_from_euler(-np.pi/2, 0., np.pi/2),
                                          rospy.Time.now(),
                                          "human",
                                          "map")