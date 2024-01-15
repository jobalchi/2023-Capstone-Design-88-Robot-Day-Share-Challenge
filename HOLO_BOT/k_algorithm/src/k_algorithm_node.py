#!/usr/bin/env python3

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

import numpy as np
import cv2
import tf_transformations

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray, Pose, TransformStamped, PoseStamped
from std_msgs.msg import Int32

from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("k_algorithm_node")

        # Declare and read parameters
        self.declare_parameter(
            name="marker_size",
            value=0.0100,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_6X6_100",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/zed2i/zed_node/rgb/image_rect_color",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/zed2i/zed_node/rgb/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="zed2i_left_camera_optical_frame",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {dictionary_id_name}")

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image info topic: {info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.t = None

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_6X6_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(dictionary_id_name)
            )
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)
        self.k_point_pub = self.create_publisher(PoseStamped, 'k_point', 10)
        self.k_goal_pub = self.create_publisher(Int32, 'k_goal', 10)
        self.k_go_sub = self.create_subscription(Int32, 'k_go', self.k_go_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.k_go_sub
        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.bridge = CvBridge()

        self.latest_image = None

    def k_go_callback(self, msg):
        # self.get_logger().info('Received k_go message: %d' % msg.data)
        if msg.data == 1:
            self.timer_callback()
        else:
            self.destroy_node()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        self.latest_image = img_msg

    def timer_callback(self):

        if self.latest_image is None:
            self.get_logger().warn("No camera info has been received!")
            return

        from_frame_rel = self.camera_frame
        to_frame_rel = 'map'
        try:
            self.t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except tf2_ros.TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')        

        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
        
        markers = ArucoMarkers()
        pose_array = PoseArray()
        if self.camera_frame is None:
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = "map"
            pose_array.header.frame_id = "map"

        markers.header.stamp = self.latest_image.header.stamp
        pose_array.header.stamp = self.latest_image.header.stamp

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
        )
        cv2.aruco.drawDetectedMarkers(cv_image, corners, marker_ids)  # Draw detected markers on the image
        cv2.imshow('Detected markers', cv_image)  # Show the image
        cv2.waitKey(1)

        if marker_ids is not None:

            if self.t is None:
                self.get_logger().warning("No")
                return

            if cv2.__version__ > "4.0.0":
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.intrinsic_mat, self.distortion
                )
            else:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.intrinsic_mat, self.distortion
                )
            for i, marker_id in enumerate(marker_ids):
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                trans_matrix = tf_transformations.quaternion_matrix([
                    self.t.transform.rotation.x,
                    self.t.transform.rotation.y, 
                    self.t.transform.rotation.z,
                    self.t.transform.rotation.w
                ])                
            if cv2.__version__ > '4.0.0':

                # trans_marker_pose = (trans_matrix[0:3, 0:3].T @ np.array([tvecs[i][0][0], tvecs[i][0][1], tvecs[i][0][2]]).reshape(3, 1)).reshape(1, 3)[0]
                # pose.position.x = self.t.transform.translation.x + trans_marker_pose[0]
                # pose.position.y = self.t.transform.translation.y + trans_marker_pose[1]
                # pose.position.z = self.t.transform.translation.z

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                self.get_logger().info("Detecting")

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

            self.publish_tf(pose, marker_id[0])
            self.publish_k(pose, marker_id[0])            
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
        else:
            self.get_logger().info("No Detected")     

    def publish_tf(self, pose, marker_id):
        t = TransformStamped()
        t.header.stamp = self.latest_image.header.stamp
        t.header.frame_id = "zed2i_left_camera_optical_frame"
        t.child_frame_id = f"marker_{marker_id}" 
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w
        self.tf_broadcaster.sendTransform(t)

    def publish_k(self, pose, marker_id):
        # num_samples = 10
        # k_point_samples = np.zeros((num_samples, 3))

        # for i in range(num_samples):
        #     k_point_samples[i, 0] = pose.position.x
        #     k_point_samples[i, 1] = pose.position.y
        #     k_point_samples[i, 2] = pose.position.z

        # average_k_point = np.mean(k_point_samples, axis=0)
        t = TransformStamped()
        t.header.stamp = self.latest_image.header.stamp
        t.header.frame_id = f"marker_{marker_id}"  
        t.child_frame_id = "K_point"
        if marker_id == 0:
            t.transform.translation.x = -1.2
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.7
        elif marker_id == 1:
            t.transform.translation.x = -0.85
            t.transform.translation.y = 0.0
            t.transform.translation.z = -0.45
        elif marker_id == 2:
            t.transform.translation.x = 0.85
            t.transform.translation.y = 0.0
            t.transform.translation.z = -0.45
            
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

        k_point = PoseStamped()
        k_point.header.stamp = self.latest_image.header.stamp
        k_point.header.frame_id = "K_point"
        k_goal_msg = Int32()
        k_point.pose.position.x = pose.position.z
        k_point.pose.position.y = pose.position.x
        k_point.pose.position.z = 0.0
        k_goal_msg.data = 1
        self.k_point_pub.publish(k_point)
        self.k_goal_pub.publish(k_goal_msg)

        self.get_logger().info('Averaged k_point: [%.2f, %.2f, %.2f]' % (
        k_point.pose.position.x, k_point.pose.position.y, k_point.pose.position.z
        ))

        self.get_logger().info('k_goal: %d' % k_goal_msg.data)
        # self.destroy_node()

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
