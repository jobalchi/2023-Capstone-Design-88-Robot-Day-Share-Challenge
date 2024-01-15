#!/usr/bin/env python3

import sys
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node

import argparse
import torch
import cv2
import pyzed.sl as sl
import torch.backends.cudnn as cudnn

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros

sys.path.insert(0, '/home/holo/yolov5')
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_boxes, xyxy2xywh
from utils.torch_utils import select_device
from utils.augmentations import letterbox

from threading import Lock, Thread
from time import sleep

sys.path.insert(0, '/home/holo/ros2_ws/src/HOLO_BOT/holo_yolo/include/holo_yolo')
import ogl_viewer.viewer as gl
import cv_viewer.tracking_viewer as cv_viewer

lock = Lock()
run_signal = False
exit_signal = False

def img_preprocess(img, device, half, net_size):
    net_image, ratio, pad = letterbox(img[:, :, :3], net_size, auto=False)
    net_image = net_image.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    net_image = np.ascontiguousarray(net_image)

    img = torch.from_numpy(net_image).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0

    if img.ndimension() == 3:
        img = img.unsqueeze(0)
    return img, ratio, pad

def xywh2abcd(xywh, im_shape):
    output = np.zeros((4, 2))

    # Center / Width / Height -> BBox corners coordinates
    x_min = (xywh[0] - 0.5*xywh[2]) * im_shape[1]
    x_max = (xywh[0] + 0.5*xywh[2]) * im_shape[1]
    y_min = (xywh[1] - 0.5*xywh[3]) * im_shape[0]
    y_max = (xywh[1] + 0.5*xywh[3]) * im_shape[0]

    # A ------ B
    # | Object |
    # D ------ C

    output[0][0] = x_min
    output[0][1] = y_min

    output[1][0] = x_max
    output[1][1] = y_min

    output[2][0] = x_min
    output[2][1] = y_max

    output[3][0] = x_max
    output[3][1] = y_max
    return output

def detections_to_custom_box(detections, im, im0):
    output = []
    for i, det in enumerate(detections):
        if len(det):
            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh

            for *xyxy, conf, cls in reversed(det):
                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh

                # Creating ingestable objects for the ZED SDK
                obj = sl.CustomBoxObjectData()
                obj.bounding_box_2d = xywh2abcd(xywh, im0.shape)
                obj.label = cls
                obj.probability = conf
                obj.is_grounded = False
                # obj.id = i
                output.append(obj)          
    return output

def torch_thread(weights, img_size, conf_thres=0.2, iou_thres=0.45):
    global image_net, exit_signal, run_signal, detections

    print("Intializing Network...")

    device = select_device()
    half = device.type != 'cpu'  # half precision only supported on CUDA
    imgsz = img_size

    # Load model
    model = attempt_load(weights, device = 'cuda')  # load FP32
    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # check img_size
    if half:
        model.half()  # to FP16
    cudnn.benchmark = True

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once

    while not exit_signal:
        if run_signal:
            lock.acquire()
            img, ratio, pad = img_preprocess(image_net, device, half, imgsz)

            pred = model(img)[0]
            det = non_max_suppression(pred, conf_thres, iou_thres)

            # ZED CustomBox format (with inverse letterboxing tf applied)
            detections = detections_to_custom_box(det, img, image_net)
            lock.release()
            run_signal = False
        sleep(0.01)

class DetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detect_node')

        self.declare_parameter('weights', '/home/holo/ros2_ws/src/HOLO_BOT/holo_yolo/include/holo_yolo/best.pt')
        self.declare_parameter('svo', None)
        self.declare_parameter('img_size', 416)
        self.declare_parameter('conf_thres', 0.4)
        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.kick_pose_pub = self.create_publisher(PoseStamped, 'kick_pose', 10)
        self.yolo_go_pub = self.create_publisher(Int32, 'yolo_go', 10)
        self.find_kick_sub = self.create_subscription(Int32, 'find_kick', self.detect_display, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.find_kick_sub

    def detect_display(self, msg):
        global image_net, exit_signal, run_signal, detections

        weights = self.get_parameter('weights').value
        svo = self.get_parameter('svo').value
        img_size = self.get_parameter('img_size').value
        conf_thres = self.get_parameter('conf_thres').value

        if msg.data == 1:
            capture_thread = Thread(target= torch_thread,
                                    kwargs={'weights': weights, 'img_size': img_size, "conf_thres": conf_thres})
            capture_thread.start()


            print("Initializing Camera...")

            zed = sl.Camera()

            input_type = sl.InputType()
            if svo is not None:
                input_type.set_from_svo_file(svo)

            # Create a InitParameters object and set configuration parameters
            init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=True)
            init_params.camera_resolution = sl.RESOLUTION.HD720
            init_params.coordinate_units = sl.UNIT.METER
            init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # QUALITY
            init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
            init_params.depth_maximum_distance = 50

            runtime_params = sl.RuntimeParameters()
            status = zed.open(init_params)

            if status != sl.ERROR_CODE.SUCCESS:
                print(repr(status))
                exit()

            image_left_tmp = sl.Mat()

            print("Initialized Camera")

            positional_tracking_parameters = sl.PositionalTrackingParameters()
            # If the camera is static, uncomment the following line to have better performances and boxes sticked to the ground.
            # positional_tracking_parameters.set_as_static = True
            zed.enable_positional_tracking(positional_tracking_parameters)

            obj_param = sl.ObjectDetectionParameters()
            obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
            obj_param.enable_tracking = True
            zed.enable_object_detection(obj_param)

            objects = sl.Objects()
            obj_runtime_param = sl.ObjectDetectionRuntimeParameters()

            # Display
            camera_infos = zed.get_camera_information()
            camera_res = camera_infos.camera_configuration.resolution
            # Create OpenGL viewer
            viewer = gl.GLViewer()
            point_cloud_res = sl.Resolution(min(camera_res.width, 720), min(camera_res.height, 404))
            point_cloud_render = sl.Mat()
            viewer.init(camera_infos.camera_model, point_cloud_res, obj_param.enable_tracking)
            point_cloud = sl.Mat(point_cloud_res.width, point_cloud_res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)
            image_left = sl.Mat()
            # Utilities for 2D display
            display_resolution = sl.Resolution(min(camera_res.width, 1280), min(camera_res.height, 720))
            image_scale = [display_resolution.width / camera_res.width, display_resolution.height / camera_res.height]
            image_left_ocv = np.full((display_resolution.height, display_resolution.width, 4), [245, 239, 239, 255], np.uint8)

            # Utilities for tracks view
            camera_config = camera_infos.camera_configuration
            tracks_resolution = sl.Resolution(400, display_resolution.height)
            track_view_generator = cv_viewer.TrackingViewer(tracks_resolution, camera_config.fps, init_params.depth_maximum_distance)
            track_view_generator.set_camera_calibration(camera_config.calibration_parameters)
            image_track_ocv = np.zeros((tracks_resolution.height, tracks_resolution.width, 4), np.uint8)
            cam_w_pose = sl.Pose()

            while viewer.is_available() and not exit_signal:
                if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                    # -- Get the image
                    lock.acquire()
                    zed.retrieve_image(image_left_tmp, sl.VIEW.LEFT)
                    image_net = image_left_tmp.get_data()
                    lock.release()
                    run_signal = True

                    # -- Detection running on the other thread
                    while run_signal:
                        sleep(0.001)

                    # Wait for detections
                    lock.acquire()
                    # -- Ingest detections
                    zed.ingest_custom_box_objects(detections)
                    lock.release()
                    zed.retrieve_objects(objects, obj_runtime_param)

                    # -- Display
                    # Retrieve display data
                    zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, point_cloud_res)
                    point_cloud.copy_to(point_cloud_render)
                    zed.retrieve_image(image_left, sl.VIEW.LEFT, sl.MEM.CPU, display_resolution)
                    zed.get_position(cam_w_pose, sl.REFERENCE_FRAME.WORLD)

                    for obj in objects.object_list:
                        position = obj.position

                        x_object = position[0]
                        y_object = position[1]
                        z_object = position[2]

                        x_camera = cam_w_pose.pose_data()[0, 3]
                        y_camera = cam_w_pose.pose_data()[1, 3]
                        z_camera = cam_w_pose.pose_data()[2, 3]
                        
                        camera_to_distance = np.sqrt(x_camera**2 + y_camera**2 + z_camera**2)
                        distance = np.sqrt((x_object - x_camera)**2 + (y_object - y_camera)**2 + (z_object - z_camera)**2)

                        if not np.isnan(distance) and not np.isinf(distance):
                            print("Distance to Reference Map Frame at ({}, {}): {:1.3} m".format(x_object, y_object, distance))
                            print("Distacne to Reference Map Frame to Camera ({}, {}): {:1.3} m".format(x_camera, y_camera, camera_to_distance))

                            t = TransformStamped()
                            t.header.stamp = Time()
                            t.header.frame_id = "zed2i_base_link"
                            t.child_frame_id = "kick_pose"
                            t.transform.translation.x = x_object
                            t.transform.translation.y = y_object
                            t.transform.translation.z = z_object
                            t.transform.rotation.x = 0.0
                            t.transform.rotation.y = 0.0
                            t.transform.rotation.z = 0.0
                            t.transform.rotation.w = 1.0
                            self.tf_broadcaster.sendTransform(t)

                            kick_pose = PoseStamped()
                            kick_pose.header.stamp = Time()
                            kick_pose.header.frame_id = "kick_pose"
                            kick_pose.pose.position.x = x_object
                            kick_pose.pose.position.y = y_object
                            kick_pose.pose.position.z = z_object
                            kick_pose.pose.orientation.x = 0.0
                            kick_pose.pose.orientation.y = 0.0
                            kick_pose.pose.orientation.z = 0.0
                            kick_pose.pose.orientation.w = 1.0
                            yolo_go_msg = Int32()
                            yolo_go_msg.data = 1
                            self.kick_pose_pub.publish(kick_pose)
                            self.yolo_go_pub.publish(yolo_go_msg)

                            self.get_logger().info('Detecting kick_pose: [%.2f, %.2f, %.2f]' % (position[0], position[1], position[2]))
                            self.get_logger().info('yolo_go: %d' % yolo_go_msg.data)
                    
                    # 3D rendering
                    viewer.updateData(point_cloud_render, objects)
                    # 2D rendering
                    np.copyto(image_left_ocv, image_left.get_data())
                    cv_viewer.render_2D(image_left_ocv, image_scale, objects, obj_param.enable_tracking)
                    global_image = cv2.hconcat([image_left_ocv, image_track_ocv])
                    # Tracking view
                    track_view_generator.generate_view(objects, cam_w_pose, image_track_ocv, objects.is_tracked)
                        
                    cv2.imshow("ZED | 2D View and Birds View", global_image)
                    key = cv2.waitKey(10)
                    if key == 27:
                        exit_signal = True
                else:
                    exit_signal = True     
            
            viewer.exit()
            exit_signal = True
            zed.close()
        else: 
            self.destroy_node()     
    

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()