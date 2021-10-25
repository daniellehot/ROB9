#!/usr/bin/env python3
from __future__ import print_function

from realsense_service.srv import intrinsics, intrinsicsResponse
from realsense_service.srv import capture, captureResponse
from realsense_service.srv import depth, depthResponse
from realsense_service.srv import rgb, rgbResponse
from std_msgs.msg import Header, Float32MultiArray
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import rospy
import pyrealsense2 as rs
import numpy as np
from cv_bridge import CvBridge
import time
from ctypes import * # convert float to uint32

cam_width = 1280
cam_height = 720

class CameraInfo():
    # from https://github.com/graspnet/graspnet-baseline/blob/main/utils/data_utils.py
    """ Camera intrisics for point cloud creation. """
    def __init__(self, width, height, fx, fy, cx, cy, scale):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.scale = scale

def create_point_cloud_from_depth_image(depth, camera, organized=True):
    # from https://github.com/graspnet/graspnet-baseline/blob/main/utils/data_utils.py
    """ Generate point cloud using depth image only.
        Input:
            depth: [numpy.ndarray, (H,W), numpy.float32]
                depth image
            camera: [CameraInfo]
                camera intrinsics
            organized: bool
                whether to keep the cloud in image shape (H,W,3)
        Output:
            cloud: [numpy.ndarray, (H,W,3)/(H*W,3), numpy.float32]
                generated cloud, (H,W,3) for organized=True, (H*W,3) for organized=False
    """
    assert(depth.shape[0] == camera.height and depth.shape[1] == camera.width)
    xmap = np.arange(camera.width)
    ymap = np.arange(camera.height)
    xmap, ymap = np.meshgrid(xmap, ymap)
    points_z = depth / camera.scale
    points_x = (xmap - camera.cx) * points_z / camera.fx
    points_y = (ymap - camera.cy) * points_z / camera.fy
    cloudGeom = np.stack([points_x, points_y, points_z], axis=-1)
    cloudPoints = np.reshape(cloudGeom, (-1,3)).astype(np.float32)

    # convert data
    cloudPoints = cloudPoints[cloudPoints[:,2] < 1.0]
    cloudPoints = cloudPoints[cloudPoints[:,2] > 0.0]
    cloudPoints = cloudPoints[cloudPoints[:,0] < 0.8]
    cloudPoints = cloudPoints[cloudPoints[:,0] > -0.8]
    return cloudPoints


def sendIntrinsics():
    pass

def sendDepth(command):
    global depth_image
    msg = depthResponse()
    msg.img.data = depth_image.flatten()
    msg.width.data = depth_image.shape[1]
    msg.height.data = depth_image.shape[0]
    return msg

def sendRGB(command):
    global color_image
    br = CvBridge()
    return br.cv2_to_imgmsg(color_image)



def captureRealsense(capture):
    global color_frame, depth_frame, depth_image, color_image, captured, framesRGB, framesDepth
    try:
        align_to = rs.stream.color
        align = rs.align(align_to)
        color_frame, depth_frame, color_image, depth_image = temporalFilter(framesRGB, framesDepth)

        generatePointcloud(depth_frame, color_frame, color_image)
        print("here")

        captured = True
        msg = captureResponse()
        msg.success.data = True
        return msg
    except:
        capture = False
        msg = captureResponse()
        msg.success.data = False
        return msg

def generatePointcloud(depth_frame, color_frame, color_image):
    global cloudGeometryStatic, cloudColorStatic

    cloud = rs.pointcloud()
    cloud.map_to(color_frame);
    points = rs.points()
    points = cloud.calculate(depth_frame)
    cloud = np.array(np.array(points.get_vertices()).tolist())

    uv = np.array(np.array(points.get_texture_coordinates()).tolist())
    uv[:,0] = uv[:,0] *cam_width
    uv[:,1] = uv[:,1] *cam_height
    uv[:, [1, 0]] = uv[:, [0, 1]]
    uv = np.rint(np.array(uv)).astype(int)

    idx = cloud[:,2] < 1.0
    cloud = cloud[idx]
    uv = uv[idx]
    idxs = np.random.choice(cloud.shape[0], 40000, replace=False)
    cloud = cloud[idxs]
    uv = uv[idxs]

    colors = []
    for idx in uv:
        colors.append(color_image[idx[0], idx[1]])
    colors = np.array(colors)

    colors = colors.flatten()

    cloudGeometryStatic = cloud
    cloudColorStatic = colors

def temporalFilter(framesRGB, framesDepth):

    temporal = rs.temporal_filter()

    for x in range(len(framesRGB)):
        filteredRGB = temporal.process(framesRGB[x])

    for x in range(len(framesDepth)):
        filteredDepth = temporal.process(framesDepth[x])


    color_frame = filteredRGB
    depth_frame = filteredDepth
    print(color_frame)
    print(filteredDepth)
    # Convert images to numpy arrays
    color_image = np.asanyarray(filteredRGB.get_data())
    depth_image = np.asanyarray(filteredDepth.get_data())

    return filteredRGB, filteredDepth, color_image, depth_image

if __name__ == "__main__":
    global profile, pipeline, captured, color_image, depth_image, cloudGeometryStatic, cloudColorStatic, framesRGB, framesDepth

    FIELDS_XYZ = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    # Initially no images are available
    captured = False

    pipeline = rs.pipeline()
    config = rs.config()

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))


    config.enable_stream(rs.stream.color, cam_width, cam_height, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, cam_width, cam_height, rs.format.z16, 30)
    profile = pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()

    preset_range = depth_sensor.get_option_range(rs.option.visual_preset)
    for i in range(int(preset_range.max)):
        visulpreset = depth_sensor.get_option_value_description(rs.option.visual_preset,i)
        print(i,visulpreset)
        if visulpreset == "High Accuracy":
            depth_sensor.set_option(rs.option.visual_preset, i)
            print("high accuracy")

    align_to = rs.stream.color
    align = rs.align(align_to)

    time.sleep(3)
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    print(aligned_frames)
    print(aligned_frames.get_color_frame())
    #exit()

    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    generatePointcloud(depth_frame, color_frame, color_image)

    captured = True

    br = CvBridge()

    rospy.init_node('realsense_service')
    serviceCapture = rospy.Service('/sensors/realsense/capture', capture, captureRealsense)
    serviceCaptureDepth = rospy.Service('/sensors/realsense/depth', depth, sendDepth)
    serviceCaptureRGB = rospy.Service('/sensors/realsense/rgb', rgb, sendRGB)
    pubPointCloudGeometryStatic = rospy.Publisher("/sensors/realsense/pointcloudGeometry/static", PointCloud2, queue_size=1)
    pubStaticRGB = rospy.Publisher("/sensors/realsense/rgb/static", Image, queue_size=1)
    pubStaticDepth = rospy.Publisher("sensors/realsense/depth/static", Image, queue_size = 1)
    pubPointCloudGeometryStaticRGB = rospy.Publisher('/sensors/realsense/pointcloudGeometry/static/rgb', Float32MultiArray, queue_size=1)

    intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

    fx = intr.fx
    fy = intr.fy
    ppx = intr.ppx
    ppy = intr.ppy
    camera = CameraInfo(1280.0, 720.0, fx, fy, ppx, ppy, 1000)

    rate = rospy.Rate(6)

    framesRGB, framesDepth = [], []
    while not rospy.is_shutdown():

        frame = pipeline.wait_for_frames()
        aligned_frame = align.process(frame)

        framesRGB.append(aligned_frames.get_color_frame())
        framesDepth.append(aligned_frames.get_depth_frame())
        if len(framesRGB) > 10:
            framesRGB.pop()
        if len(framesDepth) > 10:
            framesDepth.pop()

        # publish rgb static
        if captured:

            print("Frames Captured")

            print("Sending: ", rospy.Time.now())

            # publishing rgb image
            pubStaticRGB.publish(br.cv2_to_imgmsg(color_image))

            # publishing geometry point cloud static
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "ptu_camera_color_optical_frame"
            pubPointCloudGeometryStatic.publish(pc2.create_cloud(header, FIELDS_XYZ, cloudGeometryStatic))

            # publishing colors of static point
            msg = Float32MultiArray()
            msg.data = cloudColorStatic
            pubPointCloudGeometryStaticRGB.publish(msg)


        rate.sleep()
