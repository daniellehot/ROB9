#!/usr/bin/env python
from __future__ import print_function

from realsense_service.srv import intrinsics, intrinsicsResponse
from realsense_service.srv import capture, captureResponse
from realsense_service.srv import depth, depthResponse
from realsense_service.srv import rgb, rgbResponse
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import rospy
import pyrealsense2 as rs
import numpy as np
import open3d as o3d
from cv_bridge import CvBridge

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
    cloud = np.reshape(cloudGeom, (-1,3))

    # convert data
    #cloud = o3d.geometry.PointCloud()
    #cloud.points = o3d.utility.Vector3dVector(cloudGeom)
    #cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
    return cloud


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
    global depth_image, color_image, captured
    try:
        align_to = rs.stream.color
        align = rs.align(align_to)

        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        captured = True
        msg = captureResponse()
        msg.success.data = True
        return msg
    except:
        capture = False
        msg = captureResponse()
        msg.success.data = False
        return msg

if __name__ == "__main__":
    global profile, pipeline, captured, color_image, depth_image

    FIELDS_XYZ = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    # Initially no images are available
    captured = False

    cam_width = 1280
    cam_height = 720

    pipeline = rs.pipeline()
    config = rs.config()

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    fx = 637.598
    fy = 637.598
    ppx = 646.101
    ppy = 360.912
    camera = CameraInfo(1280.0, 720.0, fx, fy, ppx, ppy, 1000)

    config.enable_stream(rs.stream.color, cam_width, cam_height, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, cam_width, cam_height, rs.format.z16, 30)
    profile = pipeline.start(config)

    align_to = rs.stream.color
    align = rs.align(align_to)

    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    captured = True

    br = CvBridge()

    rospy.init_node('realsense_service')
    serviceCapture = rospy.Service('/sensors/realsense/capture', capture, captureRealsense)
    serviceCaptureDepth = rospy.Service('/sensors/realsense/depth', depth, sendDepth)
    serviceCaptureRGB = rospy.Service('/sensors/realsense/rgb', rgb, sendRGB)
    pubPointCloudGeometryStatic = rospy.Publisher("/sensors/realsense/pointcloudGeometry/static", PointCloud2, queue_size=1)
    pubStaticRGB = rospy.Publisher("/sensors/realsense/rgb/static", Image, queue_size=1)
    pubStaticDepth = rospy.Publisher("sensors/realsense/depth/static", Image, queue_size = 1)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        # publish rgb static
        if captured:
            print("sending...")
            pubStaticRGB.publish(br.cv2_to_imgmsg(color_image))
            pubStaticDepth.publish(br.cv2_to_imgmsg(depth_image))

            cloudStatic = create_point_cloud_from_depth_image(depth_image, camera, organized=True)
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "ptu_camera_color_optical_frame"

            # Set "fields" and "cloud_data"
            #points=np.asarray(cloudStatic.points)
            #if not open3d_cloud.colors: # XYZ only
            fields=FIELDS_XYZ
            cloud_data=cloudStatic
            pubPointCloudGeometryStatic.publish(pc2.create_cloud(header, fields, cloud_data))

        rate.sleep()
