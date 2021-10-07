#!/usr/bin/env python3

import os
import sys
import torch
import rospy
import argparse
import numpy as np
import open3d as o3d
import scipy.io as scio
from ctypes import * # convert float to uint32
from std_msgs.msg import Bool

from PIL import Image
from nav_msgs.msg import Path
from graspnetAPI import GraspGroup
from realsense_service.srv import *
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from open3d_ros_helper import open3d_ros_helper as orh

ROOT_DIR = '/graspnet/graspnet-baseline/'  # path to graspnet-baseline
sys.path.append(os.path.join(ROOT_DIR, 'models'))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))
SCRIPT_DIR = os.path.normpath(__file__ + os.sep + os.pardir)

from graspnet import GraspNet, pred_decode
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image

parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint_path', default=SCRIPT_DIR+'/checkpoint-rs.tar', help='Model checkpoint path')
parser.add_argument('--num_point', type=int, default=20000, help='Point Number [default: 20000]')
parser.add_argument('--num_view', type=int, default=300, help='View Number [default: 300]')
parser.add_argument('--collision_thresh', type=float, default=0.01, help='Collision Threshold in collision detection [default: 0.01]')
parser.add_argument('--voxel_size', type=float, default=0.01, help='Voxel Size to process point clouds before collision detection [default: 0.01]')
parser.add_argument('--cam_width', type=int, default=1280)
parser.add_argument('--cam_height', type=int, default=720)
parser.add_argument('--vis', help="Visualize result", action="store_true")
parser.add_argument('--score', help="Score threshold", type=float, default=0.0)

cfgs = parser.parse_args()

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

def main():
    global new_msg

    if not rospy.is_shutdown():

        print('Starting...')
        rospy.init_node('graspnet', anonymous=True)
        pub = rospy.Publisher('grasps', Path, queue_size=10)
        rospy.Subscriber("start_graspnet", Bool, sub_callback)
        topic_name="kinect2/qhd/points"
        pub_scene = rospy.Publisher("/grasping/scene_pointcloud", PointCloud2, queue_size=1)
        rate = rospy.Rate(5)

        print('Loading network...')
        net = get_net()

        while not rospy.is_shutdown():

            new_msg = False
            print('Waiting for go...')
            while not new_msg and not rospy.is_shutdown():
                rate.sleep()

            print('Processing data...')
            end_points, cloud = get_and_process_data()

            print('Processing image through graspnet...')
            gg = get_grasps(net, end_points)
            if cfgs.collision_thresh > 0:
                gg = collision_detection(gg, np.array(cloud.points))
            gg.nms()
            gg = remove_grasps_under_score(gg, cfgs.score) #  Score range between 0 and 2. Under 0.1 bad, over 0.7 good

            msg = generate_ros_message(gg, nr_of_grasps=0)  # nr_grasps = 0 is use all grasps

            pub.publish(msg)

            #ros_cloud = convertCloudFromOpen3dToRos(cloud)
            ros_cloud = orh.o3dpc_to_rospc(o3dpc, frame_id="ptu_camera_color_optical_frame", stamp=rospy.Time.now())
            pub_scene.publish(ros_cloud)

            #vis_grasps(gg, cloud, nr_to_visualize=0)  # nr_to_vizualize = 0 is show all
            #grasps = gg[:nr_to_visualize]
            #if nr_to_visualize == 0:
            #    grasps = gg
            #grippers = grasps.to_open3d_geometry_list()
            #o3d.visualization.draw_geometries([cloud, *grippers])




def sub_callback(data):
    global new_msg
    print('Received message')
    new_msg = True


def remove_grasps_under_score(gg, score_thresh):
    gg.sort_by_score()
    grasp_nr = 0
    for scores in gg.scores:
        if scores < score_thresh:
            gg = gg[:grasp_nr]
        grasp_nr += 1
    return gg


def generate_ros_message(gg, nr_of_grasps):
    grasps = gg[:nr_of_grasps]
    if nr_of_grasps == 0:
        grasps = gg

    grasps_msg = Path()
    grasps_msg.header.stamp = rospy.Time.now()
    grasps_msg.header.frame_id = "Header"

    seq = 0
    for grap in grasps:

        grasp = PoseStamped()
        grasp.header.stamp = rospy.Time.now()
        grasp.header.frame_id = str(grap.score)
        grasp.header.seq = seq
        seq += 1

        grasp.pose.position.x = grap.translation[0]
        grasp.pose.position.y = grap.translation[1]
        grasp.pose.position.z = grap.translation[2]

        rot = Rotation.from_matrix(grap.rotation_matrix)
        quat = rot.as_quat()
        grasp.pose.orientation.x = quat[0]
        grasp.pose.orientation.y = quat[1]
        grasp.pose.orientation.z = quat[2]
        grasp.pose.orientation.w = quat[3]

        grasps_msg.poses.append(grasp)

    return grasps_msg


def get_net():
    # Init the model
    net = GraspNet(input_feature_dim=0, num_view=cfgs.num_view, num_angle=12, num_depth=4,
             cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01, 0.02, 0.03, 0.04], is_training=False)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    #device = torch.device("cpu")
    net.to(device)
    # Load checkpoint
    checkpoint = torch.load(cfgs.checkpoint_path)
    net.load_state_dict(checkpoint['model_state_dict'])
    start_epoch = checkpoint['epoch']
    print("-> loaded checkpoint %s (epoch: %d)"%(cfgs.checkpoint_path, start_epoch))
    # set model to eval mode
    net.eval()
    return net


def get_and_process_data():

    workspace_mask = np.array(Image.open(SCRIPT_DIR+'/workspace_mask.png'))

    print('Get data from realsense...')
    captureRealsense()
    color_img = np.array(getRGB()) / 255
    depth_img = getDepth()
    print('Generate pointcloud...')
    # generate cloud
    fx = 637.598
    fy = 637.598
    ppx = 646.101
    ppy = 360.912
    camera = CameraInfo(1280.0, 720.0, fx, fy, ppx, ppy, 1000)
    cloud = create_point_cloud_from_depth_image(depth_img, camera, organized=True)

    # get valid points
    mask = (workspace_mask & (depth_img < 1000))
    cloud_masked = cloud[mask]
    color_masked = color_img[mask]

    # sample points
    if len(cloud_masked) >= cfgs.num_point:
        idxs = np.random.choice(len(cloud_masked), cfgs.num_point, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked), cfgs.num_point-len(cloud_masked), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    # convert data
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
    end_points = dict()
    cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    #device = torch.device("cpu")
    cloud_sampled = cloud_sampled.to(device)
    end_points['point_clouds'] = cloud_sampled
    end_points['cloud_colors'] = color_sampled

    return end_points, cloud


def get_grasps(net, end_points):
    # Forward pass
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)
    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)
    return gg


def collision_detection(gg, cloud):
    mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=cfgs.voxel_size)
    collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=cfgs.collision_thresh)
    gg = gg[~collision_mask]
    return gg


def vis_grasps(gg, cloud, nr_to_visualize):
    grasps = gg[:nr_to_visualize]
    if nr_to_visualize == 0:
        grasps = gg
    grippers = grasps.to_open3d_geometry_list()
    o3d.visualization.draw_geometries([cloud, *grippers])


def captureRealsense():
    rospy.wait_for_service("/sensors/realsense/capture")
    captureService = rospy.ServiceProxy("/sensors/realsense/capture", capture)
    msg = capture()
    msg.data = True
    response = captureService(msg)
    print(response)


def getRGB():
    rospy.wait_for_service("/sensors/realsense/rgb")
    rgbService = rospy.ServiceProxy("/sensors/realsense/rgb", rgb)
    msg = rgb()
    msg.data = True
    response = rgbService(msg)
    img = np.frombuffer(response.img.data, dtype=np.uint8).reshape(response.img.height, response.img.width, -1)
    return img


def getDepth():
    rospy.wait_for_service("/sensors/realsense/depth")
    depthService = rospy.ServiceProxy("/sensors/realsense/depth", depth)
    msg = depth()
    msg.data = True
    response = depthService(msg)
    w = response.width.data
    h = response.height.data
    depthImg = np.asarray(response.img.data).reshape((h, w)) # use this depth image
    return depthImg

def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="odom"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "ptu_camera_color_optical_frame"

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors: # XYZ only
        fields=FIELDS_XYZ
        cloud_data=points


    else: # XYZ + RGB
        fields=FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
        #colors = np.floor(np.asarray(open3d_cloud.colors)*255)
        colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]
        print(colors)
        print(points)
        cloud_data=np.c_[points, colors]
        print(cloud_data)


    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)

def convertCloudFromRosToOpen3d(ros_cloud):

    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = open3d.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb

        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = open3d.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = open3d.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud

if __name__ == '__main__':
    main()
