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
from std_msgs.msg import Header, Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
#from open3d_ros_helper import open3d_ros_helper as orh

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

def main():
    global new_msg, cloud, rgb
    cloud = None
    rgb = None

    if not rospy.is_shutdown():

        print('Starting...')
        rospy.init_node('graspnet', anonymous=True)
        pub = rospy.Publisher('grasps', Path, queue_size=10)
        rospy.Subscriber("start_graspnet", Bool, sub_callback)
        cloudSubscriber = rospy.Subscriber("/sensors/realsense/pointcloudGeometry/static", PointCloud2, callbackPointCloud)
        rgbSubscriber = rospy.Subscriber("/sensors/realsense/pointcloudGeometry/static/rgb", Float32MultiArray, callbackRGB)
        topic_name="kinect2/qhd/points"
        rate = rospy.Rate(5)


        print('Loading network...')
        net = get_net()
        new_msg = False

        while not rospy.is_shutdown():


            print('Waiting for go...')
            while not new_msg and not rospy.is_shutdown():
                rate.sleep()

            print('Processing data...')
            rate.sleep()
            if cloud is None or rgb is None:
                pass
            else:
                end_points = get_and_process_data(cloud, rgb)

                print('Processing image through graspnet...')
                gg = get_grasps(net, end_points)
                if cfgs.collision_thresh > 0:
                    gg = collision_detection(gg, np.array(cloud.points))
                gg.nms()
                gg = remove_grasps_under_score(gg, cfgs.score) #  Score range between 0 and 2. Under 0.1 bad, over 0.7 good

                msg = generate_ros_message(gg, nr_of_grasps=0)  # nr_grasps = 0 is use all grasps

                pub.publish(msg)
                new_msg = False
                cloud = None
                rgb = None

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


def get_and_process_data(cloud, rgb):
    workspace_mask = np.array(Image.open(SCRIPT_DIR+'/workspace_mask.png'))

    print('Get data from realsense...')
    #color_img = np.array(getRGB()) / 255
    #depth_img = getDepth()

    # convert data
    #cloud = o3d.geometry.PointCloud()
    #cloud.points = o3d.utility.Vector3dVector(points.astype(np.float32))
    cloud.colors = o3d.utility.Vector3dVector(rgb.astype(np.float32))
    end_points = dict()
    cloud_sampled = np.asarray(cloud.points)
    color_sampled = np.asarray(cloud.colors)
    #cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
    cloud_sampled = torch.from_numpy(cloud_sampled.astype(np.float32))
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    cloud_sampled = cloud_sampled.to(device)
    end_points['point_clouds'] = cloud_sampled
    end_points['cloud_colors'] = color_sampled

    return end_points


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

def callbackPointCloud(data):
    global cloud
    cloud = convertCloudFromRosToOpen3d(data)
    print("Got geometry")

def callbackRGB(msg):
    global rgb
    rgb = np.asarray(msg.data)
    rgb = np.reshape(rgb, (-1,3))
    print("Got rgb")

def convertCloudFromRosToOpen3d(ros_cloud):

    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
    xyz = np.array(xyz)
    xyz = xyz[xyz[:,2] < 1.0]
    open3d_cloud.points = o3d.utility.Vector3dVector(xyz)

    # return
    return open3d_cloud

if __name__ == '__main__':
    main()
