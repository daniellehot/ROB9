#!/usr/bin/env python3
import sys
import copy
import rospy
import math
import numpy as np
import open3d as o3d
import cv2

## ROS

import moveit_msgs
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
import std_msgs.msg
from std_msgs.msg import Int8, MultiArrayDimension, MultiArrayLayout, Int32MultiArray, Float32MultiArray, Bool, Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

## ROB9

import rob9Utils.transformations as transform
from rob9Utils.graspGroup import GraspGroup
from rob9Utils.grasp import Grasp, Orientation, Position
import rob9Utils.moveit as moveit
from cameraService.cameraClient import CameraClient
from affordanceService.client import AffordanceClient
from grasp_service.client import GraspingGeneratorClient
from rob9Utils.visualize import visualizeGrasps6DOF
import rob9Utils.visualize as viz

from moveit_scripts.srv import *
from moveit_scripts.msg import *
from grasp_aff_association.srv import *
from rob9.srv import graspGroupSrv, graspGroupSrvResponse

reset_gripper_msg = std_msgs.msg.Int8()
reset_gripper_msg.data = 0
activate_gripper_msg = std_msgs.msg.Int8()
activate_gripper_msg.data = 1
close_gripper_msg = std_msgs.msg.Int8()
close_gripper_msg = 2
open_gripper_msg = std_msgs.msg.Int8()
open_gripper_msg.data = 3
basic_gripper_msg = std_msgs.msg.Int8()
basic_gripper_msg.data = 4
pinch_gripper_msg = std_msgs.msg.Int8()
pinch_gripper_msg.data = 5

class Object(object):
    """docstring for SceneObject, which are an abstraction of objects found in
    the scene."""

    def __init__(self):
        self.tool_id = None
        self.object_instance = None
        self.affordance_ids = []
        self.affordanceClouds = []
        self.grasps = []

        self.orientation = None
        self.position = None

    def addAffordance(self, id):
        self.affordance_ids.append(id)

    def addAffordanceCloud(self, aCloud):
        self.affordanceClouds.append(aCloud)

    def setObjectInstance(self, instance):
        self.object_instance = int(instance)

    def setToolId(self, id):
        self.tool_id = int(id)

class AffordanceCloud(object):
    """docstring for AffordanceCloud."""

    def __init__(self, points, bbox, position, affordance_id, orientation = [0, 0, 0, 1]):
        """ input:  points  -   numpy array (no_points, xyz)
                    bbox    -   oriented bounding box (8, xyz)
                    position  -   numpy array (x, y, z)
                    affordance_id   -   id [1 : 9]
                    orientation -   list or numpy array in quaternion [x, y, z, w]
        """

        self.points = points
        self.bbox = bbox
        self.position = Position(position[0], position[1], position[2])
        self.affordance_id = affordance_id
        self.orientation = Orientation(qw = orientation[3], qx = orientation[0], qy = orientaiton[1], qz = orientation[2])


def associateGraspAffordance(graspData, objects, masks, cloud, cloud_uv, demo = False):

    graspMsg = graspData.toGraspGroupMsg()

    objectMsg = Int32MultiArray()
    objectMsg.data = objects.tolist()

    intToLabel = {0: 'class', 1: 'height', 2: 'width'}
    maskMsg = Int32MultiArray()

    masks = np.reshape(masks, (-1, masks.shape[2], masks.shape[3]))

    # constructing mask message
    for i in range(3):
        dimMsg = MultiArrayDimension()
        dimMsg.label = intToLabel[i]
        stride = 1
        for j in range(3-i):
            stride = stride * masks.shape[i+j]
        dimMsg.stride = stride
        dimMsg.size = masks.shape[i]
        maskMsg.layout.dim.append(dimMsg)
    maskMsg.data = masks.flatten().astype(int).tolist()

    demoMsg = Bool()
    demoMsg.data = demo

    uvDim1 = MultiArrayDimension()
    uvDim1.label = "length"
    uvDim1.size = int(cloud_uv.shape[0] * cloud_uv.shape[1])
    uvDim1.stride = cloud_uv.shape[0]

    uvDim2 = MultiArrayDimension()
    uvDim2.label = "pair"
    uvDim2.size = cloud_uv.shape[1]
    uvDim2.stride = cloud_uv.shape[1]

    uvLayout = MultiArrayLayout()
    uvLayout.dim.append(uvDim1)
    uvLayout.dim.append(uvDim2)

    uvMsg = Float32MultiArray()
    uvMsg.data = cloud_uv.flatten().tolist()
    uvMsg.layout = uvLayout

    FIELDS_XYZ = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "ptu_camera_color_optical_frame"
    cloudMsg = pc2.create_cloud(header, FIELDS_XYZ, cloud)

    rospy.wait_for_service('/grasp_affordance_association/associate')
    get_grasps_service = rospy.ServiceProxy('/grasp_affordance_association/associate', graspGroupSrv)
    response = get_grasps_service(demoMsg, graspMsg, objectMsg, maskMsg, cloudMsg, uvMsg)

    return GraspGroup().fromGraspGroupSrv(response)



def send_trajectory_to_rviz(plan):
    print("Trajectory was sent to RViZ")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory_start = moveit.getCurrentState()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)

def computeWaypoints(graspObjects, offset = 0.1):
    """ input:  graspsObjects   -   GraspGroup() of grasps
                offset          -   float, in meters for waypoint in relation to grasp
        output:                 -   nav_msgs Path
    """

    world_frame = "world"
    ee_frame = "right_ee_link"

    grasps, waypoints = [], []
    for i in range(len(graspObjects)):

        grasp = graspObjects[i]

        graspCamera = copy.deepcopy(grasp)
        waypointCamera = copy.deepcopy(grasp)

        # computing waypoint in camera frame
        rotMat = graspCamera.getRotationMatrix()
        offsetArr = np.array([[offset], [0.0], [0.0]])
        offsetCam = np.transpose(np.matmul(rotMat, offsetArr))[0]

        waypointCamera.position.x += -offsetCam[0]
        waypointCamera.position.y += -offsetCam[1]
        waypointCamera.position.z += -offsetCam[2]

        waypointWorld = Grasp().fromPoseStampedMsg(transform.transformToFrame(waypointCamera.toPoseStampedMsg(), world_frame))
        graspWorld = Grasp().fromPoseStampedMsg(transform.transformToFrame(graspCamera.toPoseStampedMsg(), world_frame))

        waypointWorld.frame_id = str(0) # we should probably do away with storing it in the header
        graspWorld.frame_id = str(0)
        waypoints.append(waypointWorld.toPoseStampedMsg())
        grasps.append(graspWorld.toPoseStampedMsg())
        print(i+1, " / ", len(graspObjects))

    grasps_msg = nav_msgs.msg.Path()
    grasps_msg.header.frame_id = "world"
    grasps_msg.header.stamp = rospy.Time.now()
    for i in range(len(grasps)):
        grasps_msg.poses.append(waypoints[i])
        grasps_msg.poses.append(grasps[i])

    return grasps_msg

def filterBySphericalCoordinates(poses, azimuth, polar):
    """ input:  poses   -   nav_msgs/Path, a list of waypoints and poses in pairs
                azimut  -   numpy array shape 2, [minAngle, maxAngle]
                polar   -   numpy array shape 2, [minAngle, maxAngle]
        output:         -   nav_msgs/Path a list of waypoints and poses in pairs
        Only returns waypoints and grasps that are inside the spherical angle
        limits
    """
    grasps, waypoints = [], []
    for i in range(int(len(poses.poses) / 2)):

        waypointWorld = poses.poses[i]
        graspWorld = poses.poses[i + 1]

        # computing local cartesian coordinates
        x = waypointWorld.pose.position.x - graspWorld.pose.position.x
        y = waypointWorld.pose.position.y - graspWorld.pose.position.y
        z = waypointWorld.pose.position.z - graspWorld.pose.position.z

        # computing spherical coordinates
        r, polarAngle, azimuthAngle = transform.cartesianToSpherical(x, y, z)

        azimuthAngleLimit = azimuth
        polarAngleLimit = polar

        # Evaluating angle limits
        if azimuthAngle > azimuthAngleLimit[0] and azimuthAngle < azimuthAngleLimit[1]:
            if polarAngle > polarAngleLimit[0] and polarAngle < polarAngleLimit[1]:
                waypoints.append(waypointWorld)
                grasps.append(graspWorld)


    if len(grasps) == 0 or len(waypoints) == 0:
        print("Could not find grasp with appropriate angle")
        grasp_msg = nav_msgs.msg.Path()
        return grasps_msg # in case no grasps can be found, return empty message

    grasps_msg = nav_msgs.msg.Path()
    grasps_msg.header.frame_id = "world"
    grasps_msg.header.stamp = rospy.Time.now()
    for i in range(len(grasps)):
        grasps_msg.poses.append(waypoints[i])
        grasps_msg.poses.append(grasps[i])

    return grasps_msg

def sortByOrientationDifference(poses):
    # Should be moved to GraspGroup.py
    # not yet implemented! this is the old deltaRPY
    """ input:  poses   -   nav_msgs/Path, a list of waypoints and grasps in pairs
        output:         -   nav_msgs/Path, a list of waypoints and grasps in pairs
        Sorted by the relative angle difference compared to current robot pose
    """

    eeWorld = tf_buffer.lookup_transform("world", "right_ee_link", rospy.Time.now(), rospy.Duration(1.0))
    weightedSums = []

    for i in range(len(grasps)):
        deltaRPY = abs(transform.delta_orientation(grasps[i], eeWorld))
        weightedSum = 0.2*deltaRPY[0]+0.4*deltaRPY[1]+0.4*deltaRPY[2]
        weightedSums.append(weightedSum)

    weightedSums_sorted = sorted(weightedSums)
    grasps_sorted = [None]*len(grasps) # sorted according to delta orientation from current orientation of gripper
    waypoints_sorted = [None]*len(waypoints)

    for i in range(len(weightedSums)):
        num = weightedSums_sorted[i]
        index = weightedSums.index(num)
        grasps_sorted[i] = grasps[index]
        waypoints_sorted[i] = waypoints[index]

if __name__ == '__main__':

    demo = std_msgs.msg.Bool()
    demo.data = False
    if len(sys.argv) > 1:
        if sys.argv[1] == 'demo':
            demo.data = True
            print("Demo = True")
        else:
            print("Invalid input argument")
            exit()

    rospy.init_node('experiment_grasping_node', anonymous=True)

    gripper_pub = rospy.Publisher('gripper_controller', Int8, queue_size=10, latch=True)

    # publishers for RVIZ visualization
    pub_grasp = rospy.Publisher('pose_to_reach', PoseStamped, queue_size=10)
    pub_waypoint = rospy.Publisher('pose_to_reach_waypoint', PoseStamped, queue_size=10)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,

                                                   queue_size=20)
    # DO NOT REMOVE THIS SLEEP, it allows gripper_pub to establish connection to the topic
    rospy.sleep(2)

    # Reset the gripper
    gripper_pub.publish(reset_gripper_msg)
    gripper_pub.publish(activate_gripper_msg)
    gripper_pub.publish(open_gripper_msg)
    gripper_pub.publish(pinch_gripper_msg)

    rospy.sleep(3)

    print("Camera is capturing new scene")

    cam = CameraClient()
    cam.captureNewScene()
    cloud, cloudColor = cam.getPointCloudStatic()
    cloud_uv = cam.getUvStatic()
    img = cam.getRGB()

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud)
    pcd.colors = o3d.utility.Vector3dVector(cloudColor)

    print("Generating grasps")

    # Get grasps from grasp generator
    graspClient = GraspingGeneratorClient()

    collision_thresh = 0.01 # Collision threshold in collision detection
    num_view = 300 # View number
    score_thresh = 0.0 # Remove every grasp with scores less than threshold
    voxel_size = 0.2

    graspClient.setSettings(collision_thresh, num_view, score_thresh, voxel_size)

    # Load the network with GPU (True or False) or CPU
    #graspClient.start(GPU=True)

    graspData = graspClient.getGrasps()

    print("Got ", len(graspData), " grasps")

    print("Segmenting affordance maps")
    # Run affordance analyzer
    affClient = AffordanceClient()

    #affClient.start(GPU=False)
    #_ = affClient.run(img, CONF_THRESHOLD = 0.5)

    _, _, _, _ = affClient.getAffordanceResult()

    _ = affClient.processMasks(conf_threshold = 40, erode_kernel=(15,15))

    # Visualize object detection and affordance segmentation to confirm
    img = affClient.visualizeMasks(img)
    #cv2.imshow("BBox's", affClient.visualizeBBox(img))
    #cv2.waitKey(0)

    masks = affClient.masks
    objects = affClient.objects

    # compute object orientation here

    objectsInScene = []
    object_instance = 1
    for obj_idx, obj_id in enumerate(objects):

        object = Object()
        object.setToolId(obj_id)
        object.setObjectInstance(object_instance)

        affordance_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9]

        for affordance_id in affordance_ids:

            ## mask point cloud
            cloud_masked_points = []
            for k, uv in enumerate(cloud_uv):
                if masks[obj_idx, affordance_id, uv[0], uv[1]] == 255:
                    cloud_masked_points.append(cloud[k])
            cloud_masked_points = np.array(cloud_masked_points)

            ## compute oriented bounding box

            if cloud_masked_points.shape[0] >= 15:

                object.addAffordance(affordance_id)

                pcd_masked = o3d.geometry.PointCloud()
                pcd_masked.points = o3d.utility.Vector3dVector(cloud_masked_points)
                pcd_masked, _ = pcd_masked.remove_statistical_outlier(nb_neighbors=8, std_ratio=1.0)
                bbox3d = o3d.geometry.OrientedBoundingBox.create_from_points(points=pcd_masked.points)

                box_corners = bbox3d.get_box_points()
                currentFrame = "ptu_camera_color_optical_frame"
                if demo.data:
                    currentFrame = "ptu_camera_color_optical_frame_real"

                box_corners = np.asarray(box_corners)
                box_center = np.asarray(bbox3d.get_center())

                affordanceCloud = AffordanceCloud(np.asanyarray(pcd_masked.points), box_corners, box_center, affordance_id)
                object.addAffordanceCloud(copy.deepcopy(affordanceCloud))

        objectsInScene.append(copy.deepcopy(object))

        object_instance += 1

    for object in objectsInScene:

        ## find 4 points closest to other affordance
        if 5 in object.affordance_ids:
            grasp_idx = object.affordance_ids.index(5)
            non_grasp_affordances_idx = []
            for idx, affordance_id in enumerate(object.affordance_ids):
                if affordance_id != 5:
                    non_grasp_affordances_idx.append(idx)

        meanCenter = np.zeros(3)
        for idx in non_grasp_affordances_idx:
            meanCenter += object.affordanceClouds[idx].position.getVector(format = "row")

        meanCenter = meanCenter / len(non_grasp_affordances_idx)

        bbox_corner_dist = []
        for idx, point in enumerate(object.affordanceClouds[grasp_idx].bbox):
            bbox_corner_dist.append([idx, np.linalg.norm(np.array(meanCenter) - np.array(point))])

        # sort computed distances in ascending order
        bbox_corner_dist = sorted(bbox_corner_dist, key = lambda bbox_corner_dist:bbox_corner_dist[1])

        ## compute surface normal
        xO = object.affordanceClouds[grasp_idx].bbox[bbox_corner_dist[0][0]]
        xA = object.affordanceClouds[grasp_idx].bbox[bbox_corner_dist[1][0]]
        xB = object.affordanceClouds[grasp_idx].bbox[bbox_corner_dist[2][0]]
        xC = object.affordanceClouds[grasp_idx].bbox[bbox_corner_dist[3][0]]


        p = []
        p.append(xO)
        p.append(xA)
        p.append(xB)
        p.append(xC)
        #p.append(object.affordanceClouds[grasp_idx].bbox[bbox_corner_dist[4][0]])
        #p.append(object.affordanceClouds[grasp_idx].bbox[bbox_corner_dist[5][0]])
        #p.append(object.affordanceClouds[grasp_idx].bbox[bbox_corner_dist[6][0]])
        #p.append(object.affordanceClouds[grasp_idx].bbox[bbox_corner_dist[7][0]])
        c = np.array([1, 0, 0])
        colorVis = []
        for s in range(len(p)):
            colorVis.append(c)


        pcd0 = o3d.geometry.PointCloud()
        pcd0.points = o3d.utility.Vector3dVector(p)
        pcd0.colors = o3d.utility.Vector3dVector(colorVis)
        #o3d.visualization.draw_geometries([pcd, pcd0])


        OA = xO - xA
        OB = xO - xB

        xAxis = np.cross(OA, OB) # cross product is orthogonal to OA,OB plane
        xAxis = xAxis / np.linalg.norm(xAxis) # normalize xAxis vector

        ## compute z axis which we want to align with the z axis of the world frame.
        bbox = copy.deepcopy(object.affordanceClouds[grasp_idx].bbox)
        for count, corner in enumerate(bbox):

            corner_msg = transform.transformToFrame(corner, "world", currentFrame = currentFrame)
            bbox[count] = np.array([corner_msg.pose.position.x, corner_msg.pose.position.y, corner_msg.pose.position.z])
        bbox_sort_by_z = sorted(bbox, key = lambda bbox:bbox[2], reverse=True)
        for count, corner in enumerate(bbox_sort_by_z):

            corner_msg = transform.transformToFrame(corner, currentFrame, currentFrame = "world")
            bbox_sort_by_z[count] = np.array([corner_msg.pose.position.x, corner_msg.pose.position.y, corner_msg.pose.position.z])

        zO = bbox_sort_by_z[0]
        zA = bbox_sort_by_z[1]
        zB = bbox_sort_by_z[2]
        zC = bbox_sort_by_z[3]

        p = []
        p.append(zO)
        p.append(zA)
        p.append(zB)
        p.append(zC)
        c = np.array([0, 0, 1])
        colorVis = []
        for s in range(len(p)):
            colorVis.append(c)


        pcd1 = o3d.geometry.PointCloud()
        pcd1.points = o3d.utility.Vector3dVector(p)
        pcd1.colors = o3d.utility.Vector3dVector(colorVis)
        o3d.visualization.draw_geometries([pcd, pcd0, pcd1])

        OA = zO - zA
        OB = zO - zB


        zAxis = np.cross(OA, OB) # cross product is orthogonal to OA,OB plane
        zAxis = zAxis / np.linalg.norm(zAxis) # normalize zAxis vector

        yAxis = np.cross(xAxis, zAxis)
        yAxis = yAxis / np.linalg.norm(yAxis)

        xAxis = np.reshape(xAxis, (3, 1))
        yAxis = np.reshape(yAxis, (3, 1))
        zAxis = np.reshape(zAxis, (3, 1))

        frame = viz.visualizeFrame(xAxis, yAxis, zAxis, object.affordanceClouds[grasp_idx].position.getVector(format = "row"))
        o3d.visualization.draw_geometries([pcd, frame])
        print(yAxis)
        print(yAxis.shape)

        rotMat = np.hstack((xAxis, yAxis, zAxis))
        quaternion = transform.quaternionFromRotation(rotMat)
        object.affordanceClouds[grasp_idx].orientation.setQuaternion(quaternion)



    print("Computing task oriented grasps")

    # Associate affordances with grasps
    grasps_affordance = associateGraspAffordance(graspData, objects, masks, cloud, cloud_uv, demo = demo.data)

    # Visualize grasps in point cloud
    visualizeGrasps6DOF(pcd, grasps_affordance)

    ## compute orientation difference between current object orientation and desired handover orientation

    grasps_affordance.sortByScore()
    grasp_waypoints_path = computeWaypoints(grasps_affordance, offset = 0.1)

    # Filter out grasps outside spherical angle limits [min, max]
    azimuthAngleLimit = [-1*math.pi, 1*math.pi]
    polarAngleLimit = [0, 0.5*math.pi]
    grasp_waypoints_path = filterBySphericalCoordinates(grasp_waypoints_path, azimuth = azimuthAngleLimit, polar = polarAngleLimit)

    # Visualize grasps in point cloud
    #visualizeGrasps6DOF(pcd, GraspGroup().fromPath(grasp_waypoints_path))

    print("Computing valid trajectories for grasps...")
    rospy.wait_for_service('get_trajectories')
    get_trajectories = rospy.ServiceProxy('get_trajectories', GetTrajectories)
    resp_trajectories = get_trajectories(grasp_waypoints_path)

    id = str(0)
    plans, goal_poses = [], []

    for i in range(len(resp_trajectories.trajectories.trajectories)):
        if resp_trajectories.trajectories.trajectories[i].joint_trajectory.header.frame_id == id:
            plans.append(resp_trajectories.trajectories.trajectories[i])

    for i in range(len(resp_trajectories.trajectories_poses.poses)):
        if resp_trajectories.trajectories_poses.poses[i].header.frame_id == id:
            goal_poses.append(resp_trajectories.trajectories_poses.poses[i])

    waypoint_msg = geometry_msgs.msg.PoseStamped()
    waypoint_msg.header.frame_id = "world"
    waypoint_msg.header.stamp = rospy.Time.now()
    waypoint_msg.pose = goal_poses[0].pose
    pub_waypoint.publish(waypoint_msg) # visualize waypoint in RVIZ

    goal_msg = geometry_msgs.msg.PoseStamped()
    goal_msg.header.frame_id = "world"
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.pose = goal_poses[1].pose
    pub_grasp.publish(goal_msg) # visualize grasp pose in RVIZ

    input("Press Enter when you are ready to move the robot")
    for i in range(3):
        send_trajectory_to_rviz(plans[i])
        moveit.execute(plans[i])
        if i == 1:
            gripper_pub.publish(close_gripper_msg)
            rospy.sleep(1)
        print("I have grasped!")

    input("Press Enter when you are ready to move the robot to the handover pose")
    moveit.moveToNamed("ready")
    moveit.moveToNamed("handover")
    input("Press Enter when you are ready to move the robot back to the ready pose")

    moveit.moveToNamed("ready")
    gripper_pub.publish(open_gripper_msg)

    print("Done")
