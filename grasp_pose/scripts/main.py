#!/usr/bin/env python2.7
import sys
import rospy
import std_msgs.msg
from affordancenet_service.srv import *
from realsense_service.srv import *
import numpy as np
import cv2
from std_msgs.msg import Bool
from nav_msgs.msg import Path
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_multiply
import copy
import math
from sensor_msgs.msg import PointCloud2
import open3d as o3d


def captureNewScene():
    rospy.wait_for_service("/sensors/realsense/capture")
    captureService = rospy.ServiceProxy("/sensors/realsense/capture", capture)
    msg = capture()
    msg.data = True
    response = captureService(msg)
    print(response)


def getAffordanceResult():
    rospy.wait_for_service("/affordanceNet/result")
    affordanceNetService = rospy.ServiceProxy("/affordanceNet/result", affordance)
    msg = affordance()
    msg.data = True
    response = affordanceNetService(msg)

    no_objects = int(response.masks.layout.dim[0].size / 10)
    masks = np.asarray(response.masks.data).reshape((no_objects, int(response.masks.layout.dim[0].size / no_objects), response.masks.layout.dim[1].size, response.masks.layout.dim[2].size)) #* 255
    masks = masks.astype(np.uint8)

    bbox = np.asarray(response.bbox.data).reshape((-1,4))

    objects = np.asarray(response.object.data)
    print(masks.shape)
    print(masks.dtype)
    print(bbox)
    print(objects)
    for j in range(3):
        for i in range(10):
            print(j,i)
            cv2.imshow("title", masks[j][i])
            cv2.waitKey(1000)
    return masks, bbox, objects


def grasp_callback(data):
    global new_grasps, grasp_data
    print('Recieved grasps')
    grasp_data = data
    new_grasps = True


def run_graspnet(pub):
    print('Send start to graspnet')
    graspnet_msg = Bool()
    graspnet_msg.data = True
    pub.publish(graspnet_msg)


def transformFrame(tf_buffer, pose, orignalFrame, newFrame):
    transformed_pose_msg = geometry_msgs.msg.PoseStamped()
    tf_buffer.lookup_transform(orignalFrame, newFrame, rospy.Time.now(), rospy.Duration(1.0))
    transformed_pose_msg = tf_buffer.transform(pose, newFrame)
    return transformed_pose_msg


def cartesianToSpherical(x, y, z):

    polar = math.atan2(math.sqrt(x**2 + y**2), z)
    azimuth = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2 + z**2)

    return r, polar, azimuth

def calculate_delta_orientation(graspWorld, eeWorld):
    graspWorldQuaternion = (
        graspWorld.pose.orientation.x,
        graspWorld.pose.orientation.y,
        graspWorld.pose.orientation.z,
        graspWorld.pose.orientation.w)
    graspWorldRPY = np.asarray(euler_from_quaternion(graspWorldQuaternion)) * 180 / math.pi

    eeWorldQuaternionInv = (
        eeWorld.transform.rotation.x,
        eeWorld.transform.rotation.y,
        eeWorld.transform.rotation.z,
        -eeWorld.transform.rotation.w)

    deltaQuaternion = quaternion_multiply(graspWorldQuaternion, eeWorldQuaternionInv)
    deltaRPY = np.asarray(euler_from_quaternion(qr)) * 180 / math.pi
    return deltaRPY


def getUvStatic():
    rospy.wait_for_service("/sensors/realsense/pointcloudGeometry/static/uv")
    uvStaticService = rospy.ServiceProxy("/sensors/realsense/pointcloudGeometry/static/uv", uvSrv)
    msg = uvSrv()

    msg.data = True
    response = uvStaticService(msg)
    rows = int(response.uv.layout.dim[0].size / response.uv.layout.dim[1].size)
    cols = int(response.uv.layout.dim[1].size)

    uvStatic = np.array(response.uv.data).astype(int)
    uvStatic = np.reshape(uvStatic, (rows, cols))

    # y value stored in first indice, x stored in second
    for uv in uvStatic:
        print(uv)

    return uvStatic


def callbackPointCloud(data):
    global cloud
    print("Got pointcloud")
    cloud = convertCloudFromRosToOpen3d(data)


def convertCloudFromRosToOpen3d(ros_cloud):

    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
    xyz = np.array(xyz)
    xyz = xyz[xyz[:,2] < 1.0]

    # return
    return xyz


# Merge list of masks, return as one greyscale image mask
def merge_masks(masks):
    # loop here for all objects
    h, w, c = masks[0].shape
    mask_full = np.full((h, w, c), (0, 0, 0), dtype=np.uint8)
    for mask in masks:
        mask_full = cv2.add(mask_full, mask)

    mask_grey = cv2.cvtColor(mask_full, cv2.COLOR_BGR2GRAY)
    return mask_grey


# Make points ready to visualize in open3d pointcloud with color
def viz_color_pointcloud(points, color):
    colors = [color for i in range(len(points))]
    points = np.array(points)
    pc_viz = o3d.geometry.PointCloud()
    pc_viz.colors = o3d.utility.Vector3dVector(colors)
    pc_viz.points = o3d.utility.Vector3dVector(points.astype(np.float32))
    return pc_viz


# Bound selected points and visualize box in open3d
def viz_boundingbox(point_min, point_max):

    xmin = point_min[0]
    xmax = point_max[0]
    ymin = point_min[1]
    ymax = point_max[1]
    zmin = point_min[2]
    zmax = point_max[2]

    points_to_viz = [[xmin, ymin, zmin], [xmax, ymin, zmin], [xmin, ymax, zmin], [xmax, ymax, zmin],
                      [xmin, ymin, zmax], [xmax, ymin, zmax],
                      [xmin, ymax, zmax], [xmax, ymax, zmax]]
    lines_to_viz = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7],
             [0, 4], [1, 5], [2, 6], [3, 7]]

    colors = [[0, 0, 0] for i in range(len(lines_to_viz))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points_to_viz)
    line_set.lines = o3d.utility.Vector2iVector(lines_to_viz)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set


def main(demo):
    global new_grasps, grasp_data, cloud
    cloud = None
    if not rospy.is_shutdown():
        rospy.init_node('grasp_pose', anonymous=True)
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        rospy.Subscriber("grasps", Path, grasp_callback)
        pub_graspnet = rospy.Publisher('start_graspnet', Bool, queue_size=10)
        pub_grasp = rospy.Publisher('pose_to_reach', PoseStamped, queue_size=10)
        pub_waypoint = rospy.Publisher('pose_to_reach_waypoint', PoseStamped, queue_size=10)
        rate = rospy.Rate(5)

        cloudSubscriber = rospy.Subscriber("/sensors/realsense/pointcloudGeometry/static", PointCloud2, callbackPointCloud)

        # capture a new scene
        print('Capturing new scene...')
        captureNewScene()

        # wait for pointcloud
        print('Waiting for point cloud...')
        while cloud is None:
            rate.sleep
        cloudSubscriber.unregister()

        #make graspnet run on images from realsense
        print('Run graspnet...')
        run_graspnet(pub_graspnet)
        new_grasps = False

        # get affordance result
        print('Getting affordance results...')
        masks, bbox, objects = getAffordanceResult()
        print('Bounding box: ')
        print(bbox)

        # get pointcloud indicies in image
        print('Get uv index for pointcloud...')
        cloud_idx = getUvStatic()

        # Merge masks
        mask_full = merge_masks(masks[0])
        # Subtract unwanted masks -- Change to be index of the grasp mask or whatever need to be removed
        #mask_to_subtract = cv2.cvtColor(masks[0][2], cv2.COLOR_BGR2GRAY)
        #mask_sub = cv2.subtract(mask_full, mask_to_subtract)
        # cv2.imshow('Mask', mask_sub)
        # cv2.waitKey(0)

        # Loop cloud indexes in mask and extract masked pointcloud --ADD bounding box stuff
        cloud_masked = []
        for count, idx in enumerate(cloud_idx):
            # print('Count: '+str(count)+'Index: '+str(idx))
            if mask_full[idx[0]][idx[1]] != 0:
                cloud_masked.append(cloud[count])

        # Vizualize masked pointcloud
        viz_cloud = viz_color_pointcloud(cloud_masked, color=[1, 0, 0])

        # Find min and max points from exstracted pointcloud, could check for outliers
        cloud_masked = np.array(cloud_masked)
        point_min = [np.min(cloud_masked[:, 0]), np.min(cloud_masked[:, 1]), np.min(cloud_masked[:, 2])]
        point_max = [np.max(cloud_masked[:, 0]), np.max(cloud_masked[:, 1]), np.max(cloud_masked[:, 2])]
        # Visualize bbox around point cloud
        bbox_cloud = viz_boundingbox(point_min, point_max)
        # print('Point min and max: ')
        # print(point_min)
        # print(point_max)

        # Find grasps close to area of interest
        search_dist = 5  # Set to value that makes sense
        search_max = [x + search_dist for x in point_max]
        search_min = [x - search_dist for x in point_min]
        # Visualize bbox around search area for nearby grasps
        bbox_search = viz_boundingbox(search_min, search_max)
        # print('Min and max search area for nearby grasps: ')
        # print(search_min)
        # print(search_max)

        exit()

        print('Waiting for grasps from graspnet...')
        while not new_grasps and not rospy.is_shutdown():
            rate.sleep()

        graspData = []

        for i in range(len(grasp_data.poses)):
            grasp = grasp_data.poses[i]
            #if float(grasp.header.frame_id) > 0.2:
            graspData.append(grasp)

        # Find nearby grasps
        grasp_nearby = []
        grasp_nearby_pos = []
        for grasp in grasp_poses:
            pos = [grasp.position.x, grasp.position.y, grasp.position.z]
            if search_min[0] < pos[0] < search_max[0]:
                if search_min[1] < pos[1] < search_max[1]:
                    if search_min[2] < pos[2] < search_max[2]:
                        grasp_nearby.append(grasp)
                        grasp_nearby_pos.append(pos)


        print('ok')
        exit()

        # Evaluating the best grasp.
        world_frame = "world"
        ee_frame = "right_ee_link"
        if demo:
            camera_frame = "ptu_camera_color_optical_frame_real"
        else:
            camera_frame = "ptu_camera_color_optical_frame"

        i, grasps, waypoints = 0, [], []
        while True:

            if i >= len(graspData):
                break
            grasp = graspData[i]
            grasp.header.frame_id = camera_frame

            graspCamera = copy.deepcopy(grasp)
            waypointCamera = copy.deepcopy(grasp)

            # computing waypoint in camera frame
            quaternion = (
                graspCamera.pose.orientation.x,
                graspCamera.pose.orientation.y,
                graspCamera.pose.orientation.z,
                graspCamera.pose.orientation.w)

            rotMat = quaternion_matrix(quaternion)[:3,:3]
            offset = np.array([[0.2], [0.0], [0.0]])
            offset = np.transpose(np.matmul(rotMat, offset))[0]

            waypointCamera.pose.position.x += -offset[0]
            waypointCamera.pose.position.y += -offset[1]
            waypointCamera.pose.position.z += -offset[2]

            # computing waypoint and grasp in world frame

            waypointWorld = transformFrame(tf_buffer, waypointCamera, camera_frame, world_frame)
            graspWorld = transformFrame(tf_buffer, graspCamera, camera_frame, world_frame)

            # computing local cartesian coordinates
            x = waypointWorld.pose.position.x - graspWorld.pose.position.x
            y = waypointWorld.pose.position.y - graspWorld.pose.position.y
            z = waypointWorld.pose.position.z - graspWorld.pose.position.z

            # computing spherical coordinates
            r, polarAngle, azimuthAngle = cartesianToSpherical(x, y, z)

            # Evaluating angle limits
            azimuthAngleLimit = [-0.5*math.pi, -0.25*math.pi]
            polarAngleLimit = [0, 0.5*math.pi]

            azimuthAngleLimit = [-1*math.pi, 1*math.pi]
            polarAngleLimit = [0, 0.5*math.pi]

            if azimuthAngle > azimuthAngleLimit[0] and azimuthAngle < azimuthAngleLimit[1]:
                if polarAngle > polarAngleLimit[0] and polarAngle < polarAngleLimit[1]:

                    waypointWorld.header.stamp = rospy.Time.now()
                    waypointWorld.header.frame_id = world_frame
                    graspWorld.header.stamp = rospy.Time.now()
                    graspWorld.header.frame_id = world_frame

                    waypoints.append(waypointWorld)
                    grasps.append(graspWorld)

                    #pub_waypoint.publish(waypoints[-1])
                    #pub_grasp.publish(grasps[-1])
                    #rospy.sleep(1)

            i += 1

        if len(grasps) == 0 or len(waypoints) == 0:
            print("Could not find grasp with appropriate angle")
        else:
            pub_waypoint.publish(waypoints[0])
            pub_grasp.publish(grasps[0])

        # Affordance segmentation here

        exit()

        # Finding the grasp with the least angle difference
        eeWorld=tf_buffer.lookup_transform("world", "right_ee_link", rospy.Time.now(), rospy.Duration(1.0))

        weightedSums = []

        for i in range(len(grasps)):

            deltaRPY = abs(calculate_delta_orientation(graspWorld, eeWorld))
            weightedSum = 0.2*deltaRPY[0]+0.4*deltaRPY[1]+0.4*deltaRPY[2]
            weightedSums.append(weightedSum)

        minIndex = weightedSums.index(min(weightedSums))
        pub_waypoint.publish(waypoints[min_index])
        pub_grasp.publish(grasps[min_index])

if __name__ == "__main__":
    demo = False
    if len(sys.argv) > 1:
        if sys.argv[1] == 'demo':
            demo = True
        else:
            print("Invalid input argument")
            exit()
    main(demo)
