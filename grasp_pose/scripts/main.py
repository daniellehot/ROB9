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
from geometry_msgs.msg import PoseStamped, PoseArray
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_multiply, quaternion_conjugate, unit_vector
import copy
import math
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2


def captureNewScene():
    rospy.wait_for_service("/sensors/realsense/capture")
    captureService = rospy.ServiceProxy("/sensors/realsense/capture", capture)
    msg = capture()
    msg.data = True
    response = captureService(msg)
    print(response)


def getAffordanceResult(live_run, save=False):
    if live_run:
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
        print('----------Affordance results---------')
        print('Shape: ' + str(masks.shape))
        print('Data type: ' + str(masks.dtype))
        print('Bounding box: ' + str(bbox))
        print('Objects: ' + str(objects))

        if save:
            with open('data/mask_meta.txt', 'w') as f:
                f.write('Shape: ' + str(masks.shape) + '\n')
                f.write('Data type: ' + str(masks.dtype) + '\n')
                f.write('Bounding box: ' + str(bbox) + '\n')
                f.write('Objects: ' + str(objects) + '\n')

            for j in range(len(masks)):
                for i in range(10):
                    title_img = 'data/mask_'+str(j)+'_'+str(i)+'.jpg'
                    cv2.imwrite(title_img, masks[j][i])

        return masks, objects

    else:
        objects = [4]
        masks = []
        for j in range(1):
            mask = []
            for i in range(10):
                title_img = 'data/mask_' + str(j) + '_' + str(i) + '.jpg'
                img = cv2.imread(title_img)
                mask.append(img)
            masks.append(mask)
        return masks, objects


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
        pass
        #print(uv)

    return uvStatic


def callbackPointCloud(data):
    global cloud
    print("Got pointcloud")
    cloud = convertCloudFromRosToOpen3d(data)


def convertCloudFromRosToOpen3d(ros_cloud):

    # Get cloud data from ros_cloud
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    xyz = [(x, y, z) for x, y, z in cloud_data ] # get xyz
    xyz = np.array(xyz)
    xyz = xyz[xyz[:, 2] < 1.0]

    # return
    return xyz


# Merge list of masks, return as one greyscale image mask
def merge_subtract_masks(masks, ids=[], visualize=False):
    # loop here for all objects

    mask_full = np.zeros((masks.shape[1], masks.shape[2])).astype(np.uint8)
    for i in range(1, masks.shape[0]):
        if i in ids:
            print(masks[i].min(), masks[i].max())
            kernel = np.ones((21,21), np.uint8)
            m = np.zeros((masks.shape[1], masks.shape[2])).astype(np.uint8)
            m[masks[i] > 50] = masks[i, masks[i] > 50]
            m = cv2.erode(m, kernel)
            mask_full[m > 0] = m[m>0]
            print(i)
            cv2.imshow('Mask', masks[i])
            cv2.waitKey(0)

        #masks_full[masks[i] != 0]

    #mask_grey = cv2.cvtColor(mask_full, cv2.COLOR_BGR2GRAY)
    mask_grey = mask_full

    #for i in ids:
    #    mask_to_subtract = masks[i]
    #    mask_grey = cv2.subtract(mask_grey, mask_to_subtract)

    if visualize:
        cv2.imshow('Mask', mask_grey)
        cv2.waitKey(0)

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


def inside_cube_test(points , cube3d):
    """
    cube3d  =  numpy array of the shape (8,3) with coordinates in the clockwise order. first the bottom plane is considered then the top one.
    points = array of points with shape (N, 3).

    Returns the indices of the points array which are outside the cube3d
    modified: https://stackoverflow.com/questions/21037241/how-to-determine-a-point-is-inside-or-outside-a-cube
    """
    b1,b2,b4,t1,t3,t4,t2,b3 = cube3d

    dir1 = (t1-b1)
    size1 = np.linalg.norm(dir1)
    dir1 = dir1 / size1

    dir2 = (b2-b1)
    size2 = np.linalg.norm(dir2)
    dir2 = dir2 / size2

    dir3 = (b4-b1)
    size3 = np.linalg.norm(dir3)
    dir3 = dir3 / size3

    cube3d_center = (b1 + t3)/2.0

    dir_vec = points - cube3d_center

    res1 = np.where( (np.absolute(np.dot(dir_vec, dir1)) * 2) > size1 )[0]
    res2 = np.where( (np.absolute(np.dot(dir_vec, dir2)) * 2) > size2 )[0]
    res3 = np.where( (np.absolute(np.dot(dir_vec, dir3)) * 2) > size3 )[0]

    return list( set().union(res1, res2, res3) )


def fuse_grasp_affordance(cloud_masked, grasps_data, visualize=False, search_dist=0.1, vec_length=0.08, steps=20):
    # vizualize and remove outliers
    viz_cloud = viz_color_pointcloud(cloud_masked, [1, 0, 0])
    _, ind = viz_cloud.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)
    viz_cloud = viz_cloud.select_down_sample(ind)

    cloud_masked = np.array(cloud_masked)

    bbox3d = o3d.geometry.OrientedBoundingBox.create_from_points(points=viz_cloud.points)
    box_corners = bbox3d.get_box_points()

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
    search_max = [x + search_dist for x in point_max]
    search_min = [x - search_dist for x in point_min]
    # Visualize bbox around search area for nearby grasps
    bbox_search = viz_boundingbox(search_min, search_max)
    # print('Min and max search area for nearby grasps: ')
    # print(search_min)
    # print(search_max)

    # Find nearby grasps
    grasp_nearby = []
    grasp_nearby_pos = []
    for grasp_pose in grasps_data:
        pos = [grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z]
        if search_min[0] < pos[0] < search_max[0]:
            if search_min[1] < pos[1] < search_max[1]:
                if search_min[2] < pos[2] < search_max[2]:
                    grasp_nearby.append(grasp_pose)
                    grasp_nearby_pos.append(pos)

    # Create and vizualize direction vectors
    viz_grasp_dir = o3d.geometry.LineSet()
    lines_to_viz = []
    points_to_viz = []
    end_points = []
    for i, grasp in enumerate(grasp_nearby):
        point = [grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z]
        quat = [grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z,
                grasp.pose.orientation.w]

        vec = [1, 0, 0]
        direction = qv_mult(quat, vec)
        length = np.linalg.norm(direction)
        direction = direction / length * vec_length
        point2 = [point[0]+direction[0], point[1]+direction[1], point[2]+direction[2]]

        end_points.append(point2)
        points_to_viz.append(point)
        points_to_viz.append(point2)
        lines_to_viz.append([0 + 2 * i, 1 + 2 * i])

    colors = [[1, 0, 0] for i in range(len(lines_to_viz))]
    viz_grasp_dir.points = o3d.utility.Vector3dVector(points_to_viz)
    viz_grasp_dir.lines = o3d.utility.Vector2iVector(lines_to_viz)
    viz_grasp_dir.colors = o3d.utility.Vector3dVector(colors)

    # Visualize grasp points
    viz_grasp_points = viz_color_pointcloud(grasp_nearby_pos, color=[0, 0, 1])

    # Check which grasps point at desired object/affordance
    interpol_points = []
    for point, end_point in zip(grasp_nearby_pos, end_points):
        interpol = []
        x_diff = (point[0] - end_point[0]) / steps
        y_diff = (point[1] - end_point[1]) / steps
        z_diff = (point[2] - end_point[2]) / steps
        for i in range(1, steps + 1):
            interpol.append([point[0] - x_diff * i, point[1] - y_diff * i, point[2] - z_diff * i])
        interpol_points.append(interpol)
    # print('Interpolation points: ')
    # print(interpol_points)

    # Find points in bbox
    good_grasp = []
    interpol_viz_true = []
    interpol_viz_false = []
    for i, point_list in enumerate(interpol_points):
        grasp_intersects = False
        point_list = np.array(point_list)
        outside_bbox = inside_cube_test(point_list, box_corners)  # Returns point indecies outside box

        for idx in range(len(point_list)):
            in_bbox = True
            for index in outside_bbox:
                if idx == index:
                    in_bbox = False

            if in_bbox:
                interpol_viz_true.append(point_list[idx])
                grasp_intersects = True
            else:
                interpol_viz_false.append(point_list[idx])
        if grasp_intersects:
            good_grasp.append(grasp_nearby[i])

    # Visualize points from grasp contained in bbox
    viz_interpol_true = viz_color_pointcloud(interpol_viz_true, color=[0, 1, 0])
    # Visualize points from grasp not contained in bbox
    viz_interpol_false = viz_color_pointcloud(interpol_viz_false, color=[1, 0, 0])

    # Visualize
    if visualize:
        o3d.visualization.draw_geometries([viz_cloud, bbox3d, bbox_search, viz_grasp_dir, viz_grasp_points, viz_interpol_true, viz_interpol_false])

    return good_grasp


def qv_mult(q1, v1):
    v1 = unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return quaternion_multiply(
        quaternion_multiply(q1, q2),
        quaternion_conjugate(q1)
    )[:3]


def main(demo):
    global new_grasps, grasp_data, cloud
    save_data = False  # Should data be saved
    live_run = True  # Are the full system running or are we using loaded data

    if not rospy.is_shutdown():
        rospy.init_node('grasp_pose', anonymous=True)
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        rospy.Subscriber("grasps", Path, grasp_callback)
        pub_graspnet = rospy.Publisher('start_graspnet', Bool, queue_size=10)
        pub_grasp = rospy.Publisher('pose_to_reach', PoseStamped, queue_size=10)
        pub_poses = rospy.Publisher('poses_to_reach', PoseArray, queue_size=10)
        pub_waypoint = rospy.Publisher('pose_to_reach_waypoint', PoseStamped, queue_size=10)
        rate = rospy.Rate(5)

        new_grasps = False
        cloud = None
        cloud_idx = None
        if live_run:
            cloudSubscriber = rospy.Subscriber("/sensors/realsense/pointcloudGeometry/static", PointCloud2, callbackPointCloud)
            print('Capturing new scene...')
            captureNewScene()

            print('Waiting for point cloud...')
            while cloud is None:
                rate.sleep
            cloudSubscriber.unregister()

            print('Get uv index for pointcloud...')
            cloud_idx = getUvStatic()
            if save_data:
                np.savetxt('data/pointcloud_idx.csv', np.asanyarray(cloud_idx), delimiter=',')
                np.savetxt('data/pointcloud.csv', np.asanyarray(cloud), delimiter=',')

            print('Run graspnet...')
            run_graspnet(pub_graspnet)
            new_grasps = False

        else:
            print('Not live run loading data...')
            cloud_idx = np.loadtxt('data/pointcloud_idx.csv', delimiter=',')
            cloud_idx = cloud_idx.astype(int)
            cloud = np.loadtxt('data/pointcloud.csv', delimiter=',')


        print('Getting affordance results...')
        masks, objects = getAffordanceResult(live_run, save=save_data)

        print('Waiting for grasps from graspnet...')
        while not new_grasps and not rospy.is_shutdown():
            rate.sleep()

        #Remove grasps with low score
        graspData = []
        for i in range(len(grasp_data.poses)):
            grasp = grasp_data.poses[i]
            if float(grasp.header.frame_id) > 0.1:
                graspData.append(grasp)

        # run through all objects found by affordance net
        good_grasps_all = []
        for obj_idx, mask in enumerate(masks):
            # Merge masks and subtract unwanted masks
            mask_ids_to_subtract = [1, 2, 6, 7, 8]
            mask_full = merge_subtract_masks(mask, mask_ids_to_subtract, visualize=False)

            # Loop cloud indexes in mask and extract masked pointcloud
            cloud_masked = []
            for count, idx in enumerate(cloud_idx):
                if mask_full[idx[0]][idx[1]] != 0:
                    cloud_masked.append(cloud[count])

            good_grasps = fuse_grasp_affordance(cloud_masked, graspData, visualize=False)
            # Set frame_id to object_id
            for grasp in good_grasps:
                grasp.header.frame_id = str(objects[obj_idx])
                good_grasps_all.append(grasp)

            print('Nr. of grasps found: ' + str(len(good_grasps)) + '  For object class: ' + str(objects[obj_idx]))

        #print('Grasp data: ' + str(good_grasps_all))


        # Evaluating the best grasp.
        world_frame = "world"
        ee_frame = "right_ee_link"
        if demo:
            camera_frame = "ptu_camera_color_optical_frame_real"
        else:
            camera_frame = "ptu_camera_color_optical_frame"

        grasps, waypoints = [], []
        for grasp in good_grasps_all:

            grasp.header.frame_id = camera_frame

            graspCamera = copy.deepcopy(grasp)
            waypointCamera = copy.deepcopy(grasp)

            # computing waypoint in camera frame
            quaternion = (
                graspCamera.pose.orientation.x,
                graspCamera.pose.orientation.y,
                graspCamera.pose.orientation.z,
                graspCamera.pose.orientation.w)

            rotMat = quaternion_matrix(quaternion)[:3, :3]
            offset = np.array([[0.15], [0.0], [0.0]])
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
            azimuthAngleLimit = [-1. * math.pi, 1.0 * math.pi]
            polarAngleLimit = [0, 1. * math.pi]

            # azimuthAngleLimit = [-1*math.pi, 1*math.pi]
            # polarAngleLimit = [0, 0.5*math.pi]

            if azimuthAngle > azimuthAngleLimit[0] and azimuthAngle < azimuthAngleLimit[1]:
                if polarAngle > polarAngleLimit[0] and polarAngle < polarAngleLimit[1]:
                    waypointWorld.header.stamp = rospy.Time.now()
                    waypointWorld.header.frame_id = world_frame
                    graspWorld.header.stamp = rospy.Time.now()
                    graspWorld.header.frame_id = world_frame

                    waypoints.append(waypointWorld)
                    grasps.append(graspWorld)

                    # pub_waypoint.publish(waypoints[-1])
                    # pub_grasp.publish(grasps[-1])
                    # rospy.sleep(1)

        if len(grasps) == 0 or len(waypoints) == 0:
            print("Could not find grasp with appropriate angle")
        else:
            """
            eeWorld = tf_buffer.lookup_transform("world", "right_ee_link", rospy.Time.now(), rospy.Duration(1.0))
            weightedSums = []
            for i in range(len(grasps)):
                deltaRPY = abs(calculate_delta_orientation(grasps[i], eeWorld))
                weightedSum = 0.2 * deltaRPY[0] + 0.4 * deltaRPY[1] + 0.4 * deltaRPY[2]
                weightedSums.append(weightedSum)

            weightedSums_sorted = sorted(weightedSums)
            grasps_sorted = [None] * len(grasps)
            for i in range(len(weightedSums)):
                num = weightedSums_sorted[i]
                index = weightedSums.index(num)
                grasps_sorted[i] = grasps[index]
            # publish both the waypoint and the grasp to their own topics for visualisation
            # pub_waypoint.publish(waypoints[0])
            # pub_grasp.publish(grasps[0])
            # now publish both as a single message for moveit
            """
            poses = geometry_msgs.msg.PoseArray()
            poses.header.frame_id = grasps[0].header.frame_id
            poses.header.stamp = rospy.Time.now()
            for i in range(len(grasps)):
                poses.poses.append(waypoints[i].pose)
                poses.poses.append(grasps[i].pose)
            pub_poses.publish(poses)
            print("Published poses")

        # Affordance segmentation here

        exit()

        # Finding the grasp with the least angle difference
        eeWorld = tf_buffer.lookup_transform("world", "right_ee_link", rospy.Time.now(), rospy.Duration(1.0))

        weightedSums = []

        for i in range(len(grasps)):
            deltaRPY = abs(calculate_delta_orientation(graspWorld, eeWorld))
            weightedSum = 0.2 * deltaRPY[0] + 0.4 * deltaRPY[1] + 0.4 * deltaRPY[2]
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
