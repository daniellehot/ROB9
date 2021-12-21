#!/usr/bin/env python3
import sys
import copy
import rospy
import math
import numpy as np
import time
import open3d as o3d
import cv2
import imageio

#import moveit_commander
#import moveit_msgs

import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
import std_msgs.msg
from std_msgs.msg import Int8, MultiArrayDimension, MultiArrayLayout, Int32MultiArray, Float32MultiArray, Bool, Header
from sensor_msgs.msg import PointCloud2, PointField, Image
import sensor_msgs.point_cloud2 as pc2

import rob9Utils.transformations as transform
from rob9Utils.graspGroup import GraspGroup
from rob9Utils.grasp import Grasp
import rob9Utils.moveit as moveit
from cameraService.cameraClient import CameraClient
from affordanceService.client import AffordanceClient
from grasp_service.client import GraspingGeneratorClient
from rob9Utils.visualize import visualizeGrasps6DOF

from moveit_scripts.srv import *
from moveit_scripts.msg import *
from grasp_aff_association.srv import *
from rob9.srv import graspGroupSrv, graspGroupSrvResponse

video_out_path = "handover.mp4"
video_out = imageio.get_writer(video_out_path, fps=25)


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


def callback(msg):
    global resp_trajectories, grasps_affordance, pcd, grasps_objects_viz, single_grasp_viz, demo, state

    if state > 6:
        #global receivedGripperCommand
        #print("Callback")
        #print("message data " + str(msg.data))
        id = msg.data

        # 1 = contain
        # 2 = cut
        # 3 = display
        # 4 = engine
        # 5 = grasp
        # 6 = hit
        # 7 = pound
        # 8 = support
        # 9 = wide grasp

        affordance_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9]

        # Select affordance label to grasp
        #print(grasps_affordance[:].affordance_id)
        graspObj = GraspGroup(grasps = copy.deepcopy(grasps_affordance.getgraspsByTool(id = id) ))
        graspAffordance = GraspGroup(grasps = [])

        state = 8

        for affordance_id in affordance_ids:
            if affordance_id != 5 and affordance_id != 9:
                graspAffordance.combine(GraspGroup(grasps = copy.deepcopy(graspObj.getgraspsByAffordanceLabel(label = affordance_id) ) ))
            #if affordance_id == 5 or affordance_id == 9:
            #    graspObj.combine(GraspGroup(grasps = copy.deepcopy(grasps_affordance.getgraspsByAffordanceLabel(label = affordance_id) ) ))
        grasps_affordance = graspAffordance
        #visualizeGrasps6DOF(pcd, grasps_affordance)
        grasps_objects_internal = visualizeGrasps6DOF(pcd, grasps_affordance)
        grasps_objects_viz = visualize_open3d(clouds = [pcd, *grasps_objects_internal])

        grasp_waypoints_path = computeWaypoints(grasps_affordance, offset = 0.1)




        #visualizeGrasps6DOF(pcd, GraspGroup(grasps = graspObjects.fromPath(grasp_waypoints_path)))

        azimuthAngleLimit = [-1*math.pi, 1*math.pi]
        polarAngleLimit = [0, 0.5*math.pi]
        #grasp_waypoints_path = filterBySphericalCoordinates(grasp_waypoints_path, azimuth = azimuthAngleLimit, polar = polarAngleLimit)

        state = 9
        print("Calling the trajectory service")
        rospy.wait_for_service('get_trajectories')
        get_trajectories = rospy.ServiceProxy('get_trajectories', GetTrajectories)
        resp_trajectories = get_trajectories(grasp_waypoints_path)
        print("I have received a trajectory server response ")

        id_list_duplicates = []
        for i in range(len(resp_trajectories.trajectories.trajectories)):
            id_list_duplicates.append(resp_trajectories.trajectories.trajectories[i].joint_trajectory.header.frame_id)
        id_list = list(dict.fromkeys(id_list_duplicates))
        print("id_list_duplicates " + str(id_list_duplicates))
        print("id_list " + str(id_list))

        id = str(id)
        plans = []
        goal_poses = []
        for i in range(len(resp_trajectories.trajectories.trajectories)):
            if resp_trajectories.trajectories.trajectories[i].joint_trajectory.header.frame_id == id:
                plans.append(resp_trajectories.trajectories.trajectories[i])

        for i in range(len(resp_trajectories.trajectories_poses.poses)):
            if resp_trajectories.trajectories_poses.poses[i].header.frame_id == id:
                goal_poses.append(resp_trajectories.trajectories_poses.poses[i])

        #print("I have sampled these trajectories "  + str(len(plans)))
        #print("I have sampled these goal poses "  + str(len(goal_poses)))

        waypoint_msg = geometry_msgs.msg.PoseStamped()
        waypoint_msg.header.frame_id = "world"
        waypoint_msg.header.stamp = rospy.Time.now()
        waypoint_msg.pose = goal_poses[0].pose
        pub_waypoint.publish(waypoint_msg)

        goal_msg = geometry_msgs.msg.PoseStamped()
        goal_msg.header.frame_id = "world"
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose = goal_poses[1].pose
        pub_grasp.publish(goal_msg)

        target_frame = "ptu_camera_color_optical_frame"
        if demo.data:
            target_frame = "ptu_camera_color_optical_frame_real"
        single_grasp = GraspGroup(grasps = [Grasp().fromPoseStampedMsg(transform.transformToFrame(goal_msg, target_frame))])
        single_grasp_internal = visualizeGrasps6DOF(pcd, single_grasp)
        single_grasp_viz = visualize_open3d(clouds = [pcd, *single_grasp_internal])
        state = 10

        #rospy.sleep(6.)
        #raw_input("Press Enter when you are ready to move the robot")
        #input("Press Enter when you are ready to move the robot")
        for i in range(3):
            send_trajectory_to_rviz(plans[i])
            #raw_input("Press Enter when you are ready to move the robot")
            #move_group.execute(plans[i], wait=True)
            #move_group.stop()
            #move_group.clear_pose_targets()
            moveit.execute(plans[i])
            if i == 1:
                gripper_pub.publish(close_gripper_msg)
                rospy.sleep(1)
            print("I have grasped!")
        #raw_input("Press Enter when you are ready to move the robot to the handover pose")
        #input("Press Enter when you are ready to move the robot to the handover pose")
        #move_to_handover()
        #input("Press Enter when you are ready to move the robot")
        moveit.moveToNamed("ready")
        #input("Press Enter when you are ready to move the robot")
        moveit.moveToNamed("handover")
        #raw_input("Press Enter when you are ready to move the robot back to the ready pose")
        #input("Press Enter when you are ready to move the robot back to the home pose")
        rospy.sleep(6)

        moveit.moveToNamed("ready")
        gripper_pub.publish(open_gripper_msg)
        state = 1
        #move_to_ready()

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

        #q_x_180 = [0, 0, 1, 0]
        #g_q = grasp.orientation.getVector(format="xyzw")
        #q = transform.quaternionMultiply(g_q, q_x_180)

        #print(grasp.orientation)

        #grasp.orientation.x = q[0]
        #grasp.orientation.y = q[1]
        #grasp.orientation.z = q[2]
        #grasp.orientation.w = q[3]

        #print(grasp.orientation)
        #print()

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

        waypointWorld.frame_id = str(graspObjects[i].tool_id) # we should probably do away with storing it in the header
        graspWorld.frame_id = str(graspObjects[i].tool_id)
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

def create_visualization(img_msg):
    global img, objects, pcd_img, img_predictions, all_grasp_viz, clouds_masked_viz, grasps_objects_viz, single_grasp_viz, state

    width = 1280
    height = 960
    h_divisions = 4
    w_divisions = 2
    viz_h_unit = int(height / h_divisions)
    viz_w_unit = viz_h_unit

    frame = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)

    state_text = ["Initializing",
                "Camera is capturing scene...",
                "Segmenting affordances in RGB image...",
                "Segmenting affordance point clouds...",
                "Generating grasps...",
                "Computing task-oriented grasps...",
                "Ready for command",
                "Computing waypoints...",
                "Computing trajectory...",
                "Performing handover..."]

    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (50, 150)
    fontscale = 0.8
    color = (0, 0, 255)
    thickness = 2
    frame = cv2.putText(frame, state_text[state-1], org, font, fontscale, color, thickness, cv2.LINE_AA)

    frame = cv2.resize(frame, (width, height))
    viz_box = np.ones((height, int(viz_w_unit * 2), 3)).astype(np.uint8) * 255
    viz_box[:viz_h_unit, :viz_w_unit, :] = cv2.resize(img, (viz_w_unit, viz_h_unit))
    viz_box[:viz_h_unit, viz_w_unit:, :] = cv2.resize(pcd_img, (viz_w_unit, viz_h_unit))

    if all_grasp_viz is not None:
        viz_box[int(viz_h_unit * 2): int(3*viz_h_unit), :viz_w_unit, :] = cv2.resize(all_grasp_viz,  (viz_w_unit, viz_h_unit))

    if img_predictions is not None:
        viz_box[int(viz_h_unit * 1): int(2*viz_h_unit), :viz_w_unit, :] = cv2.resize(img_predictions,  (viz_w_unit, viz_h_unit))

    if clouds_masked_viz is not None:
        viz_box[int(viz_h_unit * 1): int(2*viz_h_unit), viz_w_unit:, :] = cv2.resize(clouds_masked_viz,  (viz_w_unit, viz_h_unit))

    if grasps_objects_viz is not None:
        viz_box[int(viz_h_unit * 2): int(3*viz_h_unit), viz_w_unit:, :] = cv2.resize(grasps_objects_viz,  (viz_w_unit, viz_h_unit))

    if single_grasp_viz is not None:
        viz_box[int(viz_h_unit * 3): int(4*viz_h_unit), int(viz_w_unit * 0.5):int(viz_w_unit * 1.5), :] = cv2.resize(single_grasp_viz,  (viz_w_unit, viz_h_unit))


    frame = np.hstack((frame, viz_box))






    video_out.append_data(frame[:,:,::-1])
    #video_out.append_data(frame.astype(np.uint8))
    cv2.imshow("My cam video", frame)
    # Close and break the loop after pressing "x" key
    #cv2.waitKey(1)
    if cv2.waitKey(1) &0XFF == ord('x'):
        cv2.destroyAllWindows()
        exit()
    #cv2.destroyAllWindows()
    #vid_capture.release()
    #exit()

def visualize_open3d(clouds = []):

    o3d_visualizer = o3d.visualization.Visualizer()
    o3d_visualizer.create_window()
    pcds = [pcd]
    o3d_visualizer.clear_geometries()
    for cloud in clouds:
        o3d_visualizer.add_geometry(cloud)
        o3d_visualizer.update_geometry(cloud)
    #o3d_visualizer.add_geometry()
    #o3d_visualizer.update_geometry()
    ctr = o3d_visualizer.get_view_control()
    ctr.rotate(x = 0, y = 1200, xo = 0, yo = 0)
    ctr.translate(x = 65, y = 40)
    ctr.set_zoom(0.25)
    ctr.scale(0.1)
    o3d_visualizer.poll_events()
    o3d_visualizer.update_renderer()

    pcd_img = (np.asarray(o3d_visualizer.capture_screen_float_buffer()) * 255).astype(np.uint8)
    pcd_img = cv2.cvtColor(pcd_img, cv2.COLOR_BGR2RGB)
    o3d_visualizer.close()
    return pcd_img

if __name__ == '__main__':
    global all_grasp_viz, img, img_predictions, single_grasp, clouds_masked_viz, pcd_img, grasps_objects_viz, single_grasp_viz, demo, state
    demo = std_msgs.msg.Bool()
    demo.data = False
    if len(sys.argv) > 1:
        if sys.argv[1] == 'demo':
            demo.data = True
            print("Demo = True")
        else:
            print("Invalid input argument")
            exit()

    all_grasp_viz = None
    img_predictions = None
    clouds_masked_viz = None
    grasps_objects_viz = None
    single_grasp_viz = None
    state = 1
    print("Init")
    rospy.init_node('moveit_subscriber', anonymous=True)
    rospy.Subscriber('tool_id', Int8, callback)
    gripper_pub = rospy.Publisher('gripper_controller', Int8, queue_size=10, latch=True)
    pub_grasp = rospy.Publisher('pose_to_reach', PoseStamped, queue_size=10)
    pub_waypoint = rospy.Publisher('pose_to_reach_waypoint', PoseStamped, queue_size=10)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # DO NOT REMOVE THIS SLEEP, it allows gripper_pub to establish connection to the topic
    rospy.sleep(0.1)
    rospy.sleep(2)

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

    gripper_pub.publish(reset_gripper_msg)
    gripper_pub.publish(activate_gripper_msg)
    gripper_pub.publish(open_gripper_msg)
    gripper_pub.publish(pinch_gripper_msg)
    moveit.moveToNamed("ready")

    print("Services init")

    state = 2
    print("Camera is capturing new scene")

    cam = CameraClient()
    cam.captureNewScene()
    cloud, cloudColor = cam.getPointCloudStatic()
    cloud_uv = cam.getUvStatic()
    img = cam.getRGB()

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud)
    pcd.colors = o3d.utility.Vector3dVector(cloudColor)

    pcd_img = visualize_open3d(clouds= [pcd])

    rospy.Subscriber('/sensors/webcam/img', Image, create_visualization)

    state = 3

    print("Segmenting affordance maps")
    # Run affordance analyzer
    affClient = AffordanceClient()

    #affClient.start(GPU=True)
    #_ = affClient.run(img, CONF_THRESHOLD = 0.3)

    _, _, _, _ = affClient.getAffordanceResult()

    _ = affClient.processMasks(conf_threshold = 40, erode_kernel=(11,11))

    # Visualize object detection and affordance segmentation to confirm
    img_predictions = affClient.visualizeMasks(img)
    img_predictions = affClient.visualizeBBox(img_predictions)

    masks = affClient.masks
    objects = affClient.objects

    state = 4
    print("Segmenting point cloud for visualization purpose")

    affordance_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9]
    colors = [(205,0,0), (34,139,34), (128,192,192), (42, 42, 165), (128, 64, 128),
            (0, 102, 204), (11, 134, 184), (153, 153, 0), (141, 134, 0), (141, 0, 184)]

    clouds_masked = []
    clouds_masked.append(pcd) # also visualize the rest of the point cloud
    for i in range(0, masks.shape[0]):
        for affordance_id in affordance_ids:
            # Loop cloud indexes in mask and extract masked pointcloud
            cloud_masked = []
            for k, uv in enumerate(cloud_uv):
                if masks[i, affordance_id, uv[0], uv[1]] != 0:
                    cloud_masked.append(cloud[k])
            pcd_mask = o3d.geometry.PointCloud()
            pcd_mask.points = o3d.utility.Vector3dVector(cloud_masked)
            pcd_mask.paint_uniform_color(colors[affordance_id])
            clouds_masked.append(copy.deepcopy(pcd_mask))


    clouds_masked_viz = visualize_open3d(clouds = clouds_masked)

    state = 5
    print("Generating grasps")

    # Get grasps from grasp generator
    graspClient = GraspingGeneratorClient()

    collision_thresh = 0.01 # Collision threshold in collision detection
    num_view = 300 # View number
    score_thresh = 0.0 # Remove every grasp with scores less than threshold
    voxel_size = 0.2

    graspClient.setSettings(collision_thresh, num_view, score_thresh, voxel_size)

    # Load the network with GPU (True or False) or CPU
    graspClient.start(GPU=True)

    graspData = graspClient.getGrasps()
    all_grasp_viz_internal = visualizeGrasps6DOF(pcd, graspData)
    all_grasp_viz = visualize_open3d(clouds = [pcd, *all_grasp_viz_internal])

    print("Got ", len(graspData), " grasps")

    state = 6
    print("Computing task oriented grasps")

    # Associate affordances with grasps
    grasps_affordance = associateGraspAffordance(graspData, objects, masks, cloud, cloud_uv, demo = demo.data)

    grasps_affordance.sortByScore()
    grasps_affordance.thresholdByScore(0.0)

    #visualizeGrasps6DOF(pcd, grasps_affordance)
    state = 7
    print("Ready for command")


    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
