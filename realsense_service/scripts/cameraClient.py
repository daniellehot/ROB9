import sys
import rospy
from realsense_service.srv import *
import numpy as np
import cv2
from cv_bridge import CvBridge

class CameraClient(object):
    """docstring for CameraClient."""

    def __init__(self, type="realsenseD435"):
        self.type = type
        self.rgb = 0
        self.depth = 0
        self.uv = 0

        if self.type == "realsenseD435":
            self.baseService = "/sensors/realsense"
        else:
            raise Exception("Invalid type")

        self.serviceNameCapture = self.baseService + "/capture"
        self.serviceNameRGB = self.baseService + "/rgb"
        self.serviceNameDepth = self.baseService + "/depth"
        self.serviceNameUV = self.baseService + "/pointcloudGeometry/static/uv"

        self.getRGB()
        self.getDepth()
        self.getUvStatic()

    def captureNewScene(self):
        """ Tells the camera service to update the static data """

        rospy.wait_for_service(self.serviceNameCapture)
        captureService = rospy.ServiceProxy(self.serviceNameCapture, capture)
        msg = capture()
        msg.data = True
        response = captureService(msg)

    def getRGB(self):
        """ Sets the self.rgb to current static rgb captured by camera """

        rospy.wait_for_service(self.serviceNameRGB)
        rgbService = rospy.ServiceProxy(self.serviceNameRGB, rgb)
        msg = rgb()
        msg.data = True
        response = rgbService(msg)
        img = np.frombuffer(response.img.data, dtype=np.uint8).reshape(response.img.height, response.img.width, -1)
        self.rgb = img


    def getDepth(self):
        """ Sets the self.depth to current static depth image captured by
        camera """

        rospy.wait_for_service(self.serviceNameDepth)
        depthService = rospy.ServiceProxy(self.serviceNameDepth, depth)
        msg = depth()
        msg.data = True
        response = depthService(msg)
        br = CvBridge()
        img = br.imgmsg_to_cv2(response.img, desired_encoding='passthrough')
        self.depth = img

    def getUvStatic(self):
        """ Sets the self.uv to current static uv coordinates for translation
        from pixel coordinates to point cloud coordinates """
        
        rospy.wait_for_service("/sensors/realsense/pointcloudGeometry/static/uv")
        uvStaticService = rospy.ServiceProxy("/sensors/realsense/pointcloudGeometry/static/uv", uvSrv)
        msg = uvSrv()

        msg.data = True
        response = uvStaticService(msg)
        rows = int(response.uv.layout.dim[0].size / response.uv.layout.dim[1].size)
        cols = int(response.uv.layout.dim[1].size)

        uvStatic = np.array(response.uv.data).astype(int)
        uvStatic = np.reshape(uvStatic, (rows, cols))
        self.uv = uvStatic
