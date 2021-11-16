#!/usr/bin/env python
# modified from: https://github.com/nqanh/affordance-net/blob/master/tools/demo_img.py
import sys

import numpy as np
import os, cv2
import argparse
import sys
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

from affordance_analyzer.srv import affordanceSrv, affordanceSrvResponse
from std_msgs.msg import MultiArrayDimension
import rospy
from cameraService.cameraClient import CameraClient

import caffe

sys.path.append('/affordance-net/tools')

import _init_paths
from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect2
from fast_rcnn.nms_wrapper import nms
from utils.timer import Timer

CONF_THRESHOLD = 0.7
good_range = 0.005

# get current dir
cwd = os.getcwd()
#root_path = os.path.abspath(os.path.join(cwd, os.pardir))  # get parent path
root_path = "/affordance-net/"
print 'AffordanceNet root folder: ', root_path
img_folder = cwd + '/img'

OBJ_CLASSES = ('__background__', 'bowl', 'tvm', 'pan', 'hammer', 'knife', 'cup', 'drill', 'racket', 'spatula', 'bottle')

# Mask
background = [200, 222, 250]
c1 = [0,0,205]
c2 = [34,139,34]
c3 = [192,192,128]
c4 = [165,42,42]
c5 = [128,64,128]
c6 = [204,102,0]
c7 = [184,134,11]
c8 = [0,153,153]
c9 = [0,134,141]
c10 = [184,0,141]
c11 = [184,134,0]
c12 = [184,134,223]
label_colours = np.array([background, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12])
affordanceLabel = {0: 'background', 1: 'contain', 2: 'cut', 3: 'display', 4: 'engine', 5: 'grasp', 6: 'hit', 7: 'pound', 8:'support', 9:'w-grasp'}

# Object
col0 = [0, 0, 0]
col1 = [0, 255, 255]
col2 = [255, 0, 255]
col3 = [0, 125, 255]
col4 = [55, 125, 0]
col5 = [255, 50, 75]
col6 = [100, 100, 50]
col7 = [25, 234, 54]
col8 = [156, 65, 15]
col9 = [215, 25, 155]
col10 = [25, 25, 155]

col_map = [col0, col1, col2, col3, col4, col5, col6, col7, col8, col9, col10]

def reset_mask_ids(mask, before_uni_ids):
    # reset ID mask values from [0, 1, 4] to [0, 1, 2] to resize later
    counter = 0
    for id in before_uni_ids:
        mask[mask == id] = counter
        counter += 1

    return mask



def convert_mask_to_original_ids_manual(mask, original_uni_ids):
    #TODO: speed up!!!
    temp_mask = np.copy(mask) # create temp mask to do np.around()
    temp_mask = np.around(temp_mask, decimals=0)  # round 1.6 -> 2., 1.1 -> 1.
    current_uni_ids = np.unique(temp_mask)

    out_mask = np.full(mask.shape, 0, 'float32')

    mh, mw = mask.shape
    for i in range(mh-1):
        for j in range(mw-1):
            for k in range(1, len(current_uni_ids)):
                if mask[i][j] > (current_uni_ids[k] - good_range) and mask[i][j] < (current_uni_ids[k] + good_range):
                    out_mask[i][j] = original_uni_ids[k]
                    #mask[i][j] = current_uni_ids[k]

#     const = 0.005
#     out_mask = original_uni_ids[(np.abs(mask - original_uni_ids[:,None,None]) < const).argmax(0)]

    #return mask
    return out_mask




def draw_arrow(image, p, q, color, arrow_magnitude, thickness, line_type, shift):
    # draw arrow tail
    cv2.line(image, p, q, color, thickness, line_type, shift)
    # calc angle of the arrow
    angle = np.arctan2(p[1]-q[1], p[0]-q[0])
    # starting point of first line of arrow head
    p = (int(q[0] + arrow_magnitude * np.cos(angle + np.pi/4)),
    int(q[1] + arrow_magnitude * np.sin(angle + np.pi/4)))
    # draw first half of arrow head
    cv2.line(image, p, q, color, thickness, line_type, shift)
    # starting point of second line of arrow head
    p = (int(q[0] + arrow_magnitude * np.cos(angle - np.pi/4)),
    int(q[1] + arrow_magnitude * np.sin(angle - np.pi/4)))
    # draw second half of arrow head
    cv2.line(image, p, q, color, thickness, line_type, shift)

def draw_reg_text(img, obj_info):
    #print 'tbd'

    obj_id = obj_info[0]
    cfd = obj_info[1]
    xmin = obj_info[2]
    ymin = obj_info[3]
    xmax = obj_info[4]
    ymax = obj_info[5]

    draw_arrow(img, (xmin, ymin), (xmax, ymin), col_map[obj_id], 0, 5, 8, 0)
    draw_arrow(img, (xmax, ymin), (xmax, ymax), col_map[obj_id], 0, 5, 8, 0)
    draw_arrow(img, (xmax, ymax), (xmin, ymax), col_map[obj_id], 0, 5, 8, 0)
    draw_arrow(img, (xmin, ymax), (xmin, ymin), col_map[obj_id], 0, 5, 8, 0)

    # put text
    txt_obj = OBJ_CLASSES[obj_id] + ' ' + str(cfd)
    cv2.putText(img, txt_obj, (xmin, ymin-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1) # draw with red
    #cv2.putText(img, txt_obj, (xmin, ymin), cv2.FONT_HERSHEY_SIMPLEX, 1, col_map[obj_id], 2)

#     # draw center
#     center_x = (xmax - xmin)/2 + xmin
#     center_y = (ymax - ymin)/2 + ymin
#     cv2.circle(img,(center_x, center_y), 3, (0, 255, 0), -1)

    return img



def visualize_mask(im, rois_final, rois_class_score, rois_class_ind, masks, ori_height, ori_width, im_name, thresh):

    if rois_final.shape[0] == 0:
        print 'No detected box at all!'
        return

    inds = np.where(rois_class_score[:, -1] >= thresh)[0]
    if len(inds) == 0:
        print 'No detected box with probality > thresh = ', thresh, '-- Choossing highest confidence bounding box.'
        inds = [np.argmax(rois_class_score)]
        max_conf = np.max(rois_class_score)
        if max_conf < 0.001:
            return  ## confidence is < 0.001 -- no good box --> must return


    rois_final = rois_final[inds, :]
    rois_class_score = rois_class_score[inds,:]
    rois_class_ind = rois_class_ind[inds,:]


    # get mask
    masks = masks[inds, :, :, :]

    im_width = im.shape[1]
    im_height = im.shape[0]

    # transpose
    im = im[:, :, (2, 1, 0)]
    img_org = im.copy()

    num_boxes = rois_final.shape[0]

    list_bboxes = []

    for i in xrange(0, num_boxes):

        curr_mask = np.full((im_height, im_width), 0.0, 'float') # convert to int later

        class_id = int(rois_class_ind[i,0])

        bbox = rois_final[i, 1:5]
        score = rois_class_score[i,0]

        if cfg.TEST.MASK_REG:

            x1 = int(round(bbox[0]))
            y1 = int(round(bbox[1]))
            x2 = int(round(bbox[2]))
            y2 = int(round(bbox[3]))

            x1 = np.min((im_width - 1, np.max((0, x1))))
            y1 = np.min((im_height - 1, np.max((0, y1))))
            x2 = np.min((im_width - 1, np.max((0, x2))))
            y2 = np.min((im_height - 1, np.max((0, y2))))

            cur_box = [class_id, score, x1, y1, x2, y2]
            list_bboxes.append(cur_box)

            h = y2 - y1
            w = x2 - x1

            fig = plt.figure(figsize=plt.figaspect(0.1))
            gs = gridspec.GridSpec(2, 7)
            axMain = fig.add_subplot(gs[:,:2])
            axMain.set_title("Image")
            axMain.imshow(img_org[y1:y2, x1:x2])
            ax = []

            for j in range(5):
                mask = masks[i, j, :, :] * 255
                curr_mask = mask.astype('uint8')

                ax.append(fig.add_subplot(gs[0, j+2]))
                ax[-1].set_title(affordanceLabel[j])
                ax[-1].axis('off')
                ax[-1].imshow(curr_mask)
            for j in range(5):
                mask = masks[i, j+5, :, :] * 255
                curr_mask = mask.astype('uint8')

                ax.append(fig.add_subplot(gs[1, j+2]))
                ax[-1].set_title(affordanceLabel[j+5])
                ax[-1].axis('off')
                ax[-1].imshow(curr_mask)
            plt.show()

    for ab in list_bboxes:
        print 'box: ', ab
        img_out = draw_reg_text(img_org, ab)

    cv2.imshow('Obj detection', img_out)
    #cv2.imwrite('obj_detction.jpg', img_out)
    cv2.waitKey(0)



def run_affordance_net(net, im, CONF_THRESHOLD = 0.7):

    ori_height, ori_width, _ = im.shape

    # Detect all object classes and regress object bounds
    timer = Timer()
    timer.tic()
    if cfg.TEST.MASK_REG:
        rois_final, rois_class_score, rois_class_ind, masks, scores, boxes = im_detect2(net, im)
        print(rois_final, rois_class_score)
    else:
        1
    timer.toc()
    # relevant: masks (3, 10, 244, 244)
    #           rois_class_ind (3, 1) float to int
    #           rois_final (3, 5) # leave out 1. float to int
    #           rois_class_score # (3, 1) float
    # irrelevant: boxes, scores
    #visualize_mask(im, rois_final, rois_class_score, rois_class_ind, masks, ori_height, ori_width, "testnameREPLACE", thresh=CONF_THRESHOLD)
    print ('Detection took {:.3f}s for '
           '{:d} object proposals').format(timer.total_time, rois_final.shape[0])

    inds = np.where(rois_class_score[:, -1] >= CONF_THRESHOLD)[0]
    # get mask
    rois_final = rois_final[inds]
    rois_class_ind = rois_class_ind[inds]
    masks = masks[inds, :, :, :]
    masks = masks * 255
    num_boxes = rois_final.shape[0]
    im_width = im.shape[1]
    im_height = im.shape[0]

    masks_list = []
    for i in range(masks.shape[0]):
        bbox = rois_final[i]
        x1, y1, x2, y2 = int(bbox[1]), int(bbox[2]), int(bbox[3]), int(bbox[4])
        object_mask_list = []
        for j in range(masks.shape[1]):
            mask_resized = np.zeros((im_height, im_width))
            mask = masks[i, j, :, :]
            mask = cv2.resize(mask, (int(x2-x1), int(y2-y1)), interpolation=cv2.INTER_LINEAR)
            mask_resized[y1:y2, x1:x2] = mask
            object_mask_list.append(mask_resized)
        masks_list.append(object_mask_list)

    masks = np.array(masks_list)

    try:
        return rois_final, rois_class_ind, masks
    except:
        return 0
    # Visualize detections for each class

def sendResults(bbox, objects, masks):
    intToLabel = {0: 'class', 1: 'height', 2: 'width'}
    msg = affordanceSrvResponse()

    # constructing mask message
    for i in range(3):
        dimMsg = MultiArrayDimension()
        dimMsg.label = intToLabel[i]
        stride = 1
        for j in range(3-i):
            stride = stride * masks.shape[i+j]
        dimMsg.stride = stride
        dimMsg.size = masks.shape[i]
        msg.masks.layout.dim.append(dimMsg)
    masks = masks.flatten().astype(int).tolist()
    msg.masks.data = masks

    # constructing bounding box message
    msg.bbox.data = bbox.flatten().astype(int).tolist()

    # constructing object detection class message
    msg.object.data = objects.flatten().tolist()

    return msg

def analyzeAffordance(msg):
    cam = CameraClient()
    img = cam.getRGB()

    CONF_THRESHOLD = msg.confidence_threshold.data

    print("Analyzing affordance with confidence threshold: ", CONF_THRESHOLD)
    try:
        bbox, objects, masks = run_affordance_net(net, img, CONF_THRESHOLD=CONF_THRESHOLD)
        bbox = bbox[:,1:]
        m = masks[0]
        for i in range(len(masks)-1):
            m = np.vstack((m, masks[i+1]))
        masks = m

        print(bbox.shape, objects.shape, masks.shape)
        print(objects)
    except:
        bbox = np.zeros((1, 4))
        objects = np.zeros((1,1))
        masks = np.zeros((1, 10, 244, 244))

    return sendResults(bbox, objects, masks)

if __name__ == '__main__':
    cfg.TEST.HAS_RPN = True  # Use RPN for proposals

    prototxt = root_path + '/models/pascal_voc/VGG16/faster_rcnn_end2end/test.prototxt'
    caffemodel = root_path + '/pretrained/AffordanceNet_200K.caffemodel'

    if not os.path.isfile(caffemodel):
        raise IOError(('{:s} not found.\n').format(caffemodel))

    caffe.set_mode_cpu()

    # load network
    net = caffe.Net(prototxt, caffemodel, caffe.TEST)
    print '\n\nLoaded network {:s}'.format(caffemodel)

    rospy.init_node('affordance_analyzer')
    serviceCapture = rospy.Service('/affordance/result', affordanceSrv, analyzeAffordance)
    rospy.spin()
