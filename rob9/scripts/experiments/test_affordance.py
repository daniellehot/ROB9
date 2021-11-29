#!/usr/bin/env python3

import os
import sys
import cv2
import numpy as np
import argparse
import xml.etree.ElementTree as ET

import rospy

from affordanceService.client import AffordanceClient

def bb_intersection_over_union(boxA, boxB):
    """https://www.pyimagesearch.com/2016/11/07/intersection-over-union-iou-for-object-detection/ """
    # determine the (x, y)-coordinates of the intersection rectangle

    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    # compute the area of intersection rectangle
    interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
    # compute the area of both the prediction and ground-truth
    # rectangles
    boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
    boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
    # compute the intersection over union by taking the intersection
    # area and dividing it by the sum of prediction + ground-truth
    # areas - the interesection area
    iou = interArea / float(boxAArea + boxBArea - interArea)
    # return the intersection over union value
    return iou


parser = argparse.ArgumentParser()
parser.add_argument('--GPU', action="store_true", help='Use GPU acceleration')
parser.add_argument('--resolution', type=int, help='Steps in ROC')
parser.add_argument('--iou_threshold', type=float, help='iou threshold for when it is a valid detection')
parser.add_argument('--recompute', action="store_true", help="Recompute the values for the dataset")
args = parser.parse_args()

rospy.init_node('experiment_affordance', anonymous=True)

if args.resolution is None:
    args.resolution = 10

if args.iou_threshold is None:
    args.iou_threshold = 0.2

print("Using GPU: ", args.GPU)
print("ROC steps: ", args.resolution)
print("iou threshold: ", args.iou_threshold)
print("Recomputing data through affordance analyzer: ", args.recompute)

affClient = AffordanceClient()
affClient.start(GPU = args.GPU)

basePath = os.path.split(sys.argv[0])[0]
dataPath = os.path.join(basePath, "data")

_, dirs, _ = next(os.walk(dataPath))

print("Analyzing dataset with affordance analyzer...")

for i, dir in enumerate(dirs):
    basePath = os.path.join(dataPath, dir)
    filePath = os.path.join(basePath, "rgb.png")

    if (args.recompute or not os.path.exists(os.path.join(basePath, "masks.npy"))
    or not os.path.exists(os.path.join(basePath, "objects.npy")) or not
    os.path.exists(os.path.join(basePath, "scores.npy")) or not
    os.path.exists(os.path.join(basePath, "bbox.npy"))):

        img = cv2.imread(filePath)

        _ = affClient.run(img, CONF_THRESHOLD = 0.0)

        masks, objects, scores, bbox = affClient.getAffordanceResult()

        np.save(os.path.join(basePath, "masks.npy"), masks)
        np.save(os.path.join(basePath, "objects.npy"), objects)
        np.save(os.path.join(basePath, "scores.npy"), scores)
        np.save(os.path.join(basePath, "bbox.npy"), bbox)

        vizBbox = affClient.visualizeBBox(img)
        vizMasks = affClient.visualizeMasks(img)

        cv2.imwrite(os.path.join(basePath, "bbox.png"), vizBbox)
        cv2.imwrite(os.path.join(basePath, "masks.png"), vizMasks)

    print(i + 1, "/", len(dirs))

print("Computing metrics...")

scoreThresh = np.arange(args.resolution) / args.resolution


for thresh in scoreThresh:
    correct, incorrect, iou = 0, 0, []

    for i, dir in enumerate(dirs):
        basePath = os.path.join(dataPath, dir)

        # load predictions
        objects = np.load(os.path.join(basePath, "objects.npy"))
        scores = np.load(os.path.join(basePath, "scores.npy"))
        bbox = np.load(os.path.join(basePath, "bbox.npy"))

        obj_names = []
        for object in objects:
            obj_names.append(affClient.OBJ_CLASSES[object])

        # Compare against ground truth
        root = ET.parse(os.path.join(basePath, "rgb.xml"))

        for obj, score, bbox in zip(obj_names, scores, bbox):

            fail = True
            for object_tag in root.findall('object'):
                object_gt = object_tag.find('name').text

                bbox_tag = object_tag.find('bndbox')
                x_min = int(bbox_tag.find('xmin').text)
                y_min = int(bbox_tag.find('ymin').text)
                x_max = int(bbox_tag.find('xmax').text)
                y_max = int(bbox_tag.find('ymax').text)
                bbox_gt = [x_min, y_min, x_max, y_max]

                if obj == object_gt:
                    if bb_intersection_over_union(bbox_gt, bbox) > args.iou_threshold:
                        if score >= thresh:
                            correct += 1
                            iou.append(bb_intersection_over_union(bbox_gt, bbox))
                            fail = False
                            break

            if fail:
                incorrect += 1

    print("At score threshold: ", thresh)
    print("Correctly detected: ", correct)
    print("Incorrectly detected: ", incorrect)
    print("Precision: ", round(correct / (correct + incorrect), 3))
    print()
