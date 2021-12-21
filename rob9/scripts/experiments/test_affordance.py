#!/usr/bin/env python3

import os
import sys
import cv2
import numpy as np
import argparse
import xml.etree.ElementTree as ET
from tqdm import tqdm
from matplotlib import pyplot as plt

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
    args.resolution = 11

if args.iou_threshold is None:
    args.iou_threshold = 0.5

print("Using GPU: ", args.GPU)
print("ROC steps: ", args.resolution)
print("iou threshold: ", args.iou_threshold)
print("Recomputing data through affordance analyzer: ", args.recompute)

affClient = AffordanceClient()

if args.recompute:
    affClient.start(GPU = args.GPU)

basePath = os.path.split(sys.argv[0])[0]
dataPath = os.path.join(basePath, "data")

_, dirs, _ = next(os.walk(dataPath))

print("Analyzing dataset with affordance analyzer...")

for i, dir in enumerate(tqdm(dirs)):
    basePath = os.path.join(dataPath, dir)
    filePath = os.path.join(basePath, "rgb.png")
    outPath = os.path.join(basePath, affClient.name)
    if not os.path.exists(outPath):
        os.mkdir(outPath)

    if (args.recompute or not os.path.exists(os.path.join(outPath, "masks.npy"))
    or not os.path.exists(os.path.join(outPath, "objects.npy")) or not
    os.path.exists(os.path.join(outPath, "scores.npy")) or not
    os.path.exists(os.path.join(outPath, "bbox.npy"))):

        img = cv2.imread(filePath)

        _ = affClient.run(img, CONF_THRESHOLD = 0.0)

        masks, objects, scores, bbox = affClient.getAffordanceResult()

        np.save(os.path.join(outPath, "masks.npy"), masks)
        np.save(os.path.join(outPath, "objects.npy"), objects)
        np.save(os.path.join(outPath, "scores.npy"), scores)
        np.save(os.path.join(outPath, "bbox.npy"), bbox)

        vizBbox = affClient.visualizeBBox(img)
        vizMasks = affClient.visualizeMasks(img)

        cv2.imwrite(os.path.join(outPath, "bbox.png"), vizBbox)
        cv2.imwrite(os.path.join(outPath, "masks.png"), vizMasks)

print("Computing metrics...")

scoreThresh = np.linspace(0, 1, num = args.resolution)

OBJ_NAME_TO_ID = affClient.OBJ_NAME_TO_ID
precisions, recalls = [], []


for count, thresh in enumerate(tqdm(scoreThresh)):
    resultList = []
    for obj in OBJ_NAME_TO_ID:
        resultList.append([thresh, 0, 0, 0]) #confidence, TP, FP, FN

    for i, dir in enumerate(dirs):
        basePath = os.path.join(dataPath, dir)
        outPath = os.path.join(basePath, affClient.name)

        # load predictions
        objects = np.load(os.path.join(outPath, "objects.npy"))
        scores = np.load(os.path.join(outPath, "scores.npy"))
        bbox = np.load(os.path.join(outPath, "bbox.npy"))
        masks = np.load(os.path.join(outPath, "masks.npy"))

        obj_names = []
        for object in objects:
            obj_names.append(affClient.OBJ_CLASSES[object])

        # Compare against ground truth
        root = ET.parse(os.path.join(basePath, "rgb.xml"))
        no_preds = len(obj_names)

        gt_count = 0
        for count, (obj, score, bbox) in enumerate(zip(obj_names, scores, bbox)):
            obj_idx = OBJ_NAME_TO_ID[obj]

            fail = True
            for object_tag in root.findall('object'):
                if count == 0:
                    gt_count += 1
                object_gt = object_tag.find('name').text

                bbox_tag = object_tag.find('bndbox')
                x_min = int(bbox_tag.find('xmin').text)
                y_min = int(bbox_tag.find('ymin').text)
                x_max = int(bbox_tag.find('xmax').text)
                y_max = int(bbox_tag.find('ymax').text)
                bbox_gt = [x_min, y_min, x_max, y_max]

                if score >= thresh:
                    if bb_intersection_over_union(bbox_gt, bbox) >= args.iou_threshold:
                        if affClient.name == "affordancenet":
                            if obj == object_gt:
                                resultList[obj_idx][1] += 1 # TP
                                fail = False
                            if obj != object_gt:
                                resultList[obj_idx][2] += 1 # FP
                                fail = False
                        elif affClient.name == "affordancenet_context":
                            if obj == "objectness":
                                resultList[obj_idx][1] += 1 # TP
                                fail = False
                            else:
                                resultList[obj_idx][2] += 1 # FP
                                fail = False

            if fail:
                if score >= thresh:
                    resultList[obj_idx][2] += 1 # FP
                    fail = False
            if fail:
                resultList[obj_idx][3] += 1 # FN

        if affClient.name == "affordancenet_context":
            resultList[1][3] += max(0, (gt_count - no_preds))

    # precision = true positives / true positives + false positives
    # recall = true positives / true positives + false negatives
    # compute object specific recall and precision
    thresh_precision, thresh_recall = [], []
    for key, obj_id in OBJ_NAME_TO_ID.items():

        _, TP, FP, FN = resultList[obj_id]
        print(obj_id, TP, FP, FN, TP + FP + FN)
        if TP + FP + FN != 0:
            try:
                precision = TP / (TP + FP)
            except:
                precision = 1
            try:
                recall = TP / (TP + FN)
            except:
                recall = 1

            thresh_precision.append([obj_id, thresh, precision])
            thresh_recall.append([obj_id, thresh, recall])

    precisions.append(thresh_precision)
    recalls.append(thresh_recall)

unique_obj_idx = []
no_objects = len(precisions[0])
p_sum_overall = 0
p_list_overall = np.zeros(11)
for i in range(no_objects):
    p_sum = 0
    p_list, r_list = [], []
    for count, (p, r) in enumerate(zip(precisions, recalls)):
        p_list.append(p[i][2])
        r_list.append(r[i][2])

    p_list.reverse()
    r_list.reverse()
    p_arr = np.array(p_list)
    r_arr = np.array(r_list)
    for j in range(len(p_list)):
        if j == 0:
            p_sum += 1
        else:
            r_sample = (1 / args.resolution)*j
            r_idx = r_arr >= r_sample
            try:
                p = p_arr[r_idx].max()
            except:
                p = 0
            p_sum += p

            p_list_overall[j] += p
    p_sum_overall += p_sum / args.resolution

p_list_overall = np.flip(p_list_overall, 0) / no_objects
print(p_list_overall)

mAP = p_sum_overall / no_objects
print(" -------------------------------")
print("mAP score: ", mAP)
print(" -------------------------------")


plt.plot(np.linspace(0, 1, args.resolution), p_list_overall)
plt.title("Overall precision recall curve " + affClient.name.replace('_','-').capitalize())
plt.xlabel('Recall', fontSize=14)
plt.ylabel('Precision', fontSize=14)
plt.ylim([0, 1])
plt.grid()
#y = [0, 0.2, 0.4, 0.6, 0.8, 1.0]
#plt.yticks(list(range(len(y))), y)
plt.show()
