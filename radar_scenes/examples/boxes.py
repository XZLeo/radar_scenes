'''
Generate bounding box as the minimum exterior rectangle of a cluster by searching.


'''
import os
from ctypes import alignment
from turtle import shape
from frame import get_frames, get_timestamps, plot_frames
import  matplotlib.pyplot as plt
from numpy import ndarray, max, min, abs, zeros, shape, sin, cos, pi, where
from typing import List, Tuple

from radar_scenes.sequence import Sequence

# bounding box type
AABB = Tuple[float, float, float, float]
OBB = Tuple[float, float, float, float, float]

def get_AABB(cluster: ndarray)-> AABB:
    '''
    get axis algned bounding boxes from a frame by finding the max, min x and y
    param cluster: numpy array with the first row as x coordinate, second row as y coordinate
    return aligned_box: <x, y, w, h> as YOLO convention
    '''
    max_x = max(cluster, axis=0)
    max_y = max(cluster, axis=1)
    min_x = min(cluster, axis=0)
    min_y = min(cluster, axis=1)
    w = abs(max_y-min_y)
    h = abs(max_x-min_x) # becuase of vehicle coordinate system
    x = (max_x + min_x) / 2
    y = (max_y + min_y) / 2
    aligned_box = (x, y, w, h)
    return aligned_box

def rotate_cluster(cluster: ndarray, theta: float)-> ndarray:
    '''
    rotate the cluster by a given degree around the origin (clockwise), theta, ouput new coordinates
    param theta: in radiant
    '''
    rotated_cluster = zeros(shape(cluster))
    rotated_cluster[0, :] = cluster[0, :]*cos(theta) - cluster[1, :]*sin(theta)
    rotated_cluster[1, :] = cluster[0, :]*sin(theta) + cluster[1, :]*cos(theta)
    return rotated_cluster

def search_OBB(cluster: ndarray, resolution: float)->OBB:
    '''
    get oriented bounding boxes by searching for the minimum area
    param resolution: control the number of searches
    return oriented_box: <x, y, w, h, theta>
    '''
    min_area = 100
    min_theta = 0
    oriented_box = (0.0, 0.0, 0.0, 0.0, 0.0)
    # search for minimum rectnagle
    for theta in range(0, pi, resolution):
        rotated_cluster = rotate_cluster(cluster, theta)
        aligned_box = get_AABB(rotate_cluster)
        area = aligned_box[2] * aligned_box[3]
        if area < min_area:
            min_area = area
            min_theta = theta
            oriented_box = aligned_box
    # restore to origin coordinate by rotating counter-clockwise
    oriented_box[0] = oriented_box[0]*cos(min_theta) + oriented_box[1]*sin(min_theta)
    oriented_box[1] = -oriented_box[0]*sin(min_theta) + oriented_box[1]*cos(min_theta)
    oriented_box[4] =  -theta / pi # normalized to -1~ 1
    return oriented_box     


def visualize_boxes_cloud()->None:
    plot_frames()
    pt
    return


def main()->None:
    # take one secene from one sequence (write the data loader with pytorch to generate all grid maps!)
    # extract a frame, i.e., 4 continuous scenes from the start time, for DBSCAN
    path_to_dataset = "../RadarScenes"
    # Define the *.json file from which data should be loaded
    filename = os.path.join(path_to_dataset, "data", "sequence_137", "scenes.json")
    sequence = Sequence.from_json(filename)
    timestamps = get_timestamps(sequence)
    cur_idx = 0
    radar_data = get_frames(sequence, cur_idx, timestamps)
    # extract cluster 
    track_ids = set(radar_data["track_id"])
    for tr_id in track_ids:
        if len(tr_id) == 0: # no tracked objects
            continue
        idx = where(radar_data["track_id"] == tr_id)[0] # get the index of non-empty track id
        if len(idx) < 2: # only one point with same tr_id
            # ????? should boxes be generated????
            continue
        points = zeros((len(idx), 2))
        points[:, 0] = radar_data[idx]["x_cc"]
        points[:, 1] = radar_data[idx]["y_cc"]
        if len(tr_id) > 2:
            
    return



    
if __name__ == '__main__':
    main()