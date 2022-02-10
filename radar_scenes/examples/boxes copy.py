'''
Generate bounding box as the minimum exterior rectangle of a cluster by searching.


'''
import os
from ctypes import alignment
from frame import get_frames, get_timestamps
import  matplotlib.pyplot as plt
import matplotlib.patches as pc
from numpy import ndarray, max, min, abs, zeros, where,array, cov, dot, transpose, arccos, cos, pi
from numpy.linalg import inv, eig
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
    max_x = max(cluster[0, :])
    max_y = max(cluster[1, :])
    min_x = min(cluster[0, :])
    min_y = min(cluster[1, :])
    w = abs(max_x-min_x)
    h = abs(max_y-min_y) # becuase of vehicle coordinate system  
    x = (max_x + min_x) / 2
    y = (max_y + min_y) / 2
    aligned_box = (x, y, w, h)
    return aligned_box

def get_OBB(cluster: ndarray): #-> Tuple(ndarray, OBB):
    cluster = transpose(cluster)
    ca = cov(cluster,y = None,rowvar = 0,bias = 1)

    v, vect = eig(ca)
    tvect = transpose(vect)
    #use the inverse of the eigenvectors as a rotation matrix and
    #rotate the points so they align with the x and y axes
    ar = dot(cluster, inv(tvect))
    # get the minimum and maximum x and y 
    mina = min(ar,axis=0)
    maxa = max(ar,axis=0)
    width, height = maxa - mina
    diff = (maxa - mina)*0.5
    # the center is just half way between the min and max xy
    center = mina + diff
    #get the 4 corners by subtracting and adding half the bounding boxes height and width to the center
    corners = array([center+[-diff[0],-diff[1]],center+[diff[0],-diff[1]],
                    center+[diff[0],diff[1]],center+[-diff[0],diff[1]],center+[-diff[0],-diff[1]]])
    #use the the eigenvectors as a rotation matrix and
    #rotate the corners and the centerback
    corners = dot(corners,tvect)
    center = dot(center,tvect)
    # transfer to YOLO format
    yawn = arccos(tvect[0, 0]) #radius
    yawn = yawn - 2*pi if yawn > pi else yawn # clockwise is negative  
    oriented_box = (center[0], center[1], width, height, yawn) 
    return corners, oriented_box


def visualize_AABB_cloud(points: ndarray, aligned_boxes: List[AABB])->None: 
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    col = [0, 0, 0, 1]
    #plot point cloud
    ax.plot(
        points[0, :], #y_cc
        points[1, :], #x_cc
        "o",
        markerfacecolor=tuple(col),
        markeredgecolor="k",
        markersize=2
    )
    # plot AABB
    for box in aligned_boxes:
        center_x, center_y, w, h = box
        bottom_left_horizon = center_x - w/2
        bottom_left_vertical = center_y - h/2 
        rect = pc.Rectangle((bottom_left_horizon, bottom_left_vertical), w, h, 
                            angle=0, fill=False, edgecolor = 'red',linewidth=2)  # has angle as input!!! the roration is around the bottom left piont
        ax.add_patch(rect) 
    ax.set_aspect('equal', adjustable='box')
    #ax.invert_xaxis() #y_cc
    plt.show()
    return


def visualize_OBB_cloud(points: ndarray, list_corners)->None:  # merge to one function?
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    col = [0, 0, 0, 1]
    #plot point cloud
    ax.plot(
        points[0, :], #y_cc
        points[1, :], #x_cc
        "o",
        markerfacecolor=tuple(col),
        markeredgecolor="k",
        markersize=2
    )
    # plot OBB                                                                                             
    for corners in list_corners:
        ax.plot(corners[:,0],corners[:,1],'-')  # draw boxes
    ax.set_aspect('equal', adjustable='box')
    #ax.invert_xaxis() #y_cc
    plt.show()
    return


def main()->None:
    # take one secene from one sequence (write the data loader with pytorch to generate all grid maps!)
    # extract a frame, i.e., 4 continuous scenes from the start time, for DBSCAN
    path_to_dataset = "../dataset/RadarScenes"
    # Define the *.json file from which data should be loaded
    filename = os.path.join(path_to_dataset, "data", "sequence_137", "scenes.json")
    sequence = Sequence.from_json(filename)
    timestamps = get_timestamps(sequence)
    # which scene to plot
    cur_idx = 0
    radar_data = get_frames(sequence, cur_idx, timestamps,  n_prev_frames=0 , n_next_frames=4)
    # extract cluster, each track_id should be a cluster 
    track_ids = set(radar_data["track_id"])
    aligned_boxes = []
    list_corners = []
    for tr_id in track_ids:
        if len(tr_id) == 0: # no tracked objects
            continue
        idx = where(radar_data["track_id"] == tr_id)[0] # get the index of non-empty track id
        if len(idx) < 2: # only one point with same tr_id, ignore it (this needs to be synchronized while creating grid maps!)
            continue
        # more than 2 pionts in a cluster
        points = zeros((2, len(idx)))
        points[0, :] = radar_data[idx]["x_cc"].reshape((1,len(idx)))
        points[1, :] = radar_data[idx]["y_cc"].reshape((1,len(idx)))
        # generate AABB
        aligned_boxes.append(get_AABB(points))
        corners, oriented_boxes = get_OBB(points)
        list_corners.append(corners)
    # visualize the frame
    point_cloud = array([radar_data['x_cc'], radar_data['y_cc']])
    #visualize_AABB_cloud(point_cloud, aligned_boxes)           
    visualize_OBB_cloud(point_cloud, list_corners)     
    return

    
if __name__ == '__main__':
    main()