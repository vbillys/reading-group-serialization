#!/usr/bin/env python3

import cv2
import numpy as np

import sys
import os
import io

import open3d as o3d
import argparse


parser = argparse.ArgumentParser()
parser.add_argument("-n", "--npzfile", type=str,
                    help="Full path to data file", required=True)
options = parser.parse_args()

npzdata = np.load(options.npzfile)

# --- Normal Numpy Matrix

print(f"loaded Homogeneous Transformation: \n{npzdata['ht_matrix']}")

# ---

# --- Image (OpenCV) Matrix

print(f"Image matrix: shape {npzdata['img'].shape} \n{npzdata['img']}")
cv2.imshow('car', npzdata['img'])
cv2.waitKey()

# ---

# --- PointCloud (PCL) Matrix

print(f"loaded point cloud: \n{npzdata['pcd_numpy']}")
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(npzdata['pcd_numpy'])
o3d.visualization.draw_geometries([pcd])

# ---
