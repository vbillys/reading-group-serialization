#!/usr/bin/env python3

import cv2
import numpy as np

import sys
import os
import io

import open3d as o3d
import argparse



script_path = os.path.dirname(os.path.realpath(__file__))
image_filepath = script_path + '/../data/car.jpeg'

print(f'opening image: {image_filepath}')

img = cv2.imread(image_filepath)
cv2.imshow('car', img)
cv2.waitKey()

pcd_filepath = script_path + '/../data/bunny.pcd'
pcd = o3d.io.read_point_cloud(pcd_filepath)
pcd_numpy = np.asarray(pcd.points)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])

output = io.BytesIO()
np.savez(output, img=img, pcd_numpy=pcd_numpy)
npzoutput = io.FileIO('data.npz', 'w')
np.savez(npzoutput, img=img, pcd_numpy=pcd_numpy)

# print(f'here is the serialized content (.npz): \n{npzoutput.getvalue()}')
print(
    f'the image in python is of type: {type(img)} that contains {img.dtype.name}')

parser = argparse.ArgumentParser()
parser.add_argument("-o", "--output", action="store_true",
                    help="If set, spits out the output to stderr")
options = parser.parse_args()
if options.output:
    print(f'here is the serialized content (.npz): \n{output.getvalue()}')