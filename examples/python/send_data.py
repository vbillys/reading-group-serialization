#!/usr/bin/env python3

import cv2
import numpy as np

import sys
import os
import io

script_path = os.path.dirname(os.path.realpath(__file__))
image_filepath = script_path + '/../data/car.jpeg'

print(f'opening image: {image_filepath}')

img = cv2.imread(image_filepath)
cv2.imshow('car', img)
cv2.waitKey()


output = io.BytesIO()
np.save(output, img)
foutput = io.FileIO('data.bin','w')
np.save(foutput,img)

print(f'here is the serialized content (.npy): \n{output.getvalue()}')
print(f'the image in python is of type: {type(img)} that contains {img.dtype.name}')

