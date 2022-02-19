#!/usr/bin/env python3

import cv2
import numpy as np

import sys
import os
import io
import argparse

script_path = os.path.dirname(os.path.realpath(__file__))
image_filepath = script_path + '/../data/car.jpeg'

print(f'opening image: {image_filepath}')

img = cv2.imread(image_filepath)
cv2.imshow('car', img)
cv2.waitKey()

print(f'the image in python is of type: {type(img)}')

output = io.BytesIO()
np.save(output, img)

print(f'here is the serialized content (.npy): \n{output.getvalue()}')

parser = argparse.ArgumentParser()
parser.add_argument("-o", "--output", action="store_true",
                    help="If set, spits out the output to stderr")
options = parser.parse_args()

if options.output:
    sys.stderr.buffer.write(output.getvalue())
