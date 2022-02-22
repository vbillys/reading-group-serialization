#!/usr/bin/env python3

import cv2
import numpy as np

import sys, time
import os
import io

import open3d as o3d
import argparse
from transforms3d.affines import compose
from transforms3d.euler import euler2mat


from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.util import duration

import ExampleIdlData

# Create a DomainParticipant, your entrypoint to DDS
# The default domain id is 0.
dp = DomainParticipant()

# Create a Topic with topic name "Hello" and as datatype "HelloWorld" structs.
tp = Topic(dp, "example_topic", ExampleIdlData.Msg)

# Create a DataWriter that can send structs on the "Hello" topic
dw = DataWriter(dp, tp)

# Create a DataReader that can receive structs on the "Hello" topic
dr = DataReader(dp, tp)

# --- Normal Numpy Matrix

ht_matrix = compose([20, 30, 40], euler2mat(
    np.pi/2, np.pi/2, np.pi/2), np.ones(3), np.zeros(3))
#  the following is optional store as F-order to avoid transposing on the receiver side
# ht_matrix = np.asfortranarray(ht_matrix)
print(f'homogeneous transformation matrix:\n{ht_matrix}')
matStream = io.BytesIO()
np.save(matStream, ht_matrix)

# ---


sample = ExampleIdlData.Msg(1,'hi', list(matStream.getbuffer()), [], [])
dw.write(sample)

time.sleep(1.0)