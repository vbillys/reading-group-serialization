#!/usr/bin/env python3

import cv2
import numpy as np

import sys
import os
import io

import open3d as o3d
import argparse

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

# a = np.zeros(2,dtype=np.uint8)
# a = [0 ,0]

# a = io.BytesIO()
# sample = ExampleIdlData.Msg(1,'hi',a,a, a)
# dw.write(sample)

# Read samples from the network and print the data in the first one
# This should print "Hello, World!"
# sample = dr.read()[0]
# print(sample)

for sample in dr.take_iter(timeout=duration(seconds=2)):
    print(sample)