"""
  Generated by Eclipse Cyclone DDS idlc Python Backend
  Cyclone DDS IDL version: v0.9.0
  Module: ExampleIdlData
  IDL file: ExampleIdlData.idl

"""

from enum import auto
from typing import TYPE_CHECKING, Optional
from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

# root module import for resolving types
import ExampleIdlData

BytesArray = types.typedef['ExampleIdlData.BytesArray', types.sequence[types.uint8]]

@dataclass
@annotate.final
@annotate.autoid("sequential")
class Msg(idl.IdlStruct, typename="ExampleIdlData.Msg"):
    id: types.int32
    annotate.key("id")
    message: str
    payloadEigen: 'ExampleIdlData.BytesArray'
    payloadOpenCVImage: 'ExampleIdlData.BytesArray'
    payloadPCLPointCloud: 'ExampleIdlData.BytesArray'


