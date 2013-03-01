"""autogenerated by genpy from hg_user_interaction/Gesture.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class Gesture(genpy.Message):
  _md5sum = "355d3ec6c605417c6e0a093dc3cfb48c"
  _type = "hg_user_interaction/Gesture"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#gesture
uint32 GESTURE_NOT_DETECTED = 0

#hand
uint32 GESTURE_HAND_SWEEP = 1
uint32 GESTURE_HAND_PUSH_PULL = 2

#body
uint32 GESTURE_BODY_MOVE = 101
uint32 GESTURE_BODY_TWIST = 102
uint32 GESTURE_BODY_LEAN = 103

#hand(s)
uint32 HAND_ONE = 1  
uint32 HAND_TWO = 2

#direction flags
uint32 DIR_X_POS = 1
uint32 DIR_X_NEG = 2
uint32 DIR_Y_POS = 3
uint32 DIR_Y_NEG = 4
uint32 DIR_Z_POS = 5
uint32 DIR_Z_NEG = 6


uint32 type
uint32 hand_count
uint32 direction

#Only used if the type specified has some use of them 
float64[] vars

#Only used if the type specified has some use of them
geometry_msgs/Vector3[] vectors

#Only used if the type specified has some use of them
geometry_msgs/Transform transforms



================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

"""
  # Pseudo-constants
  GESTURE_NOT_DETECTED = 0
  GESTURE_HAND_SWEEP = 1
  GESTURE_HAND_PUSH_PULL = 2
  GESTURE_BODY_MOVE = 101
  GESTURE_BODY_TWIST = 102
  GESTURE_BODY_LEAN = 103
  HAND_ONE = 1
  HAND_TWO = 2
  DIR_X_POS = 1
  DIR_X_NEG = 2
  DIR_Y_POS = 3
  DIR_Y_NEG = 4
  DIR_Z_POS = 5
  DIR_Z_NEG = 6

  __slots__ = ['type','hand_count','direction','vars','vectors','transforms']
  _slot_types = ['uint32','uint32','uint32','float64[]','geometry_msgs/Vector3[]','geometry_msgs/Transform']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       type,hand_count,direction,vars,vectors,transforms

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Gesture, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.type is None:
        self.type = 0
      if self.hand_count is None:
        self.hand_count = 0
      if self.direction is None:
        self.direction = 0
      if self.vars is None:
        self.vars = []
      if self.vectors is None:
        self.vectors = []
      if self.transforms is None:
        self.transforms = geometry_msgs.msg.Transform()
    else:
      self.type = 0
      self.hand_count = 0
      self.direction = 0
      self.vars = []
      self.vectors = []
      self.transforms = geometry_msgs.msg.Transform()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.type, _x.hand_count, _x.direction))
      length = len(self.vars)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.vars))
      length = len(self.vectors)
      buff.write(_struct_I.pack(length))
      for val1 in self.vectors:
        _x = val1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_struct_7d.pack(_x.transforms.translation.x, _x.transforms.translation.y, _x.transforms.translation.z, _x.transforms.rotation.x, _x.transforms.rotation.y, _x.transforms.rotation.z, _x.transforms.rotation.w))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.vectors is None:
        self.vectors = None
      if self.transforms is None:
        self.transforms = geometry_msgs.msg.Transform()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.type, _x.hand_count, _x.direction,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.vars = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.vectors = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Vector3()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.vectors.append(val1)
      _x = self
      start = end
      end += 56
      (_x.transforms.translation.x, _x.transforms.translation.y, _x.transforms.translation.z, _x.transforms.rotation.x, _x.transforms.rotation.y, _x.transforms.rotation.z, _x.transforms.rotation.w,) = _struct_7d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.type, _x.hand_count, _x.direction))
      length = len(self.vars)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.vars.tostring())
      length = len(self.vectors)
      buff.write(_struct_I.pack(length))
      for val1 in self.vectors:
        _x = val1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_struct_7d.pack(_x.transforms.translation.x, _x.transforms.translation.y, _x.transforms.translation.z, _x.transforms.rotation.x, _x.transforms.rotation.y, _x.transforms.rotation.z, _x.transforms.rotation.w))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.vectors is None:
        self.vectors = None
      if self.transforms is None:
        self.transforms = geometry_msgs.msg.Transform()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.type, _x.hand_count, _x.direction,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.vars = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.vectors = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Vector3()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.vectors.append(val1)
      _x = self
      start = end
      end += 56
      (_x.transforms.translation.x, _x.transforms.translation.y, _x.transforms.translation.z, _x.transforms.rotation.x, _x.transforms.rotation.y, _x.transforms.rotation.z, _x.transforms.rotation.w,) = _struct_7d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_7d = struct.Struct("<7d")
_struct_3d = struct.Struct("<3d")
