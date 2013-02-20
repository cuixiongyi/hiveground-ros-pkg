"""autogenerated by genpy from kinect_msgs/Skeletons.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import kinect_msgs.msg
import std_msgs.msg

class Skeletons(genpy.Message):
  _md5sum = "f026931349e1ac4bdfdd321ca25f41b9"
  _type = "kinect_msgs/Skeletons"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
Skeleton[] skeleton

int8 SKELETON_COUNT = 6

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: kinect_msgs/Skeleton
int8 skeleton_tracking_state
uint64 tracking_id
uint64 enrollment_index
uint64 user_index
geometry_msgs/Transform position
geometry_msgs/Transform[] skeleton_positions
int8[] skeleton_position_tracking_state
uint64 quality_flag

int8 SKELETON_NOT_TRACKED = 0
int8 SKELETON_POSITION_ONLY = 1
int8 SKELETON_TRACKED = 2

int8 SKELETON_POSITION_NOT_TRACKED = 0
int8 SKELETON_POSITION_INFERRED = 1
int8 SKELETON_POSITION_TRACKED = 2

int8 SKELETON_POSITION_HIP_CENTER = 0
int8 SKELETON_POSITION_SPINE = 1
int8 SKELETON_POSITION_SHOULDER_CENTER = 2
int8 SKELETON_POSITION_HEAD = 3
int8 SKELETON_POSITION_SHOULDER_LEFT = 4
int8 SKELETON_POSITION_ELBOW_LEFT = 5
int8 SKELETON_POSITION_WRIST_LEFT = 6
int8 SKELETON_POSITION_HAND_LEFT = 7
int8 SKELETON_POSITION_SHOULDER_RIGHT = 8
int8 SKELETON_POSITION_ELBOW_RIGHT = 9
int8 SKELETON_POSITION_WRIST_RIGHT = 10
int8 SKELETON_POSITION_HAND_RIGHT = 11
int8 SKELETON_POSITION_HIP_LEFT = 12
int8 SKELETON_POSITION_KNEE_LEFT = 13
int8 SKELETON_POSITION_ANKLE_LEFT = 14
int8 SKELETON_POSITION_FOOT_LEFT = 15
int8 SKELETON_POSITION_HIP_RIGHT = 16
int8 SKELETON_POSITION_KNEE_RIGHT = 17
int8 SKELETON_POSITION_ANKLE_RIGHT = 18
int8 SKELETON_POSITION_FOOT_RIGHT = 19
int8 SKELETON_POSITION_COUNT = 20

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

"""
  # Pseudo-constants
  SKELETON_COUNT = 6

  __slots__ = ['header','skeleton']
  _slot_types = ['std_msgs/Header','kinect_msgs/Skeleton[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,skeleton

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Skeletons, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.skeleton is None:
        self.skeleton = []
    else:
      self.header = std_msgs.msg.Header()
      self.skeleton = []

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.skeleton)
      buff.write(_struct_I.pack(length))
      for val1 in self.skeleton:
        _x = val1
        buff.write(_struct_b3Q.pack(_x.skeleton_tracking_state, _x.tracking_id, _x.enrollment_index, _x.user_index))
        _v1 = val1.position
        _v2 = _v1.translation
        _x = _v2
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v3 = _v1.rotation
        _x = _v3
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        length = len(val1.skeleton_positions)
        buff.write(_struct_I.pack(length))
        for val2 in val1.skeleton_positions:
          _v4 = val2.translation
          _x = _v4
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
          _v5 = val2.rotation
          _x = _v5
          buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        length = len(val1.skeleton_position_tracking_state)
        buff.write(_struct_I.pack(length))
        pattern = '<%sb'%length
        buff.write(struct.pack(pattern, *val1.skeleton_position_tracking_state))
        buff.write(_struct_Q.pack(val1.quality_flag))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.skeleton is None:
        self.skeleton = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.skeleton = []
      for i in range(0, length):
        val1 = kinect_msgs.msg.Skeleton()
        _x = val1
        start = end
        end += 25
        (_x.skeleton_tracking_state, _x.tracking_id, _x.enrollment_index, _x.user_index,) = _struct_b3Q.unpack(str[start:end])
        _v6 = val1.position
        _v7 = _v6.translation
        _x = _v7
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v8 = _v6.rotation
        _x = _v8
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.skeleton_positions = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Transform()
          _v9 = val2.translation
          _x = _v9
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          _v10 = val2.rotation
          _x = _v10
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
          val1.skeleton_positions.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sb'%length
        start = end
        end += struct.calcsize(pattern)
        val1.skeleton_position_tracking_state = struct.unpack(pattern, str[start:end])
        start = end
        end += 8
        (val1.quality_flag,) = _struct_Q.unpack(str[start:end])
        self.skeleton.append(val1)
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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.skeleton)
      buff.write(_struct_I.pack(length))
      for val1 in self.skeleton:
        _x = val1
        buff.write(_struct_b3Q.pack(_x.skeleton_tracking_state, _x.tracking_id, _x.enrollment_index, _x.user_index))
        _v11 = val1.position
        _v12 = _v11.translation
        _x = _v12
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v13 = _v11.rotation
        _x = _v13
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        length = len(val1.skeleton_positions)
        buff.write(_struct_I.pack(length))
        for val2 in val1.skeleton_positions:
          _v14 = val2.translation
          _x = _v14
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
          _v15 = val2.rotation
          _x = _v15
          buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        length = len(val1.skeleton_position_tracking_state)
        buff.write(_struct_I.pack(length))
        pattern = '<%sb'%length
        buff.write(val1.skeleton_position_tracking_state.tostring())
        buff.write(_struct_Q.pack(val1.quality_flag))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.skeleton is None:
        self.skeleton = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.skeleton = []
      for i in range(0, length):
        val1 = kinect_msgs.msg.Skeleton()
        _x = val1
        start = end
        end += 25
        (_x.skeleton_tracking_state, _x.tracking_id, _x.enrollment_index, _x.user_index,) = _struct_b3Q.unpack(str[start:end])
        _v16 = val1.position
        _v17 = _v16.translation
        _x = _v17
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v18 = _v16.rotation
        _x = _v18
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.skeleton_positions = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Transform()
          _v19 = val2.translation
          _x = _v19
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          _v20 = val2.rotation
          _x = _v20
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
          val1.skeleton_positions.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sb'%length
        start = end
        end += struct.calcsize(pattern)
        val1.skeleton_position_tracking_state = numpy.frombuffer(str[start:end], dtype=numpy.int8, count=length)
        start = end
        end += 8
        (val1.quality_flag,) = _struct_Q.unpack(str[start:end])
        self.skeleton.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_Q = struct.Struct("<Q")
_struct_4d = struct.Struct("<4d")
_struct_3I = struct.Struct("<3I")
_struct_b3Q = struct.Struct("<b3Q")
_struct_3d = struct.Struct("<3d")
