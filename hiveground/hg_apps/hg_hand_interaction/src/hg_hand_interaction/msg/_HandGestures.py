"""autogenerated by genpy from hg_hand_interaction/HandGestures.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import hg_hand_interaction.msg
import std_msgs.msg

class HandGestures(genpy.Message):
  _md5sum = "6ec8d74eee60d32f06423a7c0f42baa1"
  _type = "hg_hand_interaction/HandGestures"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
HandGesture[] gestures

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
MSG: hg_hand_interaction/HandGesture
int8 type
float64 var1

#request constants
int8 NOT_DETECTED = 0
int8 SWEEP_UP_ONE_HAND = 1
int8 SWEEP_DOWN_ONE_HAND = 2
int8 SWEEP_LEFT_ONE_HAND = 3
int8 SWEEP_RIGHT_ONE_HAND = 4
int8 SWEEP_FORWARD_ONE_HAND = 5
int8 SWEEP_BACKWARD_ONE_HAND = 6

int8 SWEEP_UP_TWO_HAND = 7
int8 SWEEP_DOWN_TWO_HAND = 8
int8 SWEEP_LEFT_TWO_HAND = 9
int8 SWEEP_RIGHT_TWO_HAND = 10
int8 SWEEP_FORWARD_TWO_HAND = 11
int8 SWEEP_BACKWARD_TWO_HAND = 12
int8 SWEEP_OPEN_TWO_HAND = 13
int8 SWEEP_CLOSE_TWO_HAND = 14

int8 PUSH_PULL_XP = 15
int8 PUSH_PULL_XN = 16 
int8 PUSH_PULL_YP = 17
int8 PUSH_PULL_YN = 18
int8 PUSH_PULL_ZP = 19
int8 PUSH_PULL_ZN = 20

int8 PUSH_PULL_RXP = 21
int8 PUSH_PULL_RXN = 22
int8 PUSH_PULL_RYP = 23
int8 PUSH_PULL_RYN = 24
int8 PUSH_PULL_RZP = 25
int8 PUSH_PULL_RZN = 26
"""
  __slots__ = ['header','gestures']
  _slot_types = ['std_msgs/Header','hg_hand_interaction/HandGesture[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,gestures

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(HandGestures, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.gestures is None:
        self.gestures = []
    else:
      self.header = std_msgs.msg.Header()
      self.gestures = []

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
      length = len(self.gestures)
      buff.write(_struct_I.pack(length))
      for val1 in self.gestures:
        _x = val1
        buff.write(_struct_bd.pack(_x.type, _x.var1))
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
      if self.gestures is None:
        self.gestures = None
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
      self.gestures = []
      for i in range(0, length):
        val1 = hg_hand_interaction.msg.HandGesture()
        _x = val1
        start = end
        end += 9
        (_x.type, _x.var1,) = _struct_bd.unpack(str[start:end])
        self.gestures.append(val1)
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
      length = len(self.gestures)
      buff.write(_struct_I.pack(length))
      for val1 in self.gestures:
        _x = val1
        buff.write(_struct_bd.pack(_x.type, _x.var1))
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
      if self.gestures is None:
        self.gestures = None
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
      self.gestures = []
      for i in range(0, length):
        val1 = hg_hand_interaction.msg.HandGesture()
        _x = val1
        start = end
        end += 9
        (_x.type, _x.var1,) = _struct_bd.unpack(str[start:end])
        self.gestures.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_bd = struct.Struct("<bd")
_struct_3I = struct.Struct("<3I")
