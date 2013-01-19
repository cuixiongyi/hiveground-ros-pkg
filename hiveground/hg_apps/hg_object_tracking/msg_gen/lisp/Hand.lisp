; Auto-generated. Do not edit!


(cl:in-package hg_object_tracking-msg)


;//! \htmlinclude Hand.msg.html

(cl:defclass <Hand> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (seq
    :reader seq
    :initarg :seq
    :type cl:integer
    :initform 0)
   (thumb
    :reader thumb
    :initarg :thumb
    :type cl:integer
    :initform 0)
   (left
    :reader left
    :initarg :left
    :type cl:boolean
    :initform cl:nil)
   (arm
    :reader arm
    :initarg :arm
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (palm
    :reader palm
    :initarg :palm
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform))
   (fingers
    :reader fingers
    :initarg :fingers
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (handcloud
    :reader handcloud
    :initarg :handcloud
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (state
    :reader state
    :initarg :state
    :type cl:string
    :initform ""))
)

(cl:defclass Hand (<Hand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Hand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Hand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hg_object_tracking-msg:<Hand> is deprecated: use hg_object_tracking-msg:Hand instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:stamp-val is deprecated.  Use hg_object_tracking-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'seq-val :lambda-list '(m))
(cl:defmethod seq-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:seq-val is deprecated.  Use hg_object_tracking-msg:seq instead.")
  (seq m))

(cl:ensure-generic-function 'thumb-val :lambda-list '(m))
(cl:defmethod thumb-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:thumb-val is deprecated.  Use hg_object_tracking-msg:thumb instead.")
  (thumb m))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:left-val is deprecated.  Use hg_object_tracking-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'arm-val :lambda-list '(m))
(cl:defmethod arm-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:arm-val is deprecated.  Use hg_object_tracking-msg:arm instead.")
  (arm m))

(cl:ensure-generic-function 'palm-val :lambda-list '(m))
(cl:defmethod palm-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:palm-val is deprecated.  Use hg_object_tracking-msg:palm instead.")
  (palm m))

(cl:ensure-generic-function 'fingers-val :lambda-list '(m))
(cl:defmethod fingers-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:fingers-val is deprecated.  Use hg_object_tracking-msg:fingers instead.")
  (fingers m))

(cl:ensure-generic-function 'handcloud-val :lambda-list '(m))
(cl:defmethod handcloud-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:handcloud-val is deprecated.  Use hg_object_tracking-msg:handcloud instead.")
  (handcloud m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:state-val is deprecated.  Use hg_object_tracking-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hand>) ostream)
  "Serializes a message object of type '<Hand>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let* ((signed (cl:slot-value msg 'seq)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'thumb)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'arm) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'palm) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'fingers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'fingers))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handcloud) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'state))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hand>) istream)
  "Deserializes a message object of type '<Hand>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'seq) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'thumb) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'left) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'arm) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'palm) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'fingers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'fingers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handcloud) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Hand>)))
  "Returns string type for a message object of type '<Hand>"
  "hg_object_tracking/Hand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hand)))
  "Returns string type for a message object of type 'Hand"
  "hg_object_tracking/Hand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Hand>)))
  "Returns md5sum for a message object of type '<Hand>"
  "7cef0afb7e7be00fa897ba75c86d7ea4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hand)))
  "Returns md5sum for a message object of type 'Hand"
  "7cef0afb7e7be00fa897ba75c86d7ea4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hand>)))
  "Returns full string definition for message of type '<Hand>"
  (cl:format cl:nil "time stamp~%int32 seq    # seq is used for tracking the hand - ~%			 # if the seq number is the same, you can assume that the hand is the same as previously seen~%int32 thumb~%bool left    # if this hand is the left hand~%geometry_msgs/Point arm~%geometry_msgs/Transform palm~%geometry_msgs/Point[] fingers~%sensor_msgs/PointCloud2 handcloud~%#Possibilities for state variable:~%# open - open palm, usually five fingers~%# grip - fingers curled forward~%# paddle -  fingers together and straight~%# fist   ~%string state~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hand)))
  "Returns full string definition for message of type 'Hand"
  (cl:format cl:nil "time stamp~%int32 seq    # seq is used for tracking the hand - ~%			 # if the seq number is the same, you can assume that the hand is the same as previously seen~%int32 thumb~%bool left    # if this hand is the left hand~%geometry_msgs/Point arm~%geometry_msgs/Transform palm~%geometry_msgs/Point[] fingers~%sensor_msgs/PointCloud2 handcloud~%#Possibilities for state variable:~%# open - open palm, usually five fingers~%# grip - fingers curled forward~%# paddle -  fingers together and straight~%# fist   ~%string state~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hand>))
  (cl:+ 0
     8
     4
     4
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'arm))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'palm))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'fingers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handcloud))
     4 (cl:length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hand>))
  "Converts a ROS message object to a list"
  (cl:list 'Hand
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':seq (seq msg))
    (cl:cons ':thumb (thumb msg))
    (cl:cons ':left (left msg))
    (cl:cons ':arm (arm msg))
    (cl:cons ':palm (palm msg))
    (cl:cons ':fingers (fingers msg))
    (cl:cons ':handcloud (handcloud msg))
    (cl:cons ':state (state msg))
))
