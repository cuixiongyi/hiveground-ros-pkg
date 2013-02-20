; Auto-generated. Do not edit!


(cl:in-package kinect_msgs-msg)


;//! \htmlinclude Skeletons.msg.html

(cl:defclass <Skeletons> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (skeletons
    :reader skeletons
    :initarg :skeletons
    :type (cl:vector kinect_msgs-msg:Skeleton)
   :initform (cl:make-array 0 :element-type 'kinect_msgs-msg:Skeleton :initial-element (cl:make-instance 'kinect_msgs-msg:Skeleton))))
)

(cl:defclass Skeletons (<Skeletons>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Skeletons>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Skeletons)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kinect_msgs-msg:<Skeletons> is deprecated: use kinect_msgs-msg:Skeletons instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Skeletons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect_msgs-msg:header-val is deprecated.  Use kinect_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'skeletons-val :lambda-list '(m))
(cl:defmethod skeletons-val ((m <Skeletons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect_msgs-msg:skeletons-val is deprecated.  Use kinect_msgs-msg:skeletons instead.")
  (skeletons m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Skeletons>)))
    "Constants for message type '<Skeletons>"
  '((:SKELETON_COUNT . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Skeletons)))
    "Constants for message type 'Skeletons"
  '((:SKELETON_COUNT . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Skeletons>) ostream)
  "Serializes a message object of type '<Skeletons>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'skeletons))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'skeletons))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Skeletons>) istream)
  "Deserializes a message object of type '<Skeletons>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'skeletons) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'skeletons)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kinect_msgs-msg:Skeleton))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Skeletons>)))
  "Returns string type for a message object of type '<Skeletons>"
  "kinect_msgs/Skeletons")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Skeletons)))
  "Returns string type for a message object of type 'Skeletons"
  "kinect_msgs/Skeletons")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Skeletons>)))
  "Returns md5sum for a message object of type '<Skeletons>"
  "fea4b07d293d1980dc796dc53353ba55")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Skeletons)))
  "Returns md5sum for a message object of type 'Skeletons"
  "fea4b07d293d1980dc796dc53353ba55")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Skeletons>)))
  "Returns full string definition for message of type '<Skeletons>"
  (cl:format cl:nil "Header header~%Skeleton[] skeletons~%~%int8 SKELETON_COUNT = 6~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: kinect_msgs/Skeleton~%int8 skeleton_tracking_state~%uint64 tracking_id~%uint64 enrollment_index~%uint64 user_index~%geometry_msgs/Transform position~%geometry_msgs/Transform[] skeleton_positions~%int8[] skeleton_position_tracking_state~%uint64 quality_flag~%~%int8 SKELETON_NOT_TRACKED = 0~%int8 SKELETON_POSITION_ONLY = 1~%int8 SKELETON_TRACKED = 2~%~%int8 SKELETON_POSITION_NOT_TRACKED = 0~%int8 SKELETON_POSITION_INFERRED = 1~%int8 SKELETON_POSITION_TRACKED = 2~%~%int8 SKELETON_POSITION_HIP_CENTER = 0~%int8 SKELETON_POSITION_SPINE = 1~%int8 SKELETON_POSITION_SHOULDER_CENTER = 2~%int8 SKELETON_POSITION_HEAD = 3~%int8 SKELETON_POSITION_SHOULDER_LEFT = 4~%int8 SKELETON_POSITION_ELBOW_LEFT = 5~%int8 SKELETON_POSITION_WRIST_LEFT = 6~%int8 SKELETON_POSITION_HAND_LEFT = 7~%int8 SKELETON_POSITION_SHOULDER_RIGHT = 8~%int8 SKELETON_POSITION_ELBOW_RIGHT = 9~%int8 SKELETON_POSITION_WRIST_RIGHT = 10~%int8 SKELETON_POSITION_HAND_RIGHT = 11~%int8 SKELETON_POSITION_HIP_LEFT = 12~%int8 SKELETON_POSITION_KNEE_LEFT = 13~%int8 SKELETON_POSITION_ANKLE_LEFT = 14~%int8 SKELETON_POSITION_FOOT_LEFT = 15~%int8 SKELETON_POSITION_HIP_RIGHT = 16~%int8 SKELETON_POSITION_KNEE_RIGHT = 17~%int8 SKELETON_POSITION_ANKLE_RIGHT = 18~%int8 SKELETON_POSITION_FOOT_RIGHT = 19~%int8 SKELETON_POSITION_COUNT = 20~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Skeletons)))
  "Returns full string definition for message of type 'Skeletons"
  (cl:format cl:nil "Header header~%Skeleton[] skeletons~%~%int8 SKELETON_COUNT = 6~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: kinect_msgs/Skeleton~%int8 skeleton_tracking_state~%uint64 tracking_id~%uint64 enrollment_index~%uint64 user_index~%geometry_msgs/Transform position~%geometry_msgs/Transform[] skeleton_positions~%int8[] skeleton_position_tracking_state~%uint64 quality_flag~%~%int8 SKELETON_NOT_TRACKED = 0~%int8 SKELETON_POSITION_ONLY = 1~%int8 SKELETON_TRACKED = 2~%~%int8 SKELETON_POSITION_NOT_TRACKED = 0~%int8 SKELETON_POSITION_INFERRED = 1~%int8 SKELETON_POSITION_TRACKED = 2~%~%int8 SKELETON_POSITION_HIP_CENTER = 0~%int8 SKELETON_POSITION_SPINE = 1~%int8 SKELETON_POSITION_SHOULDER_CENTER = 2~%int8 SKELETON_POSITION_HEAD = 3~%int8 SKELETON_POSITION_SHOULDER_LEFT = 4~%int8 SKELETON_POSITION_ELBOW_LEFT = 5~%int8 SKELETON_POSITION_WRIST_LEFT = 6~%int8 SKELETON_POSITION_HAND_LEFT = 7~%int8 SKELETON_POSITION_SHOULDER_RIGHT = 8~%int8 SKELETON_POSITION_ELBOW_RIGHT = 9~%int8 SKELETON_POSITION_WRIST_RIGHT = 10~%int8 SKELETON_POSITION_HAND_RIGHT = 11~%int8 SKELETON_POSITION_HIP_LEFT = 12~%int8 SKELETON_POSITION_KNEE_LEFT = 13~%int8 SKELETON_POSITION_ANKLE_LEFT = 14~%int8 SKELETON_POSITION_FOOT_LEFT = 15~%int8 SKELETON_POSITION_HIP_RIGHT = 16~%int8 SKELETON_POSITION_KNEE_RIGHT = 17~%int8 SKELETON_POSITION_ANKLE_RIGHT = 18~%int8 SKELETON_POSITION_FOOT_RIGHT = 19~%int8 SKELETON_POSITION_COUNT = 20~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Skeletons>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'skeletons) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Skeletons>))
  "Converts a ROS message object to a list"
  (cl:list 'Skeletons
    (cl:cons ':header (header msg))
    (cl:cons ':skeletons (skeletons msg))
))
