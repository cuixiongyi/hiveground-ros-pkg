; Auto-generated. Do not edit!


(cl:in-package hg_cartesian_trajectory-srv)


;//! \htmlinclude HgCartesianTrajectory-request.msg.html

(cl:defclass <HgCartesianTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (poses
    :reader poses
    :initarg :poses
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose)))
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass HgCartesianTrajectory-request (<HgCartesianTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HgCartesianTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HgCartesianTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hg_cartesian_trajectory-srv:<HgCartesianTrajectory-request> is deprecated: use hg_cartesian_trajectory-srv:HgCartesianTrajectory-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HgCartesianTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_cartesian_trajectory-srv:header-val is deprecated.  Use hg_cartesian_trajectory-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'poses-val :lambda-list '(m))
(cl:defmethod poses-val ((m <HgCartesianTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_cartesian_trajectory-srv:poses-val is deprecated.  Use hg_cartesian_trajectory-srv:poses instead.")
  (poses m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <HgCartesianTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_cartesian_trajectory-srv:type-val is deprecated.  Use hg_cartesian_trajectory-srv:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<HgCartesianTrajectory-request>)))
    "Constants for message type '<HgCartesianTrajectory-request>"
  '((:SIMPLE_IK . 0)
    (:LINE . 1)
    (:CIRCLE . 2)
    (:PATH . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'HgCartesianTrajectory-request)))
    "Constants for message type 'HgCartesianTrajectory-request"
  '((:SIMPLE_IK . 0)
    (:LINE . 1)
    (:CIRCLE . 2)
    (:PATH . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HgCartesianTrajectory-request>) ostream)
  "Serializes a message object of type '<HgCartesianTrajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poses))
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HgCartesianTrajectory-request>) istream)
  "Deserializes a message object of type '<HgCartesianTrajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HgCartesianTrajectory-request>)))
  "Returns string type for a service object of type '<HgCartesianTrajectory-request>"
  "hg_cartesian_trajectory/HgCartesianTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HgCartesianTrajectory-request)))
  "Returns string type for a service object of type 'HgCartesianTrajectory-request"
  "hg_cartesian_trajectory/HgCartesianTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HgCartesianTrajectory-request>)))
  "Returns md5sum for a message object of type '<HgCartesianTrajectory-request>"
  "a3e4d8bb4e92bdb76cb1807a6f2bc3f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HgCartesianTrajectory-request)))
  "Returns md5sum for a message object of type 'HgCartesianTrajectory-request"
  "a3e4d8bb4e92bdb76cb1807a6f2bc3f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HgCartesianTrajectory-request>)))
  "Returns full string definition for message of type '<HgCartesianTrajectory-request>"
  (cl:format cl:nil "~%Header header~%geometry_msgs/Pose[] poses~%int8 type~%~%int8 SIMPLE_IK=0~%int8 LINE=1~%int8 CIRCLE=2~%int8 PATH=3~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HgCartesianTrajectory-request)))
  "Returns full string definition for message of type 'HgCartesianTrajectory-request"
  (cl:format cl:nil "~%Header header~%geometry_msgs/Pose[] poses~%int8 type~%~%int8 SIMPLE_IK=0~%int8 LINE=1~%int8 CIRCLE=2~%int8 PATH=3~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HgCartesianTrajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HgCartesianTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'HgCartesianTrajectory-request
    (cl:cons ':header (header msg))
    (cl:cons ':poses (poses msg))
    (cl:cons ':type (type msg))
))
;//! \htmlinclude HgCartesianTrajectory-response.msg.html

(cl:defclass <HgCartesianTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:integer
    :initform 0))
)

(cl:defclass HgCartesianTrajectory-response (<HgCartesianTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HgCartesianTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HgCartesianTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hg_cartesian_trajectory-srv:<HgCartesianTrajectory-response> is deprecated: use hg_cartesian_trajectory-srv:HgCartesianTrajectory-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <HgCartesianTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_cartesian_trajectory-srv:success-val is deprecated.  Use hg_cartesian_trajectory-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HgCartesianTrajectory-response>) ostream)
  "Serializes a message object of type '<HgCartesianTrajectory-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'success)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'success)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'success)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'success)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HgCartesianTrajectory-response>) istream)
  "Deserializes a message object of type '<HgCartesianTrajectory-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'success)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'success)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'success)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'success)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HgCartesianTrajectory-response>)))
  "Returns string type for a service object of type '<HgCartesianTrajectory-response>"
  "hg_cartesian_trajectory/HgCartesianTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HgCartesianTrajectory-response)))
  "Returns string type for a service object of type 'HgCartesianTrajectory-response"
  "hg_cartesian_trajectory/HgCartesianTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HgCartesianTrajectory-response>)))
  "Returns md5sum for a message object of type '<HgCartesianTrajectory-response>"
  "a3e4d8bb4e92bdb76cb1807a6f2bc3f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HgCartesianTrajectory-response)))
  "Returns md5sum for a message object of type 'HgCartesianTrajectory-response"
  "a3e4d8bb4e92bdb76cb1807a6f2bc3f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HgCartesianTrajectory-response>)))
  "Returns full string definition for message of type '<HgCartesianTrajectory-response>"
  (cl:format cl:nil "uint32 success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HgCartesianTrajectory-response)))
  "Returns full string definition for message of type 'HgCartesianTrajectory-response"
  (cl:format cl:nil "uint32 success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HgCartesianTrajectory-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HgCartesianTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'HgCartesianTrajectory-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'HgCartesianTrajectory)))
  'HgCartesianTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'HgCartesianTrajectory)))
  'HgCartesianTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HgCartesianTrajectory)))
  "Returns string type for a service object of type '<HgCartesianTrajectory>"
  "hg_cartesian_trajectory/HgCartesianTrajectory")