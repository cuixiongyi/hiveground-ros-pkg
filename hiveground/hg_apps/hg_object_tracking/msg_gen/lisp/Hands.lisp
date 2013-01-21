; Auto-generated. Do not edit!


(cl:in-package hg_object_tracking-msg)


;//! \htmlinclude Hands.msg.html

(cl:defclass <Hands> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (hands
    :reader hands
    :initarg :hands
    :type (cl:vector hg_object_tracking-msg:Hand)
   :initform (cl:make-array 0 :element-type 'hg_object_tracking-msg:Hand :initial-element (cl:make-instance 'hg_object_tracking-msg:Hand))))
)

(cl:defclass Hands (<Hands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Hands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Hands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hg_object_tracking-msg:<Hands> is deprecated: use hg_object_tracking-msg:Hands instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Hands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:header-val is deprecated.  Use hg_object_tracking-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'hands-val :lambda-list '(m))
(cl:defmethod hands-val ((m <Hands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:hands-val is deprecated.  Use hg_object_tracking-msg:hands instead.")
  (hands m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hands>) ostream)
  "Serializes a message object of type '<Hands>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'hands))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'hands))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hands>) istream)
  "Deserializes a message object of type '<Hands>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'hands) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'hands)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'hg_object_tracking-msg:Hand))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Hands>)))
  "Returns string type for a message object of type '<Hands>"
  "hg_object_tracking/Hands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hands)))
  "Returns string type for a message object of type 'Hands"
  "hg_object_tracking/Hands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Hands>)))
  "Returns md5sum for a message object of type '<Hands>"
  "b78f3ee919c8b10900946d2e5fe0a669")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hands)))
  "Returns md5sum for a message object of type 'Hands"
  "b78f3ee919c8b10900946d2e5fe0a669")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hands>)))
  "Returns full string definition for message of type '<Hands>"
  (cl:format cl:nil "Header header~%#if two hands are detected, if handedness is distinguished, left hand is first~%Hand[] hands~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: hg_object_tracking/Hand~%geometry_msgs/Vector3 arm_centroid~%geometry_msgs/Vector3 arm_eigen_value~%geometry_msgs/Vector3[] arm_eigen_vectors~%~%geometry_msgs/Vector3 hand_centroid~%geometry_msgs/Vector3 hand_eigen_value~%geometry_msgs/Vector3[] hand_eigen_vectors~%geometry_msgs/Point[] fingers~%~%#Possibilities for state variable:~%# open - open palm, usually five fingers~%# grip - fingers curled forward~%# paddle -  fingers together and straight~%# fist   ~%string state~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hands)))
  "Returns full string definition for message of type 'Hands"
  (cl:format cl:nil "Header header~%#if two hands are detected, if handedness is distinguished, left hand is first~%Hand[] hands~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: hg_object_tracking/Hand~%geometry_msgs/Vector3 arm_centroid~%geometry_msgs/Vector3 arm_eigen_value~%geometry_msgs/Vector3[] arm_eigen_vectors~%~%geometry_msgs/Vector3 hand_centroid~%geometry_msgs/Vector3 hand_eigen_value~%geometry_msgs/Vector3[] hand_eigen_vectors~%geometry_msgs/Point[] fingers~%~%#Possibilities for state variable:~%# open - open palm, usually five fingers~%# grip - fingers curled forward~%# paddle -  fingers together and straight~%# fist   ~%string state~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hands>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'hands) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hands>))
  "Converts a ROS message object to a list"
  (cl:list 'Hands
    (cl:cons ':header (header msg))
    (cl:cons ':hands (hands msg))
))
