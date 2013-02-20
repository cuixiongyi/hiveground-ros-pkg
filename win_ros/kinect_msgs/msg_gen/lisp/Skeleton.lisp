; Auto-generated. Do not edit!


(cl:in-package kinect_msgs-msg)


;//! \htmlinclude Skeleton.msg.html

(cl:defclass <Skeleton> (roslisp-msg-protocol:ros-message)
  ((skeleton_tracking_state
    :reader skeleton_tracking_state
    :initarg :skeleton_tracking_state
    :type cl:fixnum
    :initform 0)
   (tracking_id
    :reader tracking_id
    :initarg :tracking_id
    :type cl:integer
    :initform 0)
   (enrollment_index
    :reader enrollment_index
    :initarg :enrollment_index
    :type cl:integer
    :initform 0)
   (user_index
    :reader user_index
    :initarg :user_index
    :type cl:integer
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform))
   (skeleton_positions
    :reader skeleton_positions
    :initarg :skeleton_positions
    :type (cl:vector geometry_msgs-msg:Transform)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Transform :initial-element (cl:make-instance 'geometry_msgs-msg:Transform)))
   (skeleton_position_tracking_state
    :reader skeleton_position_tracking_state
    :initarg :skeleton_position_tracking_state
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (quality_flag
    :reader quality_flag
    :initarg :quality_flag
    :type cl:integer
    :initform 0))
)

(cl:defclass Skeleton (<Skeleton>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Skeleton>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Skeleton)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kinect_msgs-msg:<Skeleton> is deprecated: use kinect_msgs-msg:Skeleton instead.")))

(cl:ensure-generic-function 'skeleton_tracking_state-val :lambda-list '(m))
(cl:defmethod skeleton_tracking_state-val ((m <Skeleton>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect_msgs-msg:skeleton_tracking_state-val is deprecated.  Use kinect_msgs-msg:skeleton_tracking_state instead.")
  (skeleton_tracking_state m))

(cl:ensure-generic-function 'tracking_id-val :lambda-list '(m))
(cl:defmethod tracking_id-val ((m <Skeleton>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect_msgs-msg:tracking_id-val is deprecated.  Use kinect_msgs-msg:tracking_id instead.")
  (tracking_id m))

(cl:ensure-generic-function 'enrollment_index-val :lambda-list '(m))
(cl:defmethod enrollment_index-val ((m <Skeleton>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect_msgs-msg:enrollment_index-val is deprecated.  Use kinect_msgs-msg:enrollment_index instead.")
  (enrollment_index m))

(cl:ensure-generic-function 'user_index-val :lambda-list '(m))
(cl:defmethod user_index-val ((m <Skeleton>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect_msgs-msg:user_index-val is deprecated.  Use kinect_msgs-msg:user_index instead.")
  (user_index m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <Skeleton>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect_msgs-msg:position-val is deprecated.  Use kinect_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'skeleton_positions-val :lambda-list '(m))
(cl:defmethod skeleton_positions-val ((m <Skeleton>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect_msgs-msg:skeleton_positions-val is deprecated.  Use kinect_msgs-msg:skeleton_positions instead.")
  (skeleton_positions m))

(cl:ensure-generic-function 'skeleton_position_tracking_state-val :lambda-list '(m))
(cl:defmethod skeleton_position_tracking_state-val ((m <Skeleton>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect_msgs-msg:skeleton_position_tracking_state-val is deprecated.  Use kinect_msgs-msg:skeleton_position_tracking_state instead.")
  (skeleton_position_tracking_state m))

(cl:ensure-generic-function 'quality_flag-val :lambda-list '(m))
(cl:defmethod quality_flag-val ((m <Skeleton>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect_msgs-msg:quality_flag-val is deprecated.  Use kinect_msgs-msg:quality_flag instead.")
  (quality_flag m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Skeleton>)))
    "Constants for message type '<Skeleton>"
  '((:SKELETON_NOT_TRACKED . 0)
    (:SKELETON_POSITION_ONLY . 1)
    (:SKELETON_TRACKED . 2)
    (:SKELETON_POSITION_NOT_TRACKED . 0)
    (:SKELETON_POSITION_INFERRED . 1)
    (:SKELETON_POSITION_TRACKED . 2)
    (:SKELETON_POSITION_HIP_CENTER . 0)
    (:SKELETON_POSITION_SPINE . 1)
    (:SKELETON_POSITION_SHOULDER_CENTER . 2)
    (:SKELETON_POSITION_HEAD . 3)
    (:SKELETON_POSITION_SHOULDER_LEFT . 4)
    (:SKELETON_POSITION_ELBOW_LEFT . 5)
    (:SKELETON_POSITION_WRIST_LEFT . 6)
    (:SKELETON_POSITION_HAND_LEFT . 7)
    (:SKELETON_POSITION_SHOULDER_RIGHT . 8)
    (:SKELETON_POSITION_ELBOW_RIGHT . 9)
    (:SKELETON_POSITION_WRIST_RIGHT . 10)
    (:SKELETON_POSITION_HAND_RIGHT . 11)
    (:SKELETON_POSITION_HIP_LEFT . 12)
    (:SKELETON_POSITION_KNEE_LEFT . 13)
    (:SKELETON_POSITION_ANKLE_LEFT . 14)
    (:SKELETON_POSITION_FOOT_LEFT . 15)
    (:SKELETON_POSITION_HIP_RIGHT . 16)
    (:SKELETON_POSITION_KNEE_RIGHT . 17)
    (:SKELETON_POSITION_ANKLE_RIGHT . 18)
    (:SKELETON_POSITION_FOOT_RIGHT . 19)
    (:SKELETON_POSITION_COUNT . 20)
    (:SKELETON_QUALITY_CLIPPED_RIGHT . 1)
    (:SKELETON_QUALITY_CLIPPED_LEFT . 2)
    (:SKELETON_QUALITY_CLIPPED_TOP . 4)
    (:SKELETON_QUALITY_CLIPPED_BOTTOM . 8))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Skeleton)))
    "Constants for message type 'Skeleton"
  '((:SKELETON_NOT_TRACKED . 0)
    (:SKELETON_POSITION_ONLY . 1)
    (:SKELETON_TRACKED . 2)
    (:SKELETON_POSITION_NOT_TRACKED . 0)
    (:SKELETON_POSITION_INFERRED . 1)
    (:SKELETON_POSITION_TRACKED . 2)
    (:SKELETON_POSITION_HIP_CENTER . 0)
    (:SKELETON_POSITION_SPINE . 1)
    (:SKELETON_POSITION_SHOULDER_CENTER . 2)
    (:SKELETON_POSITION_HEAD . 3)
    (:SKELETON_POSITION_SHOULDER_LEFT . 4)
    (:SKELETON_POSITION_ELBOW_LEFT . 5)
    (:SKELETON_POSITION_WRIST_LEFT . 6)
    (:SKELETON_POSITION_HAND_LEFT . 7)
    (:SKELETON_POSITION_SHOULDER_RIGHT . 8)
    (:SKELETON_POSITION_ELBOW_RIGHT . 9)
    (:SKELETON_POSITION_WRIST_RIGHT . 10)
    (:SKELETON_POSITION_HAND_RIGHT . 11)
    (:SKELETON_POSITION_HIP_LEFT . 12)
    (:SKELETON_POSITION_KNEE_LEFT . 13)
    (:SKELETON_POSITION_ANKLE_LEFT . 14)
    (:SKELETON_POSITION_FOOT_LEFT . 15)
    (:SKELETON_POSITION_HIP_RIGHT . 16)
    (:SKELETON_POSITION_KNEE_RIGHT . 17)
    (:SKELETON_POSITION_ANKLE_RIGHT . 18)
    (:SKELETON_POSITION_FOOT_RIGHT . 19)
    (:SKELETON_POSITION_COUNT . 20)
    (:SKELETON_QUALITY_CLIPPED_RIGHT . 1)
    (:SKELETON_QUALITY_CLIPPED_LEFT . 2)
    (:SKELETON_QUALITY_CLIPPED_TOP . 4)
    (:SKELETON_QUALITY_CLIPPED_BOTTOM . 8))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Skeleton>) ostream)
  "Serializes a message object of type '<Skeleton>"
  (cl:let* ((signed (cl:slot-value msg 'skeleton_tracking_state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tracking_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tracking_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tracking_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tracking_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'enrollment_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'enrollment_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'enrollment_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'enrollment_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'user_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'user_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'user_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'user_index)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'skeleton_positions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'skeleton_positions))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'skeleton_position_tracking_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'skeleton_position_tracking_state))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'quality_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'quality_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'quality_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'quality_flag)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Skeleton>) istream)
  "Deserializes a message object of type '<Skeleton>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'skeleton_tracking_state) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tracking_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tracking_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tracking_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tracking_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'enrollment_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'enrollment_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'enrollment_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'enrollment_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'user_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'user_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'user_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'user_index)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'skeleton_positions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'skeleton_positions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Transform))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'skeleton_position_tracking_state) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'skeleton_position_tracking_state)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'quality_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'quality_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'quality_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'quality_flag)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Skeleton>)))
  "Returns string type for a message object of type '<Skeleton>"
  "kinect_msgs/Skeleton")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Skeleton)))
  "Returns string type for a message object of type 'Skeleton"
  "kinect_msgs/Skeleton")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Skeleton>)))
  "Returns md5sum for a message object of type '<Skeleton>"
  "53f31172ad726f0663f636784717b4b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Skeleton)))
  "Returns md5sum for a message object of type 'Skeleton"
  "53f31172ad726f0663f636784717b4b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Skeleton>)))
  "Returns full string definition for message of type '<Skeleton>"
  (cl:format cl:nil "int8 skeleton_tracking_state~%uint32 tracking_id~%uint32 enrollment_index~%uint32 user_index~%geometry_msgs/Transform position~%geometry_msgs/Transform[] skeleton_positions~%int8[] skeleton_position_tracking_state~%uint32 quality_flag~%~%int8 SKELETON_NOT_TRACKED = 0~%int8 SKELETON_POSITION_ONLY = 1~%int8 SKELETON_TRACKED = 2~%~%int8 SKELETON_POSITION_NOT_TRACKED = 0~%int8 SKELETON_POSITION_INFERRED = 1~%int8 SKELETON_POSITION_TRACKED = 2~%~%int8 SKELETON_POSITION_HIP_CENTER = 0~%int8 SKELETON_POSITION_SPINE = 1~%int8 SKELETON_POSITION_SHOULDER_CENTER = 2~%int8 SKELETON_POSITION_HEAD = 3~%int8 SKELETON_POSITION_SHOULDER_LEFT = 4~%int8 SKELETON_POSITION_ELBOW_LEFT = 5~%int8 SKELETON_POSITION_WRIST_LEFT = 6~%int8 SKELETON_POSITION_HAND_LEFT = 7~%int8 SKELETON_POSITION_SHOULDER_RIGHT = 8~%int8 SKELETON_POSITION_ELBOW_RIGHT = 9~%int8 SKELETON_POSITION_WRIST_RIGHT = 10~%int8 SKELETON_POSITION_HAND_RIGHT = 11~%int8 SKELETON_POSITION_HIP_LEFT = 12~%int8 SKELETON_POSITION_KNEE_LEFT = 13~%int8 SKELETON_POSITION_ANKLE_LEFT = 14~%int8 SKELETON_POSITION_FOOT_LEFT = 15~%int8 SKELETON_POSITION_HIP_RIGHT = 16~%int8 SKELETON_POSITION_KNEE_RIGHT = 17~%int8 SKELETON_POSITION_ANKLE_RIGHT = 18~%int8 SKELETON_POSITION_FOOT_RIGHT = 19~%int8 SKELETON_POSITION_COUNT = 20~%~%uint32 SKELETON_QUALITY_CLIPPED_RIGHT = 1~%uint32 SKELETON_QUALITY_CLIPPED_LEFT = 2 ~%uint32 SKELETON_QUALITY_CLIPPED_TOP = 4 ~%uint32 SKELETON_QUALITY_CLIPPED_BOTTOM = 8  ~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Skeleton)))
  "Returns full string definition for message of type 'Skeleton"
  (cl:format cl:nil "int8 skeleton_tracking_state~%uint32 tracking_id~%uint32 enrollment_index~%uint32 user_index~%geometry_msgs/Transform position~%geometry_msgs/Transform[] skeleton_positions~%int8[] skeleton_position_tracking_state~%uint32 quality_flag~%~%int8 SKELETON_NOT_TRACKED = 0~%int8 SKELETON_POSITION_ONLY = 1~%int8 SKELETON_TRACKED = 2~%~%int8 SKELETON_POSITION_NOT_TRACKED = 0~%int8 SKELETON_POSITION_INFERRED = 1~%int8 SKELETON_POSITION_TRACKED = 2~%~%int8 SKELETON_POSITION_HIP_CENTER = 0~%int8 SKELETON_POSITION_SPINE = 1~%int8 SKELETON_POSITION_SHOULDER_CENTER = 2~%int8 SKELETON_POSITION_HEAD = 3~%int8 SKELETON_POSITION_SHOULDER_LEFT = 4~%int8 SKELETON_POSITION_ELBOW_LEFT = 5~%int8 SKELETON_POSITION_WRIST_LEFT = 6~%int8 SKELETON_POSITION_HAND_LEFT = 7~%int8 SKELETON_POSITION_SHOULDER_RIGHT = 8~%int8 SKELETON_POSITION_ELBOW_RIGHT = 9~%int8 SKELETON_POSITION_WRIST_RIGHT = 10~%int8 SKELETON_POSITION_HAND_RIGHT = 11~%int8 SKELETON_POSITION_HIP_LEFT = 12~%int8 SKELETON_POSITION_KNEE_LEFT = 13~%int8 SKELETON_POSITION_ANKLE_LEFT = 14~%int8 SKELETON_POSITION_FOOT_LEFT = 15~%int8 SKELETON_POSITION_HIP_RIGHT = 16~%int8 SKELETON_POSITION_KNEE_RIGHT = 17~%int8 SKELETON_POSITION_ANKLE_RIGHT = 18~%int8 SKELETON_POSITION_FOOT_RIGHT = 19~%int8 SKELETON_POSITION_COUNT = 20~%~%uint32 SKELETON_QUALITY_CLIPPED_RIGHT = 1~%uint32 SKELETON_QUALITY_CLIPPED_LEFT = 2 ~%uint32 SKELETON_QUALITY_CLIPPED_TOP = 4 ~%uint32 SKELETON_QUALITY_CLIPPED_BOTTOM = 8  ~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Skeleton>))
  (cl:+ 0
     1
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'skeleton_positions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'skeleton_position_tracking_state) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Skeleton>))
  "Converts a ROS message object to a list"
  (cl:list 'Skeleton
    (cl:cons ':skeleton_tracking_state (skeleton_tracking_state msg))
    (cl:cons ':tracking_id (tracking_id msg))
    (cl:cons ':enrollment_index (enrollment_index msg))
    (cl:cons ':user_index (user_index msg))
    (cl:cons ':position (position msg))
    (cl:cons ':skeleton_positions (skeleton_positions msg))
    (cl:cons ':skeleton_position_tracking_state (skeleton_position_tracking_state msg))
    (cl:cons ':quality_flag (quality_flag msg))
))
