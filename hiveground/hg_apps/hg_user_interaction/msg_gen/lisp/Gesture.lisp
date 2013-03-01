; Auto-generated. Do not edit!


(cl:in-package hg_user_interaction-msg)


;//! \htmlinclude Gesture.msg.html

(cl:defclass <Gesture> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (hand_count
    :reader hand_count
    :initarg :hand_count
    :type cl:integer
    :initform 0)
   (direction
    :reader direction
    :initarg :direction
    :type cl:integer
    :initform 0)
   (vars
    :reader vars
    :initarg :vars
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (vectors
    :reader vectors
    :initarg :vectors
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3)))
   (transforms
    :reader transforms
    :initarg :transforms
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform)))
)

(cl:defclass Gesture (<Gesture>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Gesture>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Gesture)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hg_user_interaction-msg:<Gesture> is deprecated: use hg_user_interaction-msg:Gesture instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Gesture>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_user_interaction-msg:type-val is deprecated.  Use hg_user_interaction-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'hand_count-val :lambda-list '(m))
(cl:defmethod hand_count-val ((m <Gesture>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_user_interaction-msg:hand_count-val is deprecated.  Use hg_user_interaction-msg:hand_count instead.")
  (hand_count m))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <Gesture>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_user_interaction-msg:direction-val is deprecated.  Use hg_user_interaction-msg:direction instead.")
  (direction m))

(cl:ensure-generic-function 'vars-val :lambda-list '(m))
(cl:defmethod vars-val ((m <Gesture>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_user_interaction-msg:vars-val is deprecated.  Use hg_user_interaction-msg:vars instead.")
  (vars m))

(cl:ensure-generic-function 'vectors-val :lambda-list '(m))
(cl:defmethod vectors-val ((m <Gesture>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_user_interaction-msg:vectors-val is deprecated.  Use hg_user_interaction-msg:vectors instead.")
  (vectors m))

(cl:ensure-generic-function 'transforms-val :lambda-list '(m))
(cl:defmethod transforms-val ((m <Gesture>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_user_interaction-msg:transforms-val is deprecated.  Use hg_user_interaction-msg:transforms instead.")
  (transforms m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Gesture>)))
    "Constants for message type '<Gesture>"
  '((:GESTURE_NOT_DETECTED . 0)
    (:GESTURE_HAND_SWEEP . 1)
    (:GESTURE_HAND_PUSH_PULL . 2)
    (:GESTURE_BODY_MOVE . 101)
    (:GESTURE_BODY_TWIST . 102)
    (:GESTURE_BODY_LEAN . 103)
    (:HAND_ONE . 1)
    (:HAND_TWO . 2)
    (:DIR_X_POS . 1)
    (:DIR_X_NEG . 2)
    (:DIR_Y_POS . 3)
    (:DIR_Y_NEG . 4)
    (:DIR_Z_POS . 5)
    (:DIR_Z_NEG . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Gesture)))
    "Constants for message type 'Gesture"
  '((:GESTURE_NOT_DETECTED . 0)
    (:GESTURE_HAND_SWEEP . 1)
    (:GESTURE_HAND_PUSH_PULL . 2)
    (:GESTURE_BODY_MOVE . 101)
    (:GESTURE_BODY_TWIST . 102)
    (:GESTURE_BODY_LEAN . 103)
    (:HAND_ONE . 1)
    (:HAND_TWO . 2)
    (:DIR_X_POS . 1)
    (:DIR_X_NEG . 2)
    (:DIR_Y_POS . 3)
    (:DIR_Y_NEG . 4)
    (:DIR_Z_POS . 5)
    (:DIR_Z_NEG . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Gesture>) ostream)
  "Serializes a message object of type '<Gesture>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hand_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hand_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'hand_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'hand_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'direction)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'direction)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'direction)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'direction)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'vars))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'vars))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'vectors))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'vectors))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'transforms) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Gesture>) istream)
  "Deserializes a message object of type '<Gesture>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hand_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hand_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'hand_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'hand_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'direction)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'direction)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'direction)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'direction)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'vars) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'vars)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'vectors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'vectors)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'transforms) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Gesture>)))
  "Returns string type for a message object of type '<Gesture>"
  "hg_user_interaction/Gesture")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Gesture)))
  "Returns string type for a message object of type 'Gesture"
  "hg_user_interaction/Gesture")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Gesture>)))
  "Returns md5sum for a message object of type '<Gesture>"
  "355d3ec6c605417c6e0a093dc3cfb48c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Gesture)))
  "Returns md5sum for a message object of type 'Gesture"
  "355d3ec6c605417c6e0a093dc3cfb48c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Gesture>)))
  "Returns full string definition for message of type '<Gesture>"
  (cl:format cl:nil "#gesture~%uint32 GESTURE_NOT_DETECTED = 0~%~%#hand~%uint32 GESTURE_HAND_SWEEP = 1~%uint32 GESTURE_HAND_PUSH_PULL = 2~%~%#body~%uint32 GESTURE_BODY_MOVE = 101~%uint32 GESTURE_BODY_TWIST = 102~%uint32 GESTURE_BODY_LEAN = 103~%~%#hand(s)~%uint32 HAND_ONE = 1  ~%uint32 HAND_TWO = 2~%~%#direction flags~%uint32 DIR_X_POS = 1~%uint32 DIR_X_NEG = 2~%uint32 DIR_Y_POS = 3~%uint32 DIR_Y_NEG = 4~%uint32 DIR_Z_POS = 5~%uint32 DIR_Z_NEG = 6~%~%~%uint32 type~%uint32 hand_count~%uint32 direction~%~%#Only used if the type specified has some use of them ~%float64[] vars~%~%#Only used if the type specified has some use of them~%geometry_msgs/Vector3[] vectors~%~%#Only used if the type specified has some use of them~%geometry_msgs/Transform transforms~%~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Gesture)))
  "Returns full string definition for message of type 'Gesture"
  (cl:format cl:nil "#gesture~%uint32 GESTURE_NOT_DETECTED = 0~%~%#hand~%uint32 GESTURE_HAND_SWEEP = 1~%uint32 GESTURE_HAND_PUSH_PULL = 2~%~%#body~%uint32 GESTURE_BODY_MOVE = 101~%uint32 GESTURE_BODY_TWIST = 102~%uint32 GESTURE_BODY_LEAN = 103~%~%#hand(s)~%uint32 HAND_ONE = 1  ~%uint32 HAND_TWO = 2~%~%#direction flags~%uint32 DIR_X_POS = 1~%uint32 DIR_X_NEG = 2~%uint32 DIR_Y_POS = 3~%uint32 DIR_Y_NEG = 4~%uint32 DIR_Z_POS = 5~%uint32 DIR_Z_NEG = 6~%~%~%uint32 type~%uint32 hand_count~%uint32 direction~%~%#Only used if the type specified has some use of them ~%float64[] vars~%~%#Only used if the type specified has some use of them~%geometry_msgs/Vector3[] vectors~%~%#Only used if the type specified has some use of them~%geometry_msgs/Transform transforms~%~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Gesture>))
  (cl:+ 0
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vars) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vectors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'transforms))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Gesture>))
  "Converts a ROS message object to a list"
  (cl:list 'Gesture
    (cl:cons ':type (type msg))
    (cl:cons ':hand_count (hand_count msg))
    (cl:cons ':direction (direction msg))
    (cl:cons ':vars (vars msg))
    (cl:cons ':vectors (vectors msg))
    (cl:cons ':transforms (transforms msg))
))
