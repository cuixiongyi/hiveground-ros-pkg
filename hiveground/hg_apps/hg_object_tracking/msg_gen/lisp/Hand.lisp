; Auto-generated. Do not edit!


(cl:in-package hg_object_tracking-msg)


;//! \htmlinclude Hand.msg.html

(cl:defclass <Hand> (roslisp-msg-protocol:ros-message)
  ((arm_centroid
    :reader arm_centroid
    :initarg :arm_centroid
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform))
   (arm_eigen_value
    :reader arm_eigen_value
    :initarg :arm_eigen_value
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (arm_eigen_vectors
    :reader arm_eigen_vectors
    :initarg :arm_eigen_vectors
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3)))
   (hand_centroid
    :reader hand_centroid
    :initarg :hand_centroid
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform))
   (hand_eigen_value
    :reader hand_eigen_value
    :initarg :hand_eigen_value
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (hand_eigen_vectors
    :reader hand_eigen_vectors
    :initarg :hand_eigen_vectors
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3)))
   (fingers
    :reader fingers
    :initarg :fingers
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
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

(cl:ensure-generic-function 'arm_centroid-val :lambda-list '(m))
(cl:defmethod arm_centroid-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:arm_centroid-val is deprecated.  Use hg_object_tracking-msg:arm_centroid instead.")
  (arm_centroid m))

(cl:ensure-generic-function 'arm_eigen_value-val :lambda-list '(m))
(cl:defmethod arm_eigen_value-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:arm_eigen_value-val is deprecated.  Use hg_object_tracking-msg:arm_eigen_value instead.")
  (arm_eigen_value m))

(cl:ensure-generic-function 'arm_eigen_vectors-val :lambda-list '(m))
(cl:defmethod arm_eigen_vectors-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:arm_eigen_vectors-val is deprecated.  Use hg_object_tracking-msg:arm_eigen_vectors instead.")
  (arm_eigen_vectors m))

(cl:ensure-generic-function 'hand_centroid-val :lambda-list '(m))
(cl:defmethod hand_centroid-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:hand_centroid-val is deprecated.  Use hg_object_tracking-msg:hand_centroid instead.")
  (hand_centroid m))

(cl:ensure-generic-function 'hand_eigen_value-val :lambda-list '(m))
(cl:defmethod hand_eigen_value-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:hand_eigen_value-val is deprecated.  Use hg_object_tracking-msg:hand_eigen_value instead.")
  (hand_eigen_value m))

(cl:ensure-generic-function 'hand_eigen_vectors-val :lambda-list '(m))
(cl:defmethod hand_eigen_vectors-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:hand_eigen_vectors-val is deprecated.  Use hg_object_tracking-msg:hand_eigen_vectors instead.")
  (hand_eigen_vectors m))

(cl:ensure-generic-function 'fingers-val :lambda-list '(m))
(cl:defmethod fingers-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:fingers-val is deprecated.  Use hg_object_tracking-msg:fingers instead.")
  (fingers m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <Hand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_object_tracking-msg:state-val is deprecated.  Use hg_object_tracking-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hand>) ostream)
  "Serializes a message object of type '<Hand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'arm_centroid) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'arm_eigen_value) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'arm_eigen_vectors))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'arm_eigen_vectors))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'hand_centroid) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'hand_eigen_value) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'hand_eigen_vectors))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'hand_eigen_vectors))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'fingers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'fingers))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'state))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hand>) istream)
  "Deserializes a message object of type '<Hand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'arm_centroid) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'arm_eigen_value) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'arm_eigen_vectors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'arm_eigen_vectors)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'hand_centroid) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'hand_eigen_value) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'hand_eigen_vectors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'hand_eigen_vectors)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
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
  "96d008446c62806065a06ae44bfb925d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hand)))
  "Returns md5sum for a message object of type 'Hand"
  "96d008446c62806065a06ae44bfb925d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hand>)))
  "Returns full string definition for message of type '<Hand>"
  (cl:format cl:nil "geometry_msgs/Transform arm_centroid~%geometry_msgs/Vector3 arm_eigen_value~%geometry_msgs/Vector3[] arm_eigen_vectors~%~%geometry_msgs/Transform hand_centroid~%geometry_msgs/Vector3 hand_eigen_value~%geometry_msgs/Vector3[] hand_eigen_vectors~%geometry_msgs/Point[] fingers~%~%#Possibilities for state variable:~%# open - open palm, usually five fingers~%# grip - fingers curled forward~%# paddle -  fingers together and straight~%# fist   ~%string state~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hand)))
  "Returns full string definition for message of type 'Hand"
  (cl:format cl:nil "geometry_msgs/Transform arm_centroid~%geometry_msgs/Vector3 arm_eigen_value~%geometry_msgs/Vector3[] arm_eigen_vectors~%~%geometry_msgs/Transform hand_centroid~%geometry_msgs/Vector3 hand_eigen_value~%geometry_msgs/Vector3[] hand_eigen_vectors~%geometry_msgs/Point[] fingers~%~%#Possibilities for state variable:~%# open - open palm, usually five fingers~%# grip - fingers curled forward~%# paddle -  fingers together and straight~%# fist   ~%string state~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'arm_centroid))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'arm_eigen_value))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'arm_eigen_vectors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'hand_centroid))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'hand_eigen_value))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'hand_eigen_vectors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'fingers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hand>))
  "Converts a ROS message object to a list"
  (cl:list 'Hand
    (cl:cons ':arm_centroid (arm_centroid msg))
    (cl:cons ':arm_eigen_value (arm_eigen_value msg))
    (cl:cons ':arm_eigen_vectors (arm_eigen_vectors msg))
    (cl:cons ':hand_centroid (hand_centroid msg))
    (cl:cons ':hand_eigen_value (hand_eigen_value msg))
    (cl:cons ':hand_eigen_vectors (hand_eigen_vectors msg))
    (cl:cons ':fingers (fingers msg))
    (cl:cons ':state (state msg))
))
