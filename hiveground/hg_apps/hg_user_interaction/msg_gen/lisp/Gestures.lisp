; Auto-generated. Do not edit!


(cl:in-package hg_user_interaction-msg)


;//! \htmlinclude Gestures.msg.html

(cl:defclass <Gestures> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (gestures
    :reader gestures
    :initarg :gestures
    :type (cl:vector hg_user_interaction-msg:Gesture)
   :initform (cl:make-array 0 :element-type 'hg_user_interaction-msg:Gesture :initial-element (cl:make-instance 'hg_user_interaction-msg:Gesture))))
)

(cl:defclass Gestures (<Gestures>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Gestures>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Gestures)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hg_user_interaction-msg:<Gestures> is deprecated: use hg_user_interaction-msg:Gestures instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Gestures>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_user_interaction-msg:header-val is deprecated.  Use hg_user_interaction-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'gestures-val :lambda-list '(m))
(cl:defmethod gestures-val ((m <Gestures>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_user_interaction-msg:gestures-val is deprecated.  Use hg_user_interaction-msg:gestures instead.")
  (gestures m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Gestures>) ostream)
  "Serializes a message object of type '<Gestures>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gestures))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'gestures))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Gestures>) istream)
  "Deserializes a message object of type '<Gestures>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gestures) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gestures)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'hg_user_interaction-msg:Gesture))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Gestures>)))
  "Returns string type for a message object of type '<Gestures>"
  "hg_user_interaction/Gestures")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Gestures)))
  "Returns string type for a message object of type 'Gestures"
  "hg_user_interaction/Gestures")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Gestures>)))
  "Returns md5sum for a message object of type '<Gestures>"
  "acbc7e79b65b2f75cdaf2322e9e35134")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Gestures)))
  "Returns md5sum for a message object of type 'Gestures"
  "acbc7e79b65b2f75cdaf2322e9e35134")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Gestures>)))
  "Returns full string definition for message of type '<Gestures>"
  (cl:format cl:nil "Header header~%Gesture[] gestures~%~%~%~%~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: hg_user_interaction/Gesture~%#gesture~%uint32 GESTURE_NOT_DETECTED = 0~%~%#hand~%uint32 GESTURE_HAND_SWEEP = 1~%uint32 GESTURE_HAND_PUSH_PULL = 2~%~%#elbow~%uint32 GESTURE_ELBOW_TOGGLE = 51~%~%#body~%uint32 GESTURE_BODY_MOVE = 101~%uint32 GESTURE_BODY_TWIST = 102~%uint32 GESTURE_BODY_LEAN = 103~%~%#hand(s)~%uint32 HAND_ONE = 1  ~%uint32 HAND_TWO = 2~%~%#direction flags~%uint32 DIR_X_POS = 1~%uint32 DIR_X_NEG = 2~%uint32 DIR_Y_POS = 3~%uint32 DIR_Y_NEG = 4~%uint32 DIR_Z_POS = 5~%uint32 DIR_Z_NEG = 6~%uint32 ROT_X_POS = 7~%uint32 ROT_X_NEG = 8~%uint32 ROT_Y_POS = 9~%uint32 ROT_Y_NEG = 10~%uint32 ROT_Z_POS = 11~%uint32 ROT_Z_NEG = 12~%~%~%uint32 type~%uint32 hand_count~%uint32 direction~%~%#Only used if the type specified has some use of them ~%float64[] vars~%~%#Only used if the type specified has some use of them~%geometry_msgs/Vector3[] vectors~%~%#Only used if the type specified has some use of them~%geometry_msgs/Transform[] transforms~%~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Gestures)))
  "Returns full string definition for message of type 'Gestures"
  (cl:format cl:nil "Header header~%Gesture[] gestures~%~%~%~%~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: hg_user_interaction/Gesture~%#gesture~%uint32 GESTURE_NOT_DETECTED = 0~%~%#hand~%uint32 GESTURE_HAND_SWEEP = 1~%uint32 GESTURE_HAND_PUSH_PULL = 2~%~%#elbow~%uint32 GESTURE_ELBOW_TOGGLE = 51~%~%#body~%uint32 GESTURE_BODY_MOVE = 101~%uint32 GESTURE_BODY_TWIST = 102~%uint32 GESTURE_BODY_LEAN = 103~%~%#hand(s)~%uint32 HAND_ONE = 1  ~%uint32 HAND_TWO = 2~%~%#direction flags~%uint32 DIR_X_POS = 1~%uint32 DIR_X_NEG = 2~%uint32 DIR_Y_POS = 3~%uint32 DIR_Y_NEG = 4~%uint32 DIR_Z_POS = 5~%uint32 DIR_Z_NEG = 6~%uint32 ROT_X_POS = 7~%uint32 ROT_X_NEG = 8~%uint32 ROT_Y_POS = 9~%uint32 ROT_Y_NEG = 10~%uint32 ROT_Z_POS = 11~%uint32 ROT_Z_NEG = 12~%~%~%uint32 type~%uint32 hand_count~%uint32 direction~%~%#Only used if the type specified has some use of them ~%float64[] vars~%~%#Only used if the type specified has some use of them~%geometry_msgs/Vector3[] vectors~%~%#Only used if the type specified has some use of them~%geometry_msgs/Transform[] transforms~%~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Gestures>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gestures) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Gestures>))
  "Converts a ROS message object to a list"
  (cl:list 'Gestures
    (cl:cons ':header (header msg))
    (cl:cons ':gestures (gestures msg))
))
