; Auto-generated. Do not edit!


(cl:in-package hg_hand_interaction-msg)


;//! \htmlinclude HandGestures.msg.html

(cl:defclass <HandGestures> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (gestures
    :reader gestures
    :initarg :gestures
    :type (cl:vector hg_hand_interaction-msg:HandGesture)
   :initform (cl:make-array 0 :element-type 'hg_hand_interaction-msg:HandGesture :initial-element (cl:make-instance 'hg_hand_interaction-msg:HandGesture))))
)

(cl:defclass HandGestures (<HandGestures>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandGestures>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandGestures)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hg_hand_interaction-msg:<HandGestures> is deprecated: use hg_hand_interaction-msg:HandGestures instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HandGestures>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_hand_interaction-msg:header-val is deprecated.  Use hg_hand_interaction-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'gestures-val :lambda-list '(m))
(cl:defmethod gestures-val ((m <HandGestures>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_hand_interaction-msg:gestures-val is deprecated.  Use hg_hand_interaction-msg:gestures instead.")
  (gestures m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandGestures>) ostream)
  "Serializes a message object of type '<HandGestures>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gestures))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'gestures))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandGestures>) istream)
  "Deserializes a message object of type '<HandGestures>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gestures) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gestures)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'hg_hand_interaction-msg:HandGesture))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandGestures>)))
  "Returns string type for a message object of type '<HandGestures>"
  "hg_hand_interaction/HandGestures")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandGestures)))
  "Returns string type for a message object of type 'HandGestures"
  "hg_hand_interaction/HandGestures")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandGestures>)))
  "Returns md5sum for a message object of type '<HandGestures>"
  "6e53ed4eb2268a6b9114b059f3c9fbf9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandGestures)))
  "Returns md5sum for a message object of type 'HandGestures"
  "6e53ed4eb2268a6b9114b059f3c9fbf9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandGestures>)))
  "Returns full string definition for message of type '<HandGestures>"
  (cl:format cl:nil "Header header~%HandGesture[] gestures~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: hg_hand_interaction/HandGesture~%int8 type~%#request constants~%int8 NOT_DETECTED = 0~%int8 SWEEP_UP_ONE_HAND = 1~%int8 SWEEP_DOWN_ONE_HAND = 2~%int8 SWEEP_LEFT_ONE_HAND = 3~%int8 SWEEP_RIGHT_ONE_HAND = 4~%int8 SWEEP_FORWARD_ONE_HAND = 5~%int8 SWEEP_BACKWARD_ONE_HAND = 6~%~%int8 SWEEP_UP_TWO_HAND = 7~%int8 SWEEP_DOWN_TWO_HAND = 8~%int8 SWEEP_LEFT_TWO_HAND = 9~%int8 SWEEP_RIGHT_TWO_HAND = 10~%int8 SWEEP_FORWARD_TWO_HAND = 11~%int8 SWEEP_BACKWARD_TWO_HAND = 12~%int8 SWEEP_OPEN_TWO_HAND = 13~%int8 SWEEP_CLOSE_TWO_HAND = 14~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandGestures)))
  "Returns full string definition for message of type 'HandGestures"
  (cl:format cl:nil "Header header~%HandGesture[] gestures~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: hg_hand_interaction/HandGesture~%int8 type~%#request constants~%int8 NOT_DETECTED = 0~%int8 SWEEP_UP_ONE_HAND = 1~%int8 SWEEP_DOWN_ONE_HAND = 2~%int8 SWEEP_LEFT_ONE_HAND = 3~%int8 SWEEP_RIGHT_ONE_HAND = 4~%int8 SWEEP_FORWARD_ONE_HAND = 5~%int8 SWEEP_BACKWARD_ONE_HAND = 6~%~%int8 SWEEP_UP_TWO_HAND = 7~%int8 SWEEP_DOWN_TWO_HAND = 8~%int8 SWEEP_LEFT_TWO_HAND = 9~%int8 SWEEP_RIGHT_TWO_HAND = 10~%int8 SWEEP_FORWARD_TWO_HAND = 11~%int8 SWEEP_BACKWARD_TWO_HAND = 12~%int8 SWEEP_OPEN_TWO_HAND = 13~%int8 SWEEP_CLOSE_TWO_HAND = 14~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandGestures>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gestures) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandGestures>))
  "Converts a ROS message object to a list"
  (cl:list 'HandGestures
    (cl:cons ':header (header msg))
    (cl:cons ':gestures (gestures msg))
))
