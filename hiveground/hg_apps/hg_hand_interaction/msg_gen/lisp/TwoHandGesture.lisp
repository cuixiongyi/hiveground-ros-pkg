; Auto-generated. Do not edit!


(cl:in-package hg_hand_interaction-msg)


;//! \htmlinclude TwoHandGesture.msg.html

(cl:defclass <TwoHandGesture> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header)))
)

(cl:defclass TwoHandGesture (<TwoHandGesture>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TwoHandGesture>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TwoHandGesture)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hg_hand_interaction-msg:<TwoHandGesture> is deprecated: use hg_hand_interaction-msg:TwoHandGesture instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TwoHandGesture>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_hand_interaction-msg:header-val is deprecated.  Use hg_hand_interaction-msg:header instead.")
  (header m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TwoHandGesture>) ostream)
  "Serializes a message object of type '<TwoHandGesture>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TwoHandGesture>) istream)
  "Deserializes a message object of type '<TwoHandGesture>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TwoHandGesture>)))
  "Returns string type for a message object of type '<TwoHandGesture>"
  "hg_hand_interaction/TwoHandGesture")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TwoHandGesture)))
  "Returns string type for a message object of type 'TwoHandGesture"
  "hg_hand_interaction/TwoHandGesture")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TwoHandGesture>)))
  "Returns md5sum for a message object of type '<TwoHandGesture>"
  "d7be0bb39af8fb9129d5a76e6b63a290")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TwoHandGesture)))
  "Returns md5sum for a message object of type 'TwoHandGesture"
  "d7be0bb39af8fb9129d5a76e6b63a290")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TwoHandGesture>)))
  "Returns full string definition for message of type '<TwoHandGesture>"
  (cl:format cl:nil "Header header~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TwoHandGesture)))
  "Returns full string definition for message of type 'TwoHandGesture"
  (cl:format cl:nil "Header header~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TwoHandGesture>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TwoHandGesture>))
  "Converts a ROS message object to a list"
  (cl:list 'TwoHandGesture
    (cl:cons ':header (header msg))
))
