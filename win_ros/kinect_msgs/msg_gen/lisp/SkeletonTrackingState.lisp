; Auto-generated. Do not edit!


(cl:in-package kinect_msgs-msg)


;//! \htmlinclude SkeletonTrackingState.msg.html

(cl:defclass <SkeletonTrackingState> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SkeletonTrackingState (<SkeletonTrackingState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SkeletonTrackingState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SkeletonTrackingState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kinect_msgs-msg:<SkeletonTrackingState> is deprecated: use kinect_msgs-msg:SkeletonTrackingState instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SkeletonTrackingState>)))
    "Constants for message type '<SkeletonTrackingState>"
  '((:SKELETON_NOT_TRACKED . 0)
    (:SKELETON_POSITION_ONLY . 1)
    (:SKELETON_TRACKED . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SkeletonTrackingState)))
    "Constants for message type 'SkeletonTrackingState"
  '((:SKELETON_NOT_TRACKED . 0)
    (:SKELETON_POSITION_ONLY . 1)
    (:SKELETON_TRACKED . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SkeletonTrackingState>) ostream)
  "Serializes a message object of type '<SkeletonTrackingState>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SkeletonTrackingState>) istream)
  "Deserializes a message object of type '<SkeletonTrackingState>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SkeletonTrackingState>)))
  "Returns string type for a message object of type '<SkeletonTrackingState>"
  "kinect_msgs/SkeletonTrackingState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SkeletonTrackingState)))
  "Returns string type for a message object of type 'SkeletonTrackingState"
  "kinect_msgs/SkeletonTrackingState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SkeletonTrackingState>)))
  "Returns md5sum for a message object of type '<SkeletonTrackingState>"
  "5ef7185bd672713cd4304a672c9b653f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SkeletonTrackingState)))
  "Returns md5sum for a message object of type 'SkeletonTrackingState"
  "5ef7185bd672713cd4304a672c9b653f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SkeletonTrackingState>)))
  "Returns full string definition for message of type '<SkeletonTrackingState>"
  (cl:format cl:nil "int8 SKELETON_NOT_TRACKED = 0~%int8 SKELETON_POSITION_ONLY = 1~%int8 SKELETON_TRACKED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SkeletonTrackingState)))
  "Returns full string definition for message of type 'SkeletonTrackingState"
  (cl:format cl:nil "int8 SKELETON_NOT_TRACKED = 0~%int8 SKELETON_POSITION_ONLY = 1~%int8 SKELETON_TRACKED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SkeletonTrackingState>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SkeletonTrackingState>))
  "Converts a ROS message object to a list"
  (cl:list 'SkeletonTrackingState
))
