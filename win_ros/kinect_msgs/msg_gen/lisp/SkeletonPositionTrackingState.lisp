; Auto-generated. Do not edit!


(cl:in-package kinect_msgs-msg)


;//! \htmlinclude SkeletonPositionTrackingState.msg.html

(cl:defclass <SkeletonPositionTrackingState> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SkeletonPositionTrackingState (<SkeletonPositionTrackingState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SkeletonPositionTrackingState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SkeletonPositionTrackingState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kinect_msgs-msg:<SkeletonPositionTrackingState> is deprecated: use kinect_msgs-msg:SkeletonPositionTrackingState instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SkeletonPositionTrackingState>)))
    "Constants for message type '<SkeletonPositionTrackingState>"
  '((:NUI_SKELETON_POSITION_NOT_TRACKED . 0)
    (:NUI_SKELETON_POSITION_INFERRED . 1)
    (:NUI_SKELETON_POSITION_TRACKED . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SkeletonPositionTrackingState)))
    "Constants for message type 'SkeletonPositionTrackingState"
  '((:NUI_SKELETON_POSITION_NOT_TRACKED . 0)
    (:NUI_SKELETON_POSITION_INFERRED . 1)
    (:NUI_SKELETON_POSITION_TRACKED . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SkeletonPositionTrackingState>) ostream)
  "Serializes a message object of type '<SkeletonPositionTrackingState>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SkeletonPositionTrackingState>) istream)
  "Deserializes a message object of type '<SkeletonPositionTrackingState>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SkeletonPositionTrackingState>)))
  "Returns string type for a message object of type '<SkeletonPositionTrackingState>"
  "kinect_msgs/SkeletonPositionTrackingState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SkeletonPositionTrackingState)))
  "Returns string type for a message object of type 'SkeletonPositionTrackingState"
  "kinect_msgs/SkeletonPositionTrackingState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SkeletonPositionTrackingState>)))
  "Returns md5sum for a message object of type '<SkeletonPositionTrackingState>"
  "8e5b6512cb1c58f34c3877dc4c11e2fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SkeletonPositionTrackingState)))
  "Returns md5sum for a message object of type 'SkeletonPositionTrackingState"
  "8e5b6512cb1c58f34c3877dc4c11e2fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SkeletonPositionTrackingState>)))
  "Returns full string definition for message of type '<SkeletonPositionTrackingState>"
  (cl:format cl:nil "int8 NUI_SKELETON_POSITION_NOT_TRACKED = 0~%int8 NUI_SKELETON_POSITION_INFERRED = 1~%int8 NUI_SKELETON_POSITION_TRACKED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SkeletonPositionTrackingState)))
  "Returns full string definition for message of type 'SkeletonPositionTrackingState"
  (cl:format cl:nil "int8 NUI_SKELETON_POSITION_NOT_TRACKED = 0~%int8 NUI_SKELETON_POSITION_INFERRED = 1~%int8 NUI_SKELETON_POSITION_TRACKED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SkeletonPositionTrackingState>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SkeletonPositionTrackingState>))
  "Converts a ROS message object to a list"
  (cl:list 'SkeletonPositionTrackingState
))
