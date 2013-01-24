; Auto-generated. Do not edit!


(cl:in-package hg_hand_interaction-msg)


;//! \htmlinclude HandGesture.msg.html

(cl:defclass <HandGesture> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass HandGesture (<HandGesture>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandGesture>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandGesture)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hg_hand_interaction-msg:<HandGesture> is deprecated: use hg_hand_interaction-msg:HandGesture instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <HandGesture>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_hand_interaction-msg:type-val is deprecated.  Use hg_hand_interaction-msg:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<HandGesture>)))
    "Constants for message type '<HandGesture>"
  '((:NOT_DETECTED . 0)
    (:SWEEP_UP_ONE_HAND . 1)
    (:SWEEP_DOWN_ONE_HAND . 2)
    (:SWEEP_LEFT_ONE_HAND . 3)
    (:SWEEP_RIGHT_ONE_HAND . 4)
    (:SWEEP_FORWARD_ONE_HAND . 5)
    (:SWEEP_BACKWARD_ONE_HAND . 6)
    (:SWEEP_UP_TWO_HAND . 7)
    (:SWEEP_DOWN_TWO_HAND . 8)
    (:SWEEP_LEFT_TWO_HAND . 9)
    (:SWEEP_RIGHT_TWO_HAND . 10)
    (:SWEEP_FORWARD_TWO_HAND . 11)
    (:SWEEP_BACKWARD_TWO_HAND . 12)
    (:SWEEP_OPEN_TWO_HAND . 13)
    (:SWEEP_CLOSE_TWO_HAND . 14))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'HandGesture)))
    "Constants for message type 'HandGesture"
  '((:NOT_DETECTED . 0)
    (:SWEEP_UP_ONE_HAND . 1)
    (:SWEEP_DOWN_ONE_HAND . 2)
    (:SWEEP_LEFT_ONE_HAND . 3)
    (:SWEEP_RIGHT_ONE_HAND . 4)
    (:SWEEP_FORWARD_ONE_HAND . 5)
    (:SWEEP_BACKWARD_ONE_HAND . 6)
    (:SWEEP_UP_TWO_HAND . 7)
    (:SWEEP_DOWN_TWO_HAND . 8)
    (:SWEEP_LEFT_TWO_HAND . 9)
    (:SWEEP_RIGHT_TWO_HAND . 10)
    (:SWEEP_FORWARD_TWO_HAND . 11)
    (:SWEEP_BACKWARD_TWO_HAND . 12)
    (:SWEEP_OPEN_TWO_HAND . 13)
    (:SWEEP_CLOSE_TWO_HAND . 14))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandGesture>) ostream)
  "Serializes a message object of type '<HandGesture>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandGesture>) istream)
  "Deserializes a message object of type '<HandGesture>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandGesture>)))
  "Returns string type for a message object of type '<HandGesture>"
  "hg_hand_interaction/HandGesture")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandGesture)))
  "Returns string type for a message object of type 'HandGesture"
  "hg_hand_interaction/HandGesture")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandGesture>)))
  "Returns md5sum for a message object of type '<HandGesture>"
  "54924b3bfee069bfbfc09b1d80e2254d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandGesture)))
  "Returns md5sum for a message object of type 'HandGesture"
  "54924b3bfee069bfbfc09b1d80e2254d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandGesture>)))
  "Returns full string definition for message of type '<HandGesture>"
  (cl:format cl:nil "int8 type~%#request constants~%int8 NOT_DETECTED = 0~%int8 SWEEP_UP_ONE_HAND = 1~%int8 SWEEP_DOWN_ONE_HAND = 2~%int8 SWEEP_LEFT_ONE_HAND = 3~%int8 SWEEP_RIGHT_ONE_HAND = 4~%int8 SWEEP_FORWARD_ONE_HAND = 5~%int8 SWEEP_BACKWARD_ONE_HAND = 6~%~%int8 SWEEP_UP_TWO_HAND = 7~%int8 SWEEP_DOWN_TWO_HAND = 8~%int8 SWEEP_LEFT_TWO_HAND = 9~%int8 SWEEP_RIGHT_TWO_HAND = 10~%int8 SWEEP_FORWARD_TWO_HAND = 11~%int8 SWEEP_BACKWARD_TWO_HAND = 12~%int8 SWEEP_OPEN_TWO_HAND = 13~%int8 SWEEP_CLOSE_TWO_HAND = 14~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandGesture)))
  "Returns full string definition for message of type 'HandGesture"
  (cl:format cl:nil "int8 type~%#request constants~%int8 NOT_DETECTED = 0~%int8 SWEEP_UP_ONE_HAND = 1~%int8 SWEEP_DOWN_ONE_HAND = 2~%int8 SWEEP_LEFT_ONE_HAND = 3~%int8 SWEEP_RIGHT_ONE_HAND = 4~%int8 SWEEP_FORWARD_ONE_HAND = 5~%int8 SWEEP_BACKWARD_ONE_HAND = 6~%~%int8 SWEEP_UP_TWO_HAND = 7~%int8 SWEEP_DOWN_TWO_HAND = 8~%int8 SWEEP_LEFT_TWO_HAND = 9~%int8 SWEEP_RIGHT_TWO_HAND = 10~%int8 SWEEP_FORWARD_TWO_HAND = 11~%int8 SWEEP_BACKWARD_TWO_HAND = 12~%int8 SWEEP_OPEN_TWO_HAND = 13~%int8 SWEEP_CLOSE_TWO_HAND = 14~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandGesture>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandGesture>))
  "Converts a ROS message object to a list"
  (cl:list 'HandGesture
    (cl:cons ':type (type msg))
))
