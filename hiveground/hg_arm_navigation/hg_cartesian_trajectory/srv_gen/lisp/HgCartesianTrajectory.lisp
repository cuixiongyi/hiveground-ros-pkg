; Auto-generated. Do not edit!


(cl:in-package hg_cartesian_trajectory-srv)


;//! \htmlinclude HgCartesianTrajectory-request.msg.html

(cl:defclass <HgCartesianTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (motion_plan_request
    :reader motion_plan_request
    :initarg :motion_plan_request
    :type arm_navigation_msgs-msg:MotionPlanRequest
    :initform (cl:make-instance 'arm_navigation_msgs-msg:MotionPlanRequest))
   (poses
    :reader poses
    :initarg :poses
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose)))
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass HgCartesianTrajectory-request (<HgCartesianTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HgCartesianTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HgCartesianTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hg_cartesian_trajectory-srv:<HgCartesianTrajectory-request> is deprecated: use hg_cartesian_trajectory-srv:HgCartesianTrajectory-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HgCartesianTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_cartesian_trajectory-srv:header-val is deprecated.  Use hg_cartesian_trajectory-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'motion_plan_request-val :lambda-list '(m))
(cl:defmethod motion_plan_request-val ((m <HgCartesianTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_cartesian_trajectory-srv:motion_plan_request-val is deprecated.  Use hg_cartesian_trajectory-srv:motion_plan_request instead.")
  (motion_plan_request m))

(cl:ensure-generic-function 'poses-val :lambda-list '(m))
(cl:defmethod poses-val ((m <HgCartesianTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_cartesian_trajectory-srv:poses-val is deprecated.  Use hg_cartesian_trajectory-srv:poses instead.")
  (poses m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <HgCartesianTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_cartesian_trajectory-srv:type-val is deprecated.  Use hg_cartesian_trajectory-srv:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<HgCartesianTrajectory-request>)))
    "Constants for message type '<HgCartesianTrajectory-request>"
  '((:SIMPLE_IK . 0)
    (:LINE . 1)
    (:CIRCLE . 2)
    (:PATH . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'HgCartesianTrajectory-request)))
    "Constants for message type 'HgCartesianTrajectory-request"
  '((:SIMPLE_IK . 0)
    (:LINE . 1)
    (:CIRCLE . 2)
    (:PATH . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HgCartesianTrajectory-request>) ostream)
  "Serializes a message object of type '<HgCartesianTrajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'motion_plan_request) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poses))
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HgCartesianTrajectory-request>) istream)
  "Deserializes a message object of type '<HgCartesianTrajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'motion_plan_request) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HgCartesianTrajectory-request>)))
  "Returns string type for a service object of type '<HgCartesianTrajectory-request>"
  "hg_cartesian_trajectory/HgCartesianTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HgCartesianTrajectory-request)))
  "Returns string type for a service object of type 'HgCartesianTrajectory-request"
  "hg_cartesian_trajectory/HgCartesianTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HgCartesianTrajectory-request>)))
  "Returns md5sum for a message object of type '<HgCartesianTrajectory-request>"
  "c1c3b5534f4766c0c2b10933576473c6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HgCartesianTrajectory-request)))
  "Returns md5sum for a message object of type 'HgCartesianTrajectory-request"
  "c1c3b5534f4766c0c2b10933576473c6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HgCartesianTrajectory-request>)))
  "Returns full string definition for message of type '<HgCartesianTrajectory-request>"
  (cl:format cl:nil "~%Header header~%arm_navigation_msgs/MotionPlanRequest motion_plan_request~%geometry_msgs/Pose[] poses~%int8 type~%~%int8 SIMPLE_IK=0~%int8 LINE=1~%int8 CIRCLE=2~%int8 PATH=3~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: arm_navigation_msgs/MotionPlanRequest~%# This service contains the definition for a request to the motion~%# planner and the output it provides~%~%# Parameters for the workspace that the planner should work inside~%arm_navigation_msgs/WorkspaceParameters workspace_parameters~%~%# Starting state updates. If certain joints should be considered~%# at positions other than the current ones, these positions should~%# be set here~%arm_navigation_msgs/RobotState start_state~%~%# The goal state for the model to plan for. The goal is achieved~%# if all constraints are satisfied~%arm_navigation_msgs/Constraints goal_constraints~%~%# No state at any point along the path in the produced motion plan will violate these constraints~%arm_navigation_msgs/Constraints path_constraints~%~%# The name of the motion planner to use. If no name is specified,~%# a default motion planner will be used~%string planner_id~%~%# The name of the group of joints on which this planner is operating~%string group_name~%~%# The number of times this plan is to be computed. Shortest solution~%# will be reported.~%int32 num_planning_attempts~%~%# The maximum amount of time the motion planner is allowed to plan for~%duration allowed_planning_time~%~%# An expected path duration (in seconds) along with an expected discretization of the path allows the planner to determine the discretization of the trajectory that it returns~%duration expected_path_duration~%duration expected_path_dt~%~%================================================================================~%MSG: arm_navigation_msgs/WorkspaceParameters~%# This message contains a set of parameters useful in~%# setting up the workspace for planning~%arm_navigation_msgs/Shape  workspace_region_shape~%geometry_msgs/PoseStamped    workspace_region_pose~%~%~%================================================================================~%MSG: arm_navigation_msgs/Shape~%byte SPHERE=0~%byte BOX=1~%byte CYLINDER=2~%byte MESH=3~%~%byte type~%~%~%#### define sphere, box, cylinder ####~%# the origin of each shape is considered at the shape's center~%~%# for sphere~%# radius := dimensions[0]~%~%# for cylinder~%# radius := dimensions[0]~%# length := dimensions[1]~%# the length is along the Z axis~%~%# for box~%# size_x := dimensions[0]~%# size_y := dimensions[1]~%# size_z := dimensions[2]~%float64[] dimensions~%~%~%#### define mesh ####~%~%# list of triangles; triangle k is defined by tre vertices located~%# at indices triangles[3k], triangles[3k+1], triangles[3k+2]~%int32[] triangles~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: arm_navigation_msgs/RobotState~%# This message contains information about the robot state, i.e. the positions of its joints and links~%sensor_msgs/JointState joint_state~%arm_navigation_msgs/MultiDOFJointState multi_dof_joint_state~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: arm_navigation_msgs/MultiDOFJointState~%#A representation of a multi-dof joint state~%time stamp~%string[] joint_names~%string[] frame_ids~%string[] child_frame_ids~%geometry_msgs/Pose[] poses~%~%================================================================================~%MSG: arm_navigation_msgs/Constraints~%# This message contains a list of motion planning constraints.~%~%arm_navigation_msgs/JointConstraint[] joint_constraints~%arm_navigation_msgs/PositionConstraint[] position_constraints~%arm_navigation_msgs/OrientationConstraint[] orientation_constraints~%arm_navigation_msgs/VisibilityConstraint[] visibility_constraints~%~%================================================================================~%MSG: arm_navigation_msgs/JointConstraint~%# Constrain the position of a joint to be within a certain bound~%string joint_name~%~%# the bound to be achieved is [position - tolerance_below, position + tolerance_above]~%float64 position~%float64 tolerance_above~%float64 tolerance_below~%~%# A weighting factor for this constraint~%float64 weight~%================================================================================~%MSG: arm_navigation_msgs/PositionConstraint~%# This message contains the definition of a position constraint.~%Header header~%~%# The robot link this constraint refers to~%string link_name~%~%# The offset (in the link frame) for the target point on the link we are planning for~%geometry_msgs/Point target_point_offset~%~%# The nominal/target position for the point we are planning for~%geometry_msgs/Point position~%~%# The shape of the bounded region that constrains the position of the end-effector~%# This region is always centered at the position defined above~%arm_navigation_msgs/Shape constraint_region_shape~%~%# The orientation of the bounded region that constrains the position of the end-effector. ~%# This allows the specification of non-axis aligned constraints~%geometry_msgs/Quaternion constraint_region_orientation~%~%# Constraint weighting factor - a weight for this constraint~%float64 weight~%~%================================================================================~%MSG: arm_navigation_msgs/OrientationConstraint~%# This message contains the definition of an orientation constraint.~%Header header~%~%# The robot link this constraint refers to~%string link_name~%~%# The type of the constraint~%int32 type~%int32 LINK_FRAME=0~%int32 HEADER_FRAME=1~%~%# The desired orientation of the robot link specified as a quaternion~%geometry_msgs/Quaternion orientation~%~%# optional RPY error tolerances specified if ~%float64 absolute_roll_tolerance~%float64 absolute_pitch_tolerance~%float64 absolute_yaw_tolerance~%~%# Constraint weighting factor - a weight for this constraint~%float64 weight~%~%================================================================================~%MSG: arm_navigation_msgs/VisibilityConstraint~%# This message contains the definition of a visibility constraint.~%Header header~%~%# The point stamped target that needs to be kept within view of the sensor~%geometry_msgs/PointStamped target~%~%# The local pose of the frame in which visibility is to be maintained~%# The frame id should represent the robot link to which the sensor is attached~%# The visual axis of the sensor is assumed to be along the X axis of this frame~%geometry_msgs/PoseStamped sensor_pose~%~%# The deviation (in radians) that will be tolerated~%# Constraint error will be measured as the solid angle between the ~%# X axis of the frame defined above and the vector between the origin ~%# of the frame defined above and the target location~%float64 absolute_tolerance~%~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HgCartesianTrajectory-request)))
  "Returns full string definition for message of type 'HgCartesianTrajectory-request"
  (cl:format cl:nil "~%Header header~%arm_navigation_msgs/MotionPlanRequest motion_plan_request~%geometry_msgs/Pose[] poses~%int8 type~%~%int8 SIMPLE_IK=0~%int8 LINE=1~%int8 CIRCLE=2~%int8 PATH=3~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: arm_navigation_msgs/MotionPlanRequest~%# This service contains the definition for a request to the motion~%# planner and the output it provides~%~%# Parameters for the workspace that the planner should work inside~%arm_navigation_msgs/WorkspaceParameters workspace_parameters~%~%# Starting state updates. If certain joints should be considered~%# at positions other than the current ones, these positions should~%# be set here~%arm_navigation_msgs/RobotState start_state~%~%# The goal state for the model to plan for. The goal is achieved~%# if all constraints are satisfied~%arm_navigation_msgs/Constraints goal_constraints~%~%# No state at any point along the path in the produced motion plan will violate these constraints~%arm_navigation_msgs/Constraints path_constraints~%~%# The name of the motion planner to use. If no name is specified,~%# a default motion planner will be used~%string planner_id~%~%# The name of the group of joints on which this planner is operating~%string group_name~%~%# The number of times this plan is to be computed. Shortest solution~%# will be reported.~%int32 num_planning_attempts~%~%# The maximum amount of time the motion planner is allowed to plan for~%duration allowed_planning_time~%~%# An expected path duration (in seconds) along with an expected discretization of the path allows the planner to determine the discretization of the trajectory that it returns~%duration expected_path_duration~%duration expected_path_dt~%~%================================================================================~%MSG: arm_navigation_msgs/WorkspaceParameters~%# This message contains a set of parameters useful in~%# setting up the workspace for planning~%arm_navigation_msgs/Shape  workspace_region_shape~%geometry_msgs/PoseStamped    workspace_region_pose~%~%~%================================================================================~%MSG: arm_navigation_msgs/Shape~%byte SPHERE=0~%byte BOX=1~%byte CYLINDER=2~%byte MESH=3~%~%byte type~%~%~%#### define sphere, box, cylinder ####~%# the origin of each shape is considered at the shape's center~%~%# for sphere~%# radius := dimensions[0]~%~%# for cylinder~%# radius := dimensions[0]~%# length := dimensions[1]~%# the length is along the Z axis~%~%# for box~%# size_x := dimensions[0]~%# size_y := dimensions[1]~%# size_z := dimensions[2]~%float64[] dimensions~%~%~%#### define mesh ####~%~%# list of triangles; triangle k is defined by tre vertices located~%# at indices triangles[3k], triangles[3k+1], triangles[3k+2]~%int32[] triangles~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: arm_navigation_msgs/RobotState~%# This message contains information about the robot state, i.e. the positions of its joints and links~%sensor_msgs/JointState joint_state~%arm_navigation_msgs/MultiDOFJointState multi_dof_joint_state~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: arm_navigation_msgs/MultiDOFJointState~%#A representation of a multi-dof joint state~%time stamp~%string[] joint_names~%string[] frame_ids~%string[] child_frame_ids~%geometry_msgs/Pose[] poses~%~%================================================================================~%MSG: arm_navigation_msgs/Constraints~%# This message contains a list of motion planning constraints.~%~%arm_navigation_msgs/JointConstraint[] joint_constraints~%arm_navigation_msgs/PositionConstraint[] position_constraints~%arm_navigation_msgs/OrientationConstraint[] orientation_constraints~%arm_navigation_msgs/VisibilityConstraint[] visibility_constraints~%~%================================================================================~%MSG: arm_navigation_msgs/JointConstraint~%# Constrain the position of a joint to be within a certain bound~%string joint_name~%~%# the bound to be achieved is [position - tolerance_below, position + tolerance_above]~%float64 position~%float64 tolerance_above~%float64 tolerance_below~%~%# A weighting factor for this constraint~%float64 weight~%================================================================================~%MSG: arm_navigation_msgs/PositionConstraint~%# This message contains the definition of a position constraint.~%Header header~%~%# The robot link this constraint refers to~%string link_name~%~%# The offset (in the link frame) for the target point on the link we are planning for~%geometry_msgs/Point target_point_offset~%~%# The nominal/target position for the point we are planning for~%geometry_msgs/Point position~%~%# The shape of the bounded region that constrains the position of the end-effector~%# This region is always centered at the position defined above~%arm_navigation_msgs/Shape constraint_region_shape~%~%# The orientation of the bounded region that constrains the position of the end-effector. ~%# This allows the specification of non-axis aligned constraints~%geometry_msgs/Quaternion constraint_region_orientation~%~%# Constraint weighting factor - a weight for this constraint~%float64 weight~%~%================================================================================~%MSG: arm_navigation_msgs/OrientationConstraint~%# This message contains the definition of an orientation constraint.~%Header header~%~%# The robot link this constraint refers to~%string link_name~%~%# The type of the constraint~%int32 type~%int32 LINK_FRAME=0~%int32 HEADER_FRAME=1~%~%# The desired orientation of the robot link specified as a quaternion~%geometry_msgs/Quaternion orientation~%~%# optional RPY error tolerances specified if ~%float64 absolute_roll_tolerance~%float64 absolute_pitch_tolerance~%float64 absolute_yaw_tolerance~%~%# Constraint weighting factor - a weight for this constraint~%float64 weight~%~%================================================================================~%MSG: arm_navigation_msgs/VisibilityConstraint~%# This message contains the definition of a visibility constraint.~%Header header~%~%# The point stamped target that needs to be kept within view of the sensor~%geometry_msgs/PointStamped target~%~%# The local pose of the frame in which visibility is to be maintained~%# The frame id should represent the robot link to which the sensor is attached~%# The visual axis of the sensor is assumed to be along the X axis of this frame~%geometry_msgs/PoseStamped sensor_pose~%~%# The deviation (in radians) that will be tolerated~%# Constraint error will be measured as the solid angle between the ~%# X axis of the frame defined above and the vector between the origin ~%# of the frame defined above and the target location~%float64 absolute_tolerance~%~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HgCartesianTrajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'motion_plan_request))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HgCartesianTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'HgCartesianTrajectory-request
    (cl:cons ':header (header msg))
    (cl:cons ':motion_plan_request (motion_plan_request msg))
    (cl:cons ':poses (poses msg))
    (cl:cons ':type (type msg))
))
;//! \htmlinclude HgCartesianTrajectory-response.msg.html

(cl:defclass <HgCartesianTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((trajectory
    :reader trajectory
    :initarg :trajectory
    :type arm_navigation_msgs-msg:RobotTrajectory
    :initform (cl:make-instance 'arm_navigation_msgs-msg:RobotTrajectory))
   (error_code
    :reader error_code
    :initarg :error_code
    :type arm_navigation_msgs-msg:ArmNavigationErrorCodes
    :initform (cl:make-instance 'arm_navigation_msgs-msg:ArmNavigationErrorCodes))
   (trajectory_error_codes
    :reader trajectory_error_codes
    :initarg :trajectory_error_codes
    :type (cl:vector arm_navigation_msgs-msg:ArmNavigationErrorCodes)
   :initform (cl:make-array 0 :element-type 'arm_navigation_msgs-msg:ArmNavigationErrorCodes :initial-element (cl:make-instance 'arm_navigation_msgs-msg:ArmNavigationErrorCodes))))
)

(cl:defclass HgCartesianTrajectory-response (<HgCartesianTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HgCartesianTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HgCartesianTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hg_cartesian_trajectory-srv:<HgCartesianTrajectory-response> is deprecated: use hg_cartesian_trajectory-srv:HgCartesianTrajectory-response instead.")))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <HgCartesianTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_cartesian_trajectory-srv:trajectory-val is deprecated.  Use hg_cartesian_trajectory-srv:trajectory instead.")
  (trajectory m))

(cl:ensure-generic-function 'error_code-val :lambda-list '(m))
(cl:defmethod error_code-val ((m <HgCartesianTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_cartesian_trajectory-srv:error_code-val is deprecated.  Use hg_cartesian_trajectory-srv:error_code instead.")
  (error_code m))

(cl:ensure-generic-function 'trajectory_error_codes-val :lambda-list '(m))
(cl:defmethod trajectory_error_codes-val ((m <HgCartesianTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hg_cartesian_trajectory-srv:trajectory_error_codes-val is deprecated.  Use hg_cartesian_trajectory-srv:trajectory_error_codes instead.")
  (trajectory_error_codes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HgCartesianTrajectory-response>) ostream)
  "Serializes a message object of type '<HgCartesianTrajectory-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectory) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'error_code) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trajectory_error_codes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'trajectory_error_codes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HgCartesianTrajectory-response>) istream)
  "Deserializes a message object of type '<HgCartesianTrajectory-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectory) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'error_code) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trajectory_error_codes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trajectory_error_codes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'arm_navigation_msgs-msg:ArmNavigationErrorCodes))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HgCartesianTrajectory-response>)))
  "Returns string type for a service object of type '<HgCartesianTrajectory-response>"
  "hg_cartesian_trajectory/HgCartesianTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HgCartesianTrajectory-response)))
  "Returns string type for a service object of type 'HgCartesianTrajectory-response"
  "hg_cartesian_trajectory/HgCartesianTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HgCartesianTrajectory-response>)))
  "Returns md5sum for a message object of type '<HgCartesianTrajectory-response>"
  "c1c3b5534f4766c0c2b10933576473c6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HgCartesianTrajectory-response)))
  "Returns md5sum for a message object of type 'HgCartesianTrajectory-response"
  "c1c3b5534f4766c0c2b10933576473c6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HgCartesianTrajectory-response>)))
  "Returns full string definition for message of type '<HgCartesianTrajectory-response>"
  (cl:format cl:nil "~%arm_navigation_msgs/RobotTrajectory trajectory~%~%~%arm_navigation_msgs/ArmNavigationErrorCodes error_code~%~%~%arm_navigation_msgs/ArmNavigationErrorCodes[] trajectory_error_codes~%~%================================================================================~%MSG: arm_navigation_msgs/RobotTrajectory~%trajectory_msgs/JointTrajectory joint_trajectory~%arm_navigation_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%================================================================================~%MSG: arm_navigation_msgs/MultiDOFJointTrajectory~%#A representation of a multi-dof joint trajectory~%duration stamp~%string[] joint_names~%string[] frame_ids~%string[] child_frame_ids~%MultiDOFJointTrajectoryPoint[] points~%~%================================================================================~%MSG: arm_navigation_msgs/MultiDOFJointTrajectoryPoint~%geometry_msgs/Pose[] poses~%duration time_from_start~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: arm_navigation_msgs/ArmNavigationErrorCodes~%int32 val~%~%# overall behavior~%int32 PLANNING_FAILED=-1~%int32 SUCCESS=1~%int32 TIMED_OUT=-2~%~%# start state errors~%int32 START_STATE_IN_COLLISION=-3~%int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-4~%~%# goal errors~%int32 GOAL_IN_COLLISION=-5~%int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-6~%~%# robot state~%int32 INVALID_ROBOT_STATE=-7~%int32 INCOMPLETE_ROBOT_STATE=-8~%~%# planning request errors~%int32 INVALID_PLANNER_ID=-9~%int32 INVALID_NUM_PLANNING_ATTEMPTS=-10~%int32 INVALID_ALLOWED_PLANNING_TIME=-11~%int32 INVALID_GROUP_NAME=-12~%int32 INVALID_GOAL_JOINT_CONSTRAINTS=-13~%int32 INVALID_GOAL_POSITION_CONSTRAINTS=-14~%int32 INVALID_GOAL_ORIENTATION_CONSTRAINTS=-15~%int32 INVALID_PATH_JOINT_CONSTRAINTS=-16~%int32 INVALID_PATH_POSITION_CONSTRAINTS=-17~%int32 INVALID_PATH_ORIENTATION_CONSTRAINTS=-18~%~%# state/trajectory monitor errors~%int32 INVALID_TRAJECTORY=-19~%int32 INVALID_INDEX=-20~%int32 JOINT_LIMITS_VIOLATED=-21~%int32 PATH_CONSTRAINTS_VIOLATED=-22~%int32 COLLISION_CONSTRAINTS_VIOLATED=-23~%int32 GOAL_CONSTRAINTS_VIOLATED=-24~%int32 JOINTS_NOT_MOVING=-25~%int32 TRAJECTORY_CONTROLLER_FAILED=-26~%~%# system errors~%int32 FRAME_TRANSFORM_FAILURE=-27~%int32 COLLISION_CHECKING_UNAVAILABLE=-28~%int32 ROBOT_STATE_STALE=-29~%int32 SENSOR_INFO_STALE=-30~%~%# kinematics errors~%int32 NO_IK_SOLUTION=-31~%int32 INVALID_LINK_NAME=-32~%int32 IK_LINK_IN_COLLISION=-33~%int32 NO_FK_SOLUTION=-34~%int32 KINEMATICS_STATE_IN_COLLISION=-35~%~%# general errors~%int32 INVALID_TIMEOUT=-36~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HgCartesianTrajectory-response)))
  "Returns full string definition for message of type 'HgCartesianTrajectory-response"
  (cl:format cl:nil "~%arm_navigation_msgs/RobotTrajectory trajectory~%~%~%arm_navigation_msgs/ArmNavigationErrorCodes error_code~%~%~%arm_navigation_msgs/ArmNavigationErrorCodes[] trajectory_error_codes~%~%================================================================================~%MSG: arm_navigation_msgs/RobotTrajectory~%trajectory_msgs/JointTrajectory joint_trajectory~%arm_navigation_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%================================================================================~%MSG: arm_navigation_msgs/MultiDOFJointTrajectory~%#A representation of a multi-dof joint trajectory~%duration stamp~%string[] joint_names~%string[] frame_ids~%string[] child_frame_ids~%MultiDOFJointTrajectoryPoint[] points~%~%================================================================================~%MSG: arm_navigation_msgs/MultiDOFJointTrajectoryPoint~%geometry_msgs/Pose[] poses~%duration time_from_start~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: arm_navigation_msgs/ArmNavigationErrorCodes~%int32 val~%~%# overall behavior~%int32 PLANNING_FAILED=-1~%int32 SUCCESS=1~%int32 TIMED_OUT=-2~%~%# start state errors~%int32 START_STATE_IN_COLLISION=-3~%int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-4~%~%# goal errors~%int32 GOAL_IN_COLLISION=-5~%int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-6~%~%# robot state~%int32 INVALID_ROBOT_STATE=-7~%int32 INCOMPLETE_ROBOT_STATE=-8~%~%# planning request errors~%int32 INVALID_PLANNER_ID=-9~%int32 INVALID_NUM_PLANNING_ATTEMPTS=-10~%int32 INVALID_ALLOWED_PLANNING_TIME=-11~%int32 INVALID_GROUP_NAME=-12~%int32 INVALID_GOAL_JOINT_CONSTRAINTS=-13~%int32 INVALID_GOAL_POSITION_CONSTRAINTS=-14~%int32 INVALID_GOAL_ORIENTATION_CONSTRAINTS=-15~%int32 INVALID_PATH_JOINT_CONSTRAINTS=-16~%int32 INVALID_PATH_POSITION_CONSTRAINTS=-17~%int32 INVALID_PATH_ORIENTATION_CONSTRAINTS=-18~%~%# state/trajectory monitor errors~%int32 INVALID_TRAJECTORY=-19~%int32 INVALID_INDEX=-20~%int32 JOINT_LIMITS_VIOLATED=-21~%int32 PATH_CONSTRAINTS_VIOLATED=-22~%int32 COLLISION_CONSTRAINTS_VIOLATED=-23~%int32 GOAL_CONSTRAINTS_VIOLATED=-24~%int32 JOINTS_NOT_MOVING=-25~%int32 TRAJECTORY_CONTROLLER_FAILED=-26~%~%# system errors~%int32 FRAME_TRANSFORM_FAILURE=-27~%int32 COLLISION_CHECKING_UNAVAILABLE=-28~%int32 ROBOT_STATE_STALE=-29~%int32 SENSOR_INFO_STALE=-30~%~%# kinematics errors~%int32 NO_IK_SOLUTION=-31~%int32 INVALID_LINK_NAME=-32~%int32 IK_LINK_IN_COLLISION=-33~%int32 NO_FK_SOLUTION=-34~%int32 KINEMATICS_STATE_IN_COLLISION=-35~%~%# general errors~%int32 INVALID_TIMEOUT=-36~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HgCartesianTrajectory-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectory))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'error_code))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trajectory_error_codes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HgCartesianTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'HgCartesianTrajectory-response
    (cl:cons ':trajectory (trajectory msg))
    (cl:cons ':error_code (error_code msg))
    (cl:cons ':trajectory_error_codes (trajectory_error_codes msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'HgCartesianTrajectory)))
  'HgCartesianTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'HgCartesianTrajectory)))
  'HgCartesianTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HgCartesianTrajectory)))
  "Returns string type for a service object of type '<HgCartesianTrajectory>"
  "hg_cartesian_trajectory/HgCartesianTrajectory")