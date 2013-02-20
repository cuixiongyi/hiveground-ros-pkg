
(cl:in-package :asdf)

(defsystem "kinect_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Skeleton" :depends-on ("_package_Skeleton"))
    (:file "_package_Skeleton" :depends-on ("_package"))
    (:file "SkeletonPositionTrackingState" :depends-on ("_package_SkeletonPositionTrackingState"))
    (:file "_package_SkeletonPositionTrackingState" :depends-on ("_package"))
    (:file "SkeletonTrackingState" :depends-on ("_package_SkeletonTrackingState"))
    (:file "_package_SkeletonTrackingState" :depends-on ("_package"))
    (:file "Skeletons" :depends-on ("_package_Skeletons"))
    (:file "_package_Skeletons" :depends-on ("_package"))
  ))