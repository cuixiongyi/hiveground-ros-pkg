
(cl:in-package :asdf)

(defsystem "hg_cartesian_trajectory-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "HgCartesianTrajectory" :depends-on ("_package_HgCartesianTrajectory"))
    (:file "_package_HgCartesianTrajectory" :depends-on ("_package"))
  ))