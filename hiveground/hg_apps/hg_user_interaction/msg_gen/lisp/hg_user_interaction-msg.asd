
(cl:in-package :asdf)

(defsystem "hg_user_interaction-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Gesture" :depends-on ("_package_Gesture"))
    (:file "_package_Gesture" :depends-on ("_package"))
    (:file "Gestures" :depends-on ("_package_Gestures"))
    (:file "_package_Gestures" :depends-on ("_package"))
  ))