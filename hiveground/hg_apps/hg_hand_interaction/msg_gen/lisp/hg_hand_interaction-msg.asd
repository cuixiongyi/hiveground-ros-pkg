
(cl:in-package :asdf)

(defsystem "hg_hand_interaction-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "HandGesture" :depends-on ("_package_HandGesture"))
    (:file "_package_HandGesture" :depends-on ("_package"))
    (:file "HandGestures" :depends-on ("_package_HandGestures"))
    (:file "_package_HandGestures" :depends-on ("_package"))
  ))