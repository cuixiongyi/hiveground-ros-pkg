
(cl:in-package :asdf)

(defsystem "hg_hand_interaction-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "OneHandGesture" :depends-on ("_package_OneHandGesture"))
    (:file "_package_OneHandGesture" :depends-on ("_package"))
    (:file "TwoHandGesture" :depends-on ("_package_TwoHandGesture"))
    (:file "_package_TwoHandGesture" :depends-on ("_package"))
  ))