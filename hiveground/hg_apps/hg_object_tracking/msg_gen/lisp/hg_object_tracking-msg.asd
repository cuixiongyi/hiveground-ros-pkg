
(cl:in-package :asdf)

(defsystem "hg_object_tracking-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Hand" :depends-on ("_package_Hand"))
    (:file "_package_Hand" :depends-on ("_package"))
    (:file "Hands" :depends-on ("_package_Hands"))
    (:file "_package_Hands" :depends-on ("_package"))
  ))