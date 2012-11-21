
(cl:in-package :asdf)

(defsystem "prw_message-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ObjectArray" :depends-on ("_package_ObjectArray"))
    (:file "_package_ObjectArray" :depends-on ("_package"))
    (:file "Object" :depends-on ("_package_Object"))
    (:file "_package_Object" :depends-on ("_package"))
  ))