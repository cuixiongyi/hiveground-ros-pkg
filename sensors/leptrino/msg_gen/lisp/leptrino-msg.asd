
(cl:in-package :asdf)

(defsystem "leptrino-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ForceTorque" :depends-on ("_package_ForceTorque"))
    (:file "_package_ForceTorque" :depends-on ("_package"))
  ))