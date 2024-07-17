
(cl:in-package :asdf)

(defsystem "cx_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "joint_angle" :depends-on ("_package_joint_angle"))
    (:file "_package_joint_angle" :depends-on ("_package"))
    (:file "motor_info" :depends-on ("_package_motor_info"))
    (:file "_package_motor_info" :depends-on ("_package"))
  ))