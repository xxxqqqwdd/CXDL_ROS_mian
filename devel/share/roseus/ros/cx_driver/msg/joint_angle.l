;; Auto-generated. Do not edit!


(when (boundp 'cx_driver::joint_angle)
  (if (not (find-package "CX_DRIVER"))
    (make-package "CX_DRIVER"))
  (shadow 'joint_angle (find-package "CX_DRIVER")))
(unless (find-package "CX_DRIVER::JOINT_ANGLE")
  (make-package "CX_DRIVER::JOINT_ANGLE"))

(in-package "ROS")
;;//! \htmlinclude joint_angle.msg.html


(defclass cx_driver::joint_angle
  :super ros::object
  :slots (_left_arm_joint _right_arm_joint ))

(defmethod cx_driver::joint_angle
  (:init
   (&key
    ((:left_arm_joint __left_arm_joint) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:right_arm_joint __right_arm_joint) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _left_arm_joint __left_arm_joint)
   (setq _right_arm_joint __right_arm_joint)
   self)
  (:left_arm_joint
   (&optional __left_arm_joint)
   (if __left_arm_joint (setq _left_arm_joint __left_arm_joint)) _left_arm_joint)
  (:right_arm_joint
   (&optional __right_arm_joint)
   (if __right_arm_joint (setq _right_arm_joint __right_arm_joint)) _right_arm_joint)
  (:serialization-length
   ()
   (+
    ;; float64[6] _left_arm_joint
    (* 8    6)
    ;; float64[6] _right_arm_joint
    (* 8    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _left_arm_joint
     (dotimes (i 6)
       (sys::poke (elt _left_arm_joint i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _right_arm_joint
     (dotimes (i 6)
       (sys::poke (elt _right_arm_joint i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _left_arm_joint
   (dotimes (i (length _left_arm_joint))
     (setf (elt _left_arm_joint i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _right_arm_joint
   (dotimes (i (length _right_arm_joint))
     (setf (elt _right_arm_joint i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get cx_driver::joint_angle :md5sum-) "990341802786d12533e60a9263f3d7c1")
(setf (get cx_driver::joint_angle :datatype-) "cx_driver/joint_angle")
(setf (get cx_driver::joint_angle :definition-)
      "float64[6] left_arm_joint
float64[6] right_arm_joint


")



(provide :cx_driver/joint_angle "990341802786d12533e60a9263f3d7c1")


