;; Auto-generated. Do not edit!


(when (boundp 'cx_driver::motor_info)
  (if (not (find-package "CX_DRIVER"))
    (make-package "CX_DRIVER"))
  (shadow 'motor_info (find-package "CX_DRIVER")))
(unless (find-package "CX_DRIVER::MOTOR_INFO")
  (make-package "CX_DRIVER::MOTOR_INFO"))

(in-package "ROS")
;;//! \htmlinclude motor_info.msg.html


(defclass cx_driver::motor_info
  :super ros::object
  :slots (_id _is_rtr _is_extended _is_error _dlc _data ))

(defmethod cx_driver::motor_info
  (:init
   (&key
    ((:id __id) 0)
    ((:is_rtr __is_rtr) nil)
    ((:is_extended __is_extended) nil)
    ((:is_error __is_error) nil)
    ((:dlc __dlc) 0)
    ((:data __data) (make-array 8 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _is_rtr __is_rtr)
   (setq _is_extended __is_extended)
   (setq _is_error __is_error)
   (setq _dlc (round __dlc))
   (setq _data __data)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:is_rtr
   (&optional (__is_rtr :null))
   (if (not (eq __is_rtr :null)) (setq _is_rtr __is_rtr)) _is_rtr)
  (:is_extended
   (&optional (__is_extended :null))
   (if (not (eq __is_extended :null)) (setq _is_extended __is_extended)) _is_extended)
  (:is_error
   (&optional (__is_error :null))
   (if (not (eq __is_error :null)) (setq _is_error __is_error)) _is_error)
  (:dlc
   (&optional __dlc)
   (if __dlc (setq _dlc __dlc)) _dlc)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; uint32 _id
    4
    ;; bool _is_rtr
    1
    ;; bool _is_extended
    1
    ;; bool _is_error
    1
    ;; uint8 _dlc
    1
    ;; uint8[8] _data
    (* 1    8)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _id
       (write-long _id s)
     ;; bool _is_rtr
       (if _is_rtr (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_extended
       (if _is_extended (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_error
       (if _is_error (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _dlc
       (write-byte _dlc s)
     ;; uint8[8] _data
     (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _is_rtr
     (setq _is_rtr (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_extended
     (setq _is_extended (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_error
     (setq _is_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _dlc
     (setq _dlc (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[8] _data
   (setq _data (make-array 8 :element-type :char))
   (replace _data buf :start2 ptr-) (incf ptr- 8)
   ;;
   self)
  )

(setf (get cx_driver::motor_info :md5sum-) "e581c8420d38dcd98d0263cda11f3fb6")
(setf (get cx_driver::motor_info :datatype-) "cx_driver/motor_info")
(setf (get cx_driver::motor_info :definition-)
      "uint32 id
bool is_rtr
bool is_extended
bool is_error
uint8 dlc
uint8[8] data
")



(provide :cx_driver/motor_info "e581c8420d38dcd98d0263cda11f3fb6")


