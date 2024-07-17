; Auto-generated. Do not edit!


(cl:in-package cx_driver-msg)


;//! \htmlinclude motor_info.msg.html

(cl:defclass <motor_info> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (is_rtr
    :reader is_rtr
    :initarg :is_rtr
    :type cl:boolean
    :initform cl:nil)
   (is_extended
    :reader is_extended
    :initarg :is_extended
    :type cl:boolean
    :initform cl:nil)
   (is_error
    :reader is_error
    :initarg :is_error
    :type cl:boolean
    :initform cl:nil)
   (dlc
    :reader dlc
    :initarg :dlc
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 8 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass motor_info (<motor_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motor_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motor_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cx_driver-msg:<motor_info> is deprecated: use cx_driver-msg:motor_info instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <motor_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cx_driver-msg:id-val is deprecated.  Use cx_driver-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'is_rtr-val :lambda-list '(m))
(cl:defmethod is_rtr-val ((m <motor_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cx_driver-msg:is_rtr-val is deprecated.  Use cx_driver-msg:is_rtr instead.")
  (is_rtr m))

(cl:ensure-generic-function 'is_extended-val :lambda-list '(m))
(cl:defmethod is_extended-val ((m <motor_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cx_driver-msg:is_extended-val is deprecated.  Use cx_driver-msg:is_extended instead.")
  (is_extended m))

(cl:ensure-generic-function 'is_error-val :lambda-list '(m))
(cl:defmethod is_error-val ((m <motor_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cx_driver-msg:is_error-val is deprecated.  Use cx_driver-msg:is_error instead.")
  (is_error m))

(cl:ensure-generic-function 'dlc-val :lambda-list '(m))
(cl:defmethod dlc-val ((m <motor_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cx_driver-msg:dlc-val is deprecated.  Use cx_driver-msg:dlc instead.")
  (dlc m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <motor_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cx_driver-msg:data-val is deprecated.  Use cx_driver-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motor_info>) ostream)
  "Serializes a message object of type '<motor_info>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_rtr) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_extended) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dlc)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motor_info>) istream)
  "Deserializes a message object of type '<motor_info>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'is_rtr) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_extended) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dlc)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i 8)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motor_info>)))
  "Returns string type for a message object of type '<motor_info>"
  "cx_driver/motor_info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_info)))
  "Returns string type for a message object of type 'motor_info"
  "cx_driver/motor_info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motor_info>)))
  "Returns md5sum for a message object of type '<motor_info>"
  "e581c8420d38dcd98d0263cda11f3fb6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motor_info)))
  "Returns md5sum for a message object of type 'motor_info"
  "e581c8420d38dcd98d0263cda11f3fb6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motor_info>)))
  "Returns full string definition for message of type '<motor_info>"
  (cl:format cl:nil "uint32 id~%bool is_rtr~%bool is_extended~%bool is_error~%uint8 dlc~%uint8[8] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motor_info)))
  "Returns full string definition for message of type 'motor_info"
  (cl:format cl:nil "uint32 id~%bool is_rtr~%bool is_extended~%bool is_error~%uint8 dlc~%uint8[8] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motor_info>))
  (cl:+ 0
     4
     1
     1
     1
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motor_info>))
  "Converts a ROS message object to a list"
  (cl:list 'motor_info
    (cl:cons ':id (id msg))
    (cl:cons ':is_rtr (is_rtr msg))
    (cl:cons ':is_extended (is_extended msg))
    (cl:cons ':is_error (is_error msg))
    (cl:cons ':dlc (dlc msg))
    (cl:cons ':data (data msg))
))
