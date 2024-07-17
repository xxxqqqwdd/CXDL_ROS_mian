; Auto-generated. Do not edit!


(cl:in-package cx_driver-msg)


;//! \htmlinclude joint_angle.msg.html

(cl:defclass <joint_angle> (roslisp-msg-protocol:ros-message)
  ((left_arm_joint
    :reader left_arm_joint
    :initarg :left_arm_joint
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (right_arm_joint
    :reader right_arm_joint
    :initarg :right_arm_joint
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass joint_angle (<joint_angle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <joint_angle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'joint_angle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cx_driver-msg:<joint_angle> is deprecated: use cx_driver-msg:joint_angle instead.")))

(cl:ensure-generic-function 'left_arm_joint-val :lambda-list '(m))
(cl:defmethod left_arm_joint-val ((m <joint_angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cx_driver-msg:left_arm_joint-val is deprecated.  Use cx_driver-msg:left_arm_joint instead.")
  (left_arm_joint m))

(cl:ensure-generic-function 'right_arm_joint-val :lambda-list '(m))
(cl:defmethod right_arm_joint-val ((m <joint_angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cx_driver-msg:right_arm_joint-val is deprecated.  Use cx_driver-msg:right_arm_joint instead.")
  (right_arm_joint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <joint_angle>) ostream)
  "Serializes a message object of type '<joint_angle>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'left_arm_joint))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'right_arm_joint))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <joint_angle>) istream)
  "Deserializes a message object of type '<joint_angle>"
  (cl:setf (cl:slot-value msg 'left_arm_joint) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'left_arm_joint)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'right_arm_joint) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'right_arm_joint)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<joint_angle>)))
  "Returns string type for a message object of type '<joint_angle>"
  "cx_driver/joint_angle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'joint_angle)))
  "Returns string type for a message object of type 'joint_angle"
  "cx_driver/joint_angle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<joint_angle>)))
  "Returns md5sum for a message object of type '<joint_angle>"
  "990341802786d12533e60a9263f3d7c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'joint_angle)))
  "Returns md5sum for a message object of type 'joint_angle"
  "990341802786d12533e60a9263f3d7c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<joint_angle>)))
  "Returns full string definition for message of type '<joint_angle>"
  (cl:format cl:nil "float64[6] left_arm_joint~%float64[6] right_arm_joint~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'joint_angle)))
  "Returns full string definition for message of type 'joint_angle"
  (cl:format cl:nil "float64[6] left_arm_joint~%float64[6] right_arm_joint~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <joint_angle>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'left_arm_joint) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'right_arm_joint) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <joint_angle>))
  "Converts a ROS message object to a list"
  (cl:list 'joint_angle
    (cl:cons ':left_arm_joint (left_arm_joint msg))
    (cl:cons ':right_arm_joint (right_arm_joint msg))
))
