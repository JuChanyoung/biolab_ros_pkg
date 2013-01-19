; Auto-generated. Do not edit!


(cl:in-package brl_teleop_msgs-msg)


;//! \htmlinclude Package.msg.html

(cl:defclass <Package> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (fx
    :reader fx
    :initarg :fx
    :type cl:float
    :initform 0.0)
   (fy
    :reader fy
    :initarg :fy
    :type cl:float
    :initform 0.0)
   (fz
    :reader fz
    :initarg :fz
    :type cl:float
    :initform 0.0)
   (Ex
    :reader Ex
    :initarg :Ex
    :type cl:float
    :initform 0.0)
   (Ey
    :reader Ey
    :initarg :Ey
    :type cl:float
    :initform 0.0)
   (Ez
    :reader Ez
    :initarg :Ez
    :type cl:float
    :initform 0.0))
)

(cl:defclass Package (<Package>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Package>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Package)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name brl_teleop_msgs-msg:<Package> is deprecated: use brl_teleop_msgs-msg:Package instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Package>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brl_teleop_msgs-msg:header-val is deprecated.  Use brl_teleop_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Package>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brl_teleop_msgs-msg:x-val is deprecated.  Use brl_teleop_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Package>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brl_teleop_msgs-msg:y-val is deprecated.  Use brl_teleop_msgs-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <Package>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brl_teleop_msgs-msg:z-val is deprecated.  Use brl_teleop_msgs-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'fx-val :lambda-list '(m))
(cl:defmethod fx-val ((m <Package>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brl_teleop_msgs-msg:fx-val is deprecated.  Use brl_teleop_msgs-msg:fx instead.")
  (fx m))

(cl:ensure-generic-function 'fy-val :lambda-list '(m))
(cl:defmethod fy-val ((m <Package>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brl_teleop_msgs-msg:fy-val is deprecated.  Use brl_teleop_msgs-msg:fy instead.")
  (fy m))

(cl:ensure-generic-function 'fz-val :lambda-list '(m))
(cl:defmethod fz-val ((m <Package>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brl_teleop_msgs-msg:fz-val is deprecated.  Use brl_teleop_msgs-msg:fz instead.")
  (fz m))

(cl:ensure-generic-function 'Ex-val :lambda-list '(m))
(cl:defmethod Ex-val ((m <Package>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brl_teleop_msgs-msg:Ex-val is deprecated.  Use brl_teleop_msgs-msg:Ex instead.")
  (Ex m))

(cl:ensure-generic-function 'Ey-val :lambda-list '(m))
(cl:defmethod Ey-val ((m <Package>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brl_teleop_msgs-msg:Ey-val is deprecated.  Use brl_teleop_msgs-msg:Ey instead.")
  (Ey m))

(cl:ensure-generic-function 'Ez-val :lambda-list '(m))
(cl:defmethod Ez-val ((m <Package>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brl_teleop_msgs-msg:Ez-val is deprecated.  Use brl_teleop_msgs-msg:Ez instead.")
  (Ez m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Package>) ostream)
  "Serializes a message object of type '<Package>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Ex))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Ey))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Ez))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Package>) istream)
  "Deserializes a message object of type '<Package>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Ex) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Ey) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Ez) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Package>)))
  "Returns string type for a message object of type '<Package>"
  "brl_teleop_msgs/Package")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Package)))
  "Returns string type for a message object of type 'Package"
  "brl_teleop_msgs/Package")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Package>)))
  "Returns md5sum for a message object of type '<Package>"
  "fb30a7db79b96e58ecb61226f90eb3f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Package)))
  "Returns md5sum for a message object of type 'Package"
  "fb30a7db79b96e58ecb61226f90eb3f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Package>)))
  "Returns full string definition for message of type '<Package>"
  (cl:format cl:nil "# This is definition of common message.~%Header header~%~%float32 x	~%float32 y~%float32 z~%~%float32 fx~%float32 fy~%float32 fz~%~%float32 Ex~%float32 Ey~%float32 Ez~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Package)))
  "Returns full string definition for message of type 'Package"
  (cl:format cl:nil "# This is definition of common message.~%Header header~%~%float32 x	~%float32 y~%float32 z~%~%float32 fx~%float32 fy~%float32 fz~%~%float32 Ex~%float32 Ey~%float32 Ez~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Package>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Package>))
  "Converts a ROS message object to a list"
  (cl:list 'Package
    (cl:cons ':header (header msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':fx (fx msg))
    (cl:cons ':fy (fy msg))
    (cl:cons ':fz (fz msg))
    (cl:cons ':Ex (Ex msg))
    (cl:cons ':Ey (Ey msg))
    (cl:cons ':Ez (Ez msg))
))
