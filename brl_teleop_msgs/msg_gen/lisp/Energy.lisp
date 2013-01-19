; Auto-generated. Do not edit!


(cl:in-package brl_teleop_msgs-msg)


;//! \htmlinclude Energy.msg.html

(cl:defclass <Energy> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
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

(cl:defclass Energy (<Energy>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Energy>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Energy)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name brl_teleop_msgs-msg:<Energy> is deprecated: use brl_teleop_msgs-msg:Energy instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Energy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brl_teleop_msgs-msg:header-val is deprecated.  Use brl_teleop_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'Ex-val :lambda-list '(m))
(cl:defmethod Ex-val ((m <Energy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brl_teleop_msgs-msg:Ex-val is deprecated.  Use brl_teleop_msgs-msg:Ex instead.")
  (Ex m))

(cl:ensure-generic-function 'Ey-val :lambda-list '(m))
(cl:defmethod Ey-val ((m <Energy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brl_teleop_msgs-msg:Ey-val is deprecated.  Use brl_teleop_msgs-msg:Ey instead.")
  (Ey m))

(cl:ensure-generic-function 'Ez-val :lambda-list '(m))
(cl:defmethod Ez-val ((m <Energy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brl_teleop_msgs-msg:Ez-val is deprecated.  Use brl_teleop_msgs-msg:Ez instead.")
  (Ez m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Energy>) ostream)
  "Serializes a message object of type '<Energy>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Energy>) istream)
  "Deserializes a message object of type '<Energy>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Energy>)))
  "Returns string type for a message object of type '<Energy>"
  "brl_teleop_msgs/Energy")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Energy)))
  "Returns string type for a message object of type 'Energy"
  "brl_teleop_msgs/Energy")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Energy>)))
  "Returns md5sum for a message object of type '<Energy>"
  "d6d27ae3a18c83ba5c7546d84ca7c4e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Energy)))
  "Returns md5sum for a message object of type 'Energy"
  "d6d27ae3a18c83ba5c7546d84ca7c4e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Energy>)))
  "Returns full string definition for message of type '<Energy>"
  (cl:format cl:nil "# This is definition of 3D energy message.~%Header header~%float32 Ex~%float32 Ey~%float32 Ez~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Energy)))
  "Returns full string definition for message of type 'Energy"
  (cl:format cl:nil "# This is definition of 3D energy message.~%Header header~%float32 Ex~%float32 Ey~%float32 Ez~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Energy>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Energy>))
  "Converts a ROS message object to a list"
  (cl:list 'Energy
    (cl:cons ':header (header msg))
    (cl:cons ':Ex (Ex msg))
    (cl:cons ':Ey (Ey msg))
    (cl:cons ':Ez (Ez msg))
))
