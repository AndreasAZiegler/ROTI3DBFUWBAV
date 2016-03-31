; Auto-generated. Do not edit!


(cl:in-package uwb-msg)


;//! \htmlinclude UWBRange.msg.html

(cl:defclass <UWBRange> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (address
    :reader address
    :initarg :address
    :type cl:fixnum
    :initform 0)
   (remote_address
    :reader remote_address
    :initarg :remote_address
    :type cl:fixnum
    :initform 0)
   (stamp_us
    :reader stamp_us
    :initarg :stamp_us
    :type cl:integer
    :initform 0)
   (round_trip_time
    :reader round_trip_time
    :initarg :round_trip_time
    :type cl:float
    :initform 0.0)
   (range
    :reader range
    :initarg :range
    :type cl:float
    :initform 0.0))
)

(cl:defclass UWBRange (<UWBRange>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UWBRange>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UWBRange)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uwb-msg:<UWBRange> is deprecated: use uwb-msg:UWBRange instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <UWBRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:header-val is deprecated.  Use uwb-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <UWBRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:address-val is deprecated.  Use uwb-msg:address instead.")
  (address m))

(cl:ensure-generic-function 'remote_address-val :lambda-list '(m))
(cl:defmethod remote_address-val ((m <UWBRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:remote_address-val is deprecated.  Use uwb-msg:remote_address instead.")
  (remote_address m))

(cl:ensure-generic-function 'stamp_us-val :lambda-list '(m))
(cl:defmethod stamp_us-val ((m <UWBRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:stamp_us-val is deprecated.  Use uwb-msg:stamp_us instead.")
  (stamp_us m))

(cl:ensure-generic-function 'round_trip_time-val :lambda-list '(m))
(cl:defmethod round_trip_time-val ((m <UWBRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:round_trip_time-val is deprecated.  Use uwb-msg:round_trip_time instead.")
  (round_trip_time m))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <UWBRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:range-val is deprecated.  Use uwb-msg:range instead.")
  (range m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UWBRange>) ostream)
  "Serializes a message object of type '<UWBRange>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'remote_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stamp_us)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'stamp_us)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'stamp_us)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'stamp_us)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'round_trip_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UWBRange>) istream)
  "Deserializes a message object of type '<UWBRange>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'remote_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stamp_us)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'stamp_us)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'stamp_us)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'stamp_us)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'round_trip_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UWBRange>)))
  "Returns string type for a message object of type '<UWBRange>"
  "uwb/UWBRange")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UWBRange)))
  "Returns string type for a message object of type 'UWBRange"
  "uwb/UWBRange")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UWBRange>)))
  "Returns md5sum for a message object of type '<UWBRange>"
  "59f7e9a9fe44ac74dcea199a0dda5d66")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UWBRange)))
  "Returns md5sum for a message object of type 'UWBRange"
  "59f7e9a9fe44ac74dcea199a0dda5d66")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UWBRange>)))
  "Returns full string definition for message of type '<UWBRange>"
  (cl:format cl:nil "Header header~%uint8 address~%uint8 remote_address~%uint32 stamp_us~%float32 round_trip_time~%float32 range~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UWBRange)))
  "Returns full string definition for message of type 'UWBRange"
  (cl:format cl:nil "Header header~%uint8 address~%uint8 remote_address~%uint32 stamp_us~%float32 round_trip_time~%float32 range~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UWBRange>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UWBRange>))
  "Converts a ROS message object to a list"
  (cl:list 'UWBRange
    (cl:cons ':header (header msg))
    (cl:cons ':address (address msg))
    (cl:cons ':remote_address (remote_address msg))
    (cl:cons ':stamp_us (stamp_us msg))
    (cl:cons ':round_trip_time (round_trip_time msg))
    (cl:cons ':range (range msg))
))
