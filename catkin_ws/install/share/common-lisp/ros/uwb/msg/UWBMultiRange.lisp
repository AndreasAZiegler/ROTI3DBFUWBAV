; Auto-generated. Do not edit!


(cl:in-package uwb-msg)


;//! \htmlinclude UWBMultiRange.msg.html

(cl:defclass <UWBMultiRange> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (num_of_units
    :reader num_of_units
    :initarg :num_of_units
    :type cl:fixnum
    :initform 0)
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
   (tofs
    :reader tofs
    :initarg :tofs
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (ranges
    :reader ranges
    :initarg :ranges
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass UWBMultiRange (<UWBMultiRange>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UWBMultiRange>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UWBMultiRange)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uwb-msg:<UWBMultiRange> is deprecated: use uwb-msg:UWBMultiRange instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <UWBMultiRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:header-val is deprecated.  Use uwb-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'num_of_units-val :lambda-list '(m))
(cl:defmethod num_of_units-val ((m <UWBMultiRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:num_of_units-val is deprecated.  Use uwb-msg:num_of_units instead.")
  (num_of_units m))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <UWBMultiRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:address-val is deprecated.  Use uwb-msg:address instead.")
  (address m))

(cl:ensure-generic-function 'remote_address-val :lambda-list '(m))
(cl:defmethod remote_address-val ((m <UWBMultiRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:remote_address-val is deprecated.  Use uwb-msg:remote_address instead.")
  (remote_address m))

(cl:ensure-generic-function 'tofs-val :lambda-list '(m))
(cl:defmethod tofs-val ((m <UWBMultiRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:tofs-val is deprecated.  Use uwb-msg:tofs instead.")
  (tofs m))

(cl:ensure-generic-function 'ranges-val :lambda-list '(m))
(cl:defmethod ranges-val ((m <UWBMultiRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:ranges-val is deprecated.  Use uwb-msg:ranges instead.")
  (ranges m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UWBMultiRange>) ostream)
  "Serializes a message object of type '<UWBMultiRange>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_of_units)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'remote_address)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tofs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'tofs))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ranges))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'ranges))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UWBMultiRange>) istream)
  "Deserializes a message object of type '<UWBMultiRange>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_of_units)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'remote_address)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tofs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tofs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ranges) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ranges)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UWBMultiRange>)))
  "Returns string type for a message object of type '<UWBMultiRange>"
  "uwb/UWBMultiRange")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UWBMultiRange)))
  "Returns string type for a message object of type 'UWBMultiRange"
  "uwb/UWBMultiRange")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UWBMultiRange>)))
  "Returns md5sum for a message object of type '<UWBMultiRange>"
  "1d5b8085d2d9728e3c5038f539f7e4f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UWBMultiRange)))
  "Returns md5sum for a message object of type 'UWBMultiRange"
  "1d5b8085d2d9728e3c5038f539f7e4f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UWBMultiRange>)))
  "Returns full string definition for message of type '<UWBMultiRange>"
  (cl:format cl:nil "Header header~%uint8 num_of_units~%uint8 address~%uint8 remote_address~%float32[] tofs~%float32[] ranges~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UWBMultiRange)))
  "Returns full string definition for message of type 'UWBMultiRange"
  (cl:format cl:nil "Header header~%uint8 num_of_units~%uint8 address~%uint8 remote_address~%float32[] tofs~%float32[] ranges~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UWBMultiRange>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tofs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ranges) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UWBMultiRange>))
  "Converts a ROS message object to a list"
  (cl:list 'UWBMultiRange
    (cl:cons ':header (header msg))
    (cl:cons ':num_of_units (num_of_units msg))
    (cl:cons ':address (address msg))
    (cl:cons ':remote_address (remote_address msg))
    (cl:cons ':tofs (tofs msg))
    (cl:cons ':ranges (ranges msg))
))
