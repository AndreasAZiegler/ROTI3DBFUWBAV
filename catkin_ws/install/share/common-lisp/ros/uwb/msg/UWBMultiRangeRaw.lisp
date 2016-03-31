; Auto-generated. Do not edit!


(cl:in-package uwb-msg)


;//! \htmlinclude UWBMultiRangeRaw.msg.html

(cl:defclass <UWBMultiRangeRaw> (roslisp-msg-protocol:ros-message)
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
   (timestamp_master_request_1_recv
    :reader timestamp_master_request_1_recv
    :initarg :timestamp_master_request_1_recv
    :type cl:integer
    :initform 0)
   (timestamp_slave_reply_send
    :reader timestamp_slave_reply_send
    :initarg :timestamp_slave_reply_send
    :type cl:integer
    :initform 0)
   (timestamp_master_request_2_recv
    :reader timestamp_master_request_2_recv
    :initarg :timestamp_master_request_2_recv
    :type cl:integer
    :initform 0)
   (timestamp_master_request_1
    :reader timestamp_master_request_1
    :initarg :timestamp_master_request_1
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (timestamp_slave_reply
    :reader timestamp_slave_reply
    :initarg :timestamp_slave_reply
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (timestamp_master_request_2
    :reader timestamp_master_request_2
    :initarg :timestamp_master_request_2
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass UWBMultiRangeRaw (<UWBMultiRangeRaw>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UWBMultiRangeRaw>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UWBMultiRangeRaw)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uwb-msg:<UWBMultiRangeRaw> is deprecated: use uwb-msg:UWBMultiRangeRaw instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <UWBMultiRangeRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:header-val is deprecated.  Use uwb-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'num_of_units-val :lambda-list '(m))
(cl:defmethod num_of_units-val ((m <UWBMultiRangeRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:num_of_units-val is deprecated.  Use uwb-msg:num_of_units instead.")
  (num_of_units m))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <UWBMultiRangeRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:address-val is deprecated.  Use uwb-msg:address instead.")
  (address m))

(cl:ensure-generic-function 'remote_address-val :lambda-list '(m))
(cl:defmethod remote_address-val ((m <UWBMultiRangeRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:remote_address-val is deprecated.  Use uwb-msg:remote_address instead.")
  (remote_address m))

(cl:ensure-generic-function 'timestamp_master_request_1_recv-val :lambda-list '(m))
(cl:defmethod timestamp_master_request_1_recv-val ((m <UWBMultiRangeRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:timestamp_master_request_1_recv-val is deprecated.  Use uwb-msg:timestamp_master_request_1_recv instead.")
  (timestamp_master_request_1_recv m))

(cl:ensure-generic-function 'timestamp_slave_reply_send-val :lambda-list '(m))
(cl:defmethod timestamp_slave_reply_send-val ((m <UWBMultiRangeRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:timestamp_slave_reply_send-val is deprecated.  Use uwb-msg:timestamp_slave_reply_send instead.")
  (timestamp_slave_reply_send m))

(cl:ensure-generic-function 'timestamp_master_request_2_recv-val :lambda-list '(m))
(cl:defmethod timestamp_master_request_2_recv-val ((m <UWBMultiRangeRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:timestamp_master_request_2_recv-val is deprecated.  Use uwb-msg:timestamp_master_request_2_recv instead.")
  (timestamp_master_request_2_recv m))

(cl:ensure-generic-function 'timestamp_master_request_1-val :lambda-list '(m))
(cl:defmethod timestamp_master_request_1-val ((m <UWBMultiRangeRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:timestamp_master_request_1-val is deprecated.  Use uwb-msg:timestamp_master_request_1 instead.")
  (timestamp_master_request_1 m))

(cl:ensure-generic-function 'timestamp_slave_reply-val :lambda-list '(m))
(cl:defmethod timestamp_slave_reply-val ((m <UWBMultiRangeRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:timestamp_slave_reply-val is deprecated.  Use uwb-msg:timestamp_slave_reply instead.")
  (timestamp_slave_reply m))

(cl:ensure-generic-function 'timestamp_master_request_2-val :lambda-list '(m))
(cl:defmethod timestamp_master_request_2-val ((m <UWBMultiRangeRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:timestamp_master_request_2-val is deprecated.  Use uwb-msg:timestamp_master_request_2 instead.")
  (timestamp_master_request_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UWBMultiRangeRaw>) ostream)
  "Serializes a message object of type '<UWBMultiRangeRaw>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_of_units)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'remote_address)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'timestamp_master_request_1_recv)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'timestamp_slave_reply_send)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'timestamp_master_request_2_recv)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'timestamp_master_request_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    ))
   (cl:slot-value msg 'timestamp_master_request_1))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'timestamp_slave_reply))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    ))
   (cl:slot-value msg 'timestamp_slave_reply))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'timestamp_master_request_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    ))
   (cl:slot-value msg 'timestamp_master_request_2))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UWBMultiRangeRaw>) istream)
  "Deserializes a message object of type '<UWBMultiRangeRaw>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_of_units)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'remote_address)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestamp_master_request_1_recv) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestamp_slave_reply_send) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestamp_master_request_2_recv) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'timestamp_master_request_1) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'timestamp_master_request_1)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'timestamp_slave_reply) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'timestamp_slave_reply)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'timestamp_master_request_2) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'timestamp_master_request_2)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UWBMultiRangeRaw>)))
  "Returns string type for a message object of type '<UWBMultiRangeRaw>"
  "uwb/UWBMultiRangeRaw")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UWBMultiRangeRaw)))
  "Returns string type for a message object of type 'UWBMultiRangeRaw"
  "uwb/UWBMultiRangeRaw")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UWBMultiRangeRaw>)))
  "Returns md5sum for a message object of type '<UWBMultiRangeRaw>"
  "86220690a868e2b487d423fe45af1477")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UWBMultiRangeRaw)))
  "Returns md5sum for a message object of type 'UWBMultiRangeRaw"
  "86220690a868e2b487d423fe45af1477")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UWBMultiRangeRaw>)))
  "Returns full string definition for message of type '<UWBMultiRangeRaw>"
  (cl:format cl:nil "Header header~%uint8 num_of_units~%uint8 address~%uint8 remote_address~%int64 timestamp_master_request_1_recv~%int64 timestamp_slave_reply_send~%int64 timestamp_master_request_2_recv~%int64[] timestamp_master_request_1~%int64[] timestamp_slave_reply~%int64[] timestamp_master_request_2~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UWBMultiRangeRaw)))
  "Returns full string definition for message of type 'UWBMultiRangeRaw"
  (cl:format cl:nil "Header header~%uint8 num_of_units~%uint8 address~%uint8 remote_address~%int64 timestamp_master_request_1_recv~%int64 timestamp_slave_reply_send~%int64 timestamp_master_request_2_recv~%int64[] timestamp_master_request_1~%int64[] timestamp_slave_reply~%int64[] timestamp_master_request_2~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UWBMultiRangeRaw>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     8
     8
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'timestamp_master_request_1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'timestamp_slave_reply) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'timestamp_master_request_2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UWBMultiRangeRaw>))
  "Converts a ROS message object to a list"
  (cl:list 'UWBMultiRangeRaw
    (cl:cons ':header (header msg))
    (cl:cons ':num_of_units (num_of_units msg))
    (cl:cons ':address (address msg))
    (cl:cons ':remote_address (remote_address msg))
    (cl:cons ':timestamp_master_request_1_recv (timestamp_master_request_1_recv msg))
    (cl:cons ':timestamp_slave_reply_send (timestamp_slave_reply_send msg))
    (cl:cons ':timestamp_master_request_2_recv (timestamp_master_request_2_recv msg))
    (cl:cons ':timestamp_master_request_1 (timestamp_master_request_1 msg))
    (cl:cons ':timestamp_slave_reply (timestamp_slave_reply msg))
    (cl:cons ':timestamp_master_request_2 (timestamp_master_request_2 msg))
))
