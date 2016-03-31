; Auto-generated. Do not edit!


(cl:in-package uwb-msg)


;//! \htmlinclude UWBRangeStats.msg.html

(cl:defclass <UWBRangeStats> (roslisp-msg-protocol:ros-message)
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
    :initform 0.0)
   (std_noise
    :reader std_noise
    :initarg :std_noise
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (preamble_acc_count
    :reader preamble_acc_count
    :initarg :preamble_acc_count
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (first_path_index
    :reader first_path_index
    :initarg :first_path_index
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (first_path_amp_1
    :reader first_path_amp_1
    :initarg :first_path_amp_1
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (first_path_amp_2
    :reader first_path_amp_2
    :initarg :first_path_amp_2
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (first_path_amp_3
    :reader first_path_amp_3
    :initarg :first_path_amp_3
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (channel_impulse_response_power
    :reader channel_impulse_response_power
    :initarg :channel_impulse_response_power
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (prf
    :reader prf
    :initarg :prf
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass UWBRangeStats (<UWBRangeStats>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UWBRangeStats>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UWBRangeStats)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uwb-msg:<UWBRangeStats> is deprecated: use uwb-msg:UWBRangeStats instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <UWBRangeStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:header-val is deprecated.  Use uwb-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <UWBRangeStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:address-val is deprecated.  Use uwb-msg:address instead.")
  (address m))

(cl:ensure-generic-function 'remote_address-val :lambda-list '(m))
(cl:defmethod remote_address-val ((m <UWBRangeStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:remote_address-val is deprecated.  Use uwb-msg:remote_address instead.")
  (remote_address m))

(cl:ensure-generic-function 'stamp_us-val :lambda-list '(m))
(cl:defmethod stamp_us-val ((m <UWBRangeStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:stamp_us-val is deprecated.  Use uwb-msg:stamp_us instead.")
  (stamp_us m))

(cl:ensure-generic-function 'round_trip_time-val :lambda-list '(m))
(cl:defmethod round_trip_time-val ((m <UWBRangeStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:round_trip_time-val is deprecated.  Use uwb-msg:round_trip_time instead.")
  (round_trip_time m))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <UWBRangeStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:range-val is deprecated.  Use uwb-msg:range instead.")
  (range m))

(cl:ensure-generic-function 'std_noise-val :lambda-list '(m))
(cl:defmethod std_noise-val ((m <UWBRangeStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:std_noise-val is deprecated.  Use uwb-msg:std_noise instead.")
  (std_noise m))

(cl:ensure-generic-function 'preamble_acc_count-val :lambda-list '(m))
(cl:defmethod preamble_acc_count-val ((m <UWBRangeStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:preamble_acc_count-val is deprecated.  Use uwb-msg:preamble_acc_count instead.")
  (preamble_acc_count m))

(cl:ensure-generic-function 'first_path_index-val :lambda-list '(m))
(cl:defmethod first_path_index-val ((m <UWBRangeStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:first_path_index-val is deprecated.  Use uwb-msg:first_path_index instead.")
  (first_path_index m))

(cl:ensure-generic-function 'first_path_amp_1-val :lambda-list '(m))
(cl:defmethod first_path_amp_1-val ((m <UWBRangeStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:first_path_amp_1-val is deprecated.  Use uwb-msg:first_path_amp_1 instead.")
  (first_path_amp_1 m))

(cl:ensure-generic-function 'first_path_amp_2-val :lambda-list '(m))
(cl:defmethod first_path_amp_2-val ((m <UWBRangeStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:first_path_amp_2-val is deprecated.  Use uwb-msg:first_path_amp_2 instead.")
  (first_path_amp_2 m))

(cl:ensure-generic-function 'first_path_amp_3-val :lambda-list '(m))
(cl:defmethod first_path_amp_3-val ((m <UWBRangeStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:first_path_amp_3-val is deprecated.  Use uwb-msg:first_path_amp_3 instead.")
  (first_path_amp_3 m))

(cl:ensure-generic-function 'channel_impulse_response_power-val :lambda-list '(m))
(cl:defmethod channel_impulse_response_power-val ((m <UWBRangeStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:channel_impulse_response_power-val is deprecated.  Use uwb-msg:channel_impulse_response_power instead.")
  (channel_impulse_response_power m))

(cl:ensure-generic-function 'prf-val :lambda-list '(m))
(cl:defmethod prf-val ((m <UWBRangeStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uwb-msg:prf-val is deprecated.  Use uwb-msg:prf instead.")
  (prf m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UWBRangeStats>) ostream)
  "Serializes a message object of type '<UWBRangeStats>"
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
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'std_noise))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'preamble_acc_count))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'first_path_index))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'first_path_amp_1))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'first_path_amp_2))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'first_path_amp_3))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'channel_impulse_response_power))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'prf))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UWBRangeStats>) istream)
  "Deserializes a message object of type '<UWBRangeStats>"
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
  (cl:setf (cl:slot-value msg 'std_noise) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'std_noise)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'preamble_acc_count) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'preamble_acc_count)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'first_path_index) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'first_path_index)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'first_path_amp_1) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'first_path_amp_1)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'first_path_amp_2) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'first_path_amp_2)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'first_path_amp_3) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'first_path_amp_3)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'channel_impulse_response_power) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'channel_impulse_response_power)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'prf) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'prf)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UWBRangeStats>)))
  "Returns string type for a message object of type '<UWBRangeStats>"
  "uwb/UWBRangeStats")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UWBRangeStats)))
  "Returns string type for a message object of type 'UWBRangeStats"
  "uwb/UWBRangeStats")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UWBRangeStats>)))
  "Returns md5sum for a message object of type '<UWBRangeStats>"
  "b6ca04d144ef745d3c9c8be35d4fa277")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UWBRangeStats)))
  "Returns md5sum for a message object of type 'UWBRangeStats"
  "b6ca04d144ef745d3c9c8be35d4fa277")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UWBRangeStats>)))
  "Returns full string definition for message of type '<UWBRangeStats>"
  (cl:format cl:nil "Header header~%uint8 address~%uint8 remote_address~%uint32 stamp_us~%float32 round_trip_time~%float32 range~%uint16[3] std_noise~%uint16[3] preamble_acc_count~%uint16[3] first_path_index~%uint16[3] first_path_amp_1~%uint16[3] first_path_amp_2~%uint16[3] first_path_amp_3~%uint16[3] channel_impulse_response_power~%uint16[3] prf~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UWBRangeStats)))
  "Returns full string definition for message of type 'UWBRangeStats"
  (cl:format cl:nil "Header header~%uint8 address~%uint8 remote_address~%uint32 stamp_us~%float32 round_trip_time~%float32 range~%uint16[3] std_noise~%uint16[3] preamble_acc_count~%uint16[3] first_path_index~%uint16[3] first_path_amp_1~%uint16[3] first_path_amp_2~%uint16[3] first_path_amp_3~%uint16[3] channel_impulse_response_power~%uint16[3] prf~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UWBRangeStats>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     4
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'std_noise) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'preamble_acc_count) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'first_path_index) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'first_path_amp_1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'first_path_amp_2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'first_path_amp_3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'channel_impulse_response_power) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'prf) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UWBRangeStats>))
  "Converts a ROS message object to a list"
  (cl:list 'UWBRangeStats
    (cl:cons ':header (header msg))
    (cl:cons ':address (address msg))
    (cl:cons ':remote_address (remote_address msg))
    (cl:cons ':stamp_us (stamp_us msg))
    (cl:cons ':round_trip_time (round_trip_time msg))
    (cl:cons ':range (range msg))
    (cl:cons ':std_noise (std_noise msg))
    (cl:cons ':preamble_acc_count (preamble_acc_count msg))
    (cl:cons ':first_path_index (first_path_index msg))
    (cl:cons ':first_path_amp_1 (first_path_amp_1 msg))
    (cl:cons ':first_path_amp_2 (first_path_amp_2 msg))
    (cl:cons ':first_path_amp_3 (first_path_amp_3 msg))
    (cl:cons ':channel_impulse_response_power (channel_impulse_response_power msg))
    (cl:cons ':prf (prf msg))
))
