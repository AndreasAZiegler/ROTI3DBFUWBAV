;; Auto-generated. Do not edit!


(when (boundp 'uwb::UWBMultiRangeRaw)
  (if (not (find-package "UWB"))
    (make-package "UWB"))
  (shadow 'UWBMultiRangeRaw (find-package "UWB")))
(unless (find-package "UWB::UWBMULTIRANGERAW")
  (make-package "UWB::UWBMULTIRANGERAW"))

(in-package "ROS")
;;//! \htmlinclude UWBMultiRangeRaw.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass uwb::UWBMultiRangeRaw
  :super ros::object
  :slots (_header _num_of_units _address _remote_address _timestamp_master_request_1_recv _timestamp_slave_reply_send _timestamp_master_request_2_recv _timestamp_master_request_1 _timestamp_slave_reply _timestamp_master_request_2 ))

(defmethod uwb::UWBMultiRangeRaw
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:num_of_units __num_of_units) 0)
    ((:address __address) 0)
    ((:remote_address __remote_address) 0)
    ((:timestamp_master_request_1_recv __timestamp_master_request_1_recv) 0)
    ((:timestamp_slave_reply_send __timestamp_slave_reply_send) 0)
    ((:timestamp_master_request_2_recv __timestamp_master_request_2_recv) 0)
    ((:timestamp_master_request_1 __timestamp_master_request_1) (make-array 0 :initial-element 0 :element-type :integer))
    ((:timestamp_slave_reply __timestamp_slave_reply) (make-array 0 :initial-element 0 :element-type :integer))
    ((:timestamp_master_request_2 __timestamp_master_request_2) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _num_of_units (round __num_of_units))
   (setq _address (round __address))
   (setq _remote_address (round __remote_address))
   (setq _timestamp_master_request_1_recv (round __timestamp_master_request_1_recv))
   (setq _timestamp_slave_reply_send (round __timestamp_slave_reply_send))
   (setq _timestamp_master_request_2_recv (round __timestamp_master_request_2_recv))
   (setq _timestamp_master_request_1 __timestamp_master_request_1)
   (setq _timestamp_slave_reply __timestamp_slave_reply)
   (setq _timestamp_master_request_2 __timestamp_master_request_2)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:num_of_units
   (&optional __num_of_units)
   (if __num_of_units (setq _num_of_units __num_of_units)) _num_of_units)
  (:address
   (&optional __address)
   (if __address (setq _address __address)) _address)
  (:remote_address
   (&optional __remote_address)
   (if __remote_address (setq _remote_address __remote_address)) _remote_address)
  (:timestamp_master_request_1_recv
   (&optional __timestamp_master_request_1_recv)
   (if __timestamp_master_request_1_recv (setq _timestamp_master_request_1_recv __timestamp_master_request_1_recv)) _timestamp_master_request_1_recv)
  (:timestamp_slave_reply_send
   (&optional __timestamp_slave_reply_send)
   (if __timestamp_slave_reply_send (setq _timestamp_slave_reply_send __timestamp_slave_reply_send)) _timestamp_slave_reply_send)
  (:timestamp_master_request_2_recv
   (&optional __timestamp_master_request_2_recv)
   (if __timestamp_master_request_2_recv (setq _timestamp_master_request_2_recv __timestamp_master_request_2_recv)) _timestamp_master_request_2_recv)
  (:timestamp_master_request_1
   (&optional __timestamp_master_request_1)
   (if __timestamp_master_request_1 (setq _timestamp_master_request_1 __timestamp_master_request_1)) _timestamp_master_request_1)
  (:timestamp_slave_reply
   (&optional __timestamp_slave_reply)
   (if __timestamp_slave_reply (setq _timestamp_slave_reply __timestamp_slave_reply)) _timestamp_slave_reply)
  (:timestamp_master_request_2
   (&optional __timestamp_master_request_2)
   (if __timestamp_master_request_2 (setq _timestamp_master_request_2 __timestamp_master_request_2)) _timestamp_master_request_2)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _num_of_units
    1
    ;; uint8 _address
    1
    ;; uint8 _remote_address
    1
    ;; int64 _timestamp_master_request_1_recv
    8
    ;; int64 _timestamp_slave_reply_send
    8
    ;; int64 _timestamp_master_request_2_recv
    8
    ;; int64[] _timestamp_master_request_1
    (* 8    (length _timestamp_master_request_1)) 4
    ;; int64[] _timestamp_slave_reply
    (* 8    (length _timestamp_slave_reply)) 4
    ;; int64[] _timestamp_master_request_2
    (* 8    (length _timestamp_master_request_2)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _num_of_units
       (write-byte _num_of_units s)
     ;; uint8 _address
       (write-byte _address s)
     ;; uint8 _remote_address
       (write-byte _remote_address s)
     ;; int64 _timestamp_master_request_1_recv
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _timestamp_master_request_1_recv (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _timestamp_master_request_1_recv) (= (length (_timestamp_master_request_1_recv . bv)) 2)) ;; bignum
              (write-long (ash (elt (_timestamp_master_request_1_recv . bv) 0) 0) s)
              (write-long (ash (elt (_timestamp_master_request_1_recv . bv) 1) -1) s))
             ((and (class _timestamp_master_request_1_recv) (= (length (_timestamp_master_request_1_recv . bv)) 1)) ;; big1
              (write-long (elt (_timestamp_master_request_1_recv . bv) 0) s)
              (write-long (if (>= _timestamp_master_request_1_recv 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _timestamp_master_request_1_recv s)(write-long (if (>= _timestamp_master_request_1_recv 0) 0 #xffffffff) s)))
     ;; int64 _timestamp_slave_reply_send
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _timestamp_slave_reply_send (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _timestamp_slave_reply_send) (= (length (_timestamp_slave_reply_send . bv)) 2)) ;; bignum
              (write-long (ash (elt (_timestamp_slave_reply_send . bv) 0) 0) s)
              (write-long (ash (elt (_timestamp_slave_reply_send . bv) 1) -1) s))
             ((and (class _timestamp_slave_reply_send) (= (length (_timestamp_slave_reply_send . bv)) 1)) ;; big1
              (write-long (elt (_timestamp_slave_reply_send . bv) 0) s)
              (write-long (if (>= _timestamp_slave_reply_send 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _timestamp_slave_reply_send s)(write-long (if (>= _timestamp_slave_reply_send 0) 0 #xffffffff) s)))
     ;; int64 _timestamp_master_request_2_recv
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _timestamp_master_request_2_recv (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _timestamp_master_request_2_recv) (= (length (_timestamp_master_request_2_recv . bv)) 2)) ;; bignum
              (write-long (ash (elt (_timestamp_master_request_2_recv . bv) 0) 0) s)
              (write-long (ash (elt (_timestamp_master_request_2_recv . bv) 1) -1) s))
             ((and (class _timestamp_master_request_2_recv) (= (length (_timestamp_master_request_2_recv . bv)) 1)) ;; big1
              (write-long (elt (_timestamp_master_request_2_recv . bv) 0) s)
              (write-long (if (>= _timestamp_master_request_2_recv 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _timestamp_master_request_2_recv s)(write-long (if (>= _timestamp_master_request_2_recv 0) 0 #xffffffff) s)))
     ;; int64[] _timestamp_master_request_1
     (write-long (length _timestamp_master_request_1) s)
     (dotimes (i (length _timestamp_master_request_1))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _timestamp_master_request_1 i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _timestamp_master_request_1 i)) (= (length ((elt _timestamp_master_request_1 i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _timestamp_master_request_1 i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _timestamp_master_request_1 i) . bv) 1) -1) s))
             ((and (class (elt _timestamp_master_request_1 i)) (= (length ((elt _timestamp_master_request_1 i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _timestamp_master_request_1 i) . bv) 0) s)
              (write-long (if (>= (elt _timestamp_master_request_1 i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _timestamp_master_request_1 i) s)(write-long (if (>= (elt _timestamp_master_request_1 i) 0) 0 #xffffffff) s)))
       )
     ;; int64[] _timestamp_slave_reply
     (write-long (length _timestamp_slave_reply) s)
     (dotimes (i (length _timestamp_slave_reply))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _timestamp_slave_reply i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _timestamp_slave_reply i)) (= (length ((elt _timestamp_slave_reply i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _timestamp_slave_reply i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _timestamp_slave_reply i) . bv) 1) -1) s))
             ((and (class (elt _timestamp_slave_reply i)) (= (length ((elt _timestamp_slave_reply i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _timestamp_slave_reply i) . bv) 0) s)
              (write-long (if (>= (elt _timestamp_slave_reply i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _timestamp_slave_reply i) s)(write-long (if (>= (elt _timestamp_slave_reply i) 0) 0 #xffffffff) s)))
       )
     ;; int64[] _timestamp_master_request_2
     (write-long (length _timestamp_master_request_2) s)
     (dotimes (i (length _timestamp_master_request_2))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _timestamp_master_request_2 i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _timestamp_master_request_2 i)) (= (length ((elt _timestamp_master_request_2 i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _timestamp_master_request_2 i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _timestamp_master_request_2 i) . bv) 1) -1) s))
             ((and (class (elt _timestamp_master_request_2 i)) (= (length ((elt _timestamp_master_request_2 i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _timestamp_master_request_2 i) . bv) 0) s)
              (write-long (if (>= (elt _timestamp_master_request_2 i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _timestamp_master_request_2 i) s)(write-long (if (>= (elt _timestamp_master_request_2 i) 0) 0 #xffffffff) s)))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _num_of_units
     (setq _num_of_units (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _address
     (setq _address (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _remote_address
     (setq _remote_address (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int64 _timestamp_master_request_1_recv
#+(or :alpha :irix6 :x86_64)
      (setf _timestamp_master_request_1_recv (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _timestamp_master_request_1_recv (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _timestamp_slave_reply_send
#+(or :alpha :irix6 :x86_64)
      (setf _timestamp_slave_reply_send (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _timestamp_slave_reply_send (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _timestamp_master_request_2_recv
#+(or :alpha :irix6 :x86_64)
      (setf _timestamp_master_request_2_recv (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _timestamp_master_request_2_recv (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64[] _timestamp_master_request_1
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _timestamp_master_request_1 (instantiate integer-vector n))
     (dotimes (i n)
#+(or :alpha :irix6 :x86_64)
      (setf (elt _timestamp_master_request_1 i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _timestamp_master_request_1 i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;; int64[] _timestamp_slave_reply
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _timestamp_slave_reply (instantiate integer-vector n))
     (dotimes (i n)
#+(or :alpha :irix6 :x86_64)
      (setf (elt _timestamp_slave_reply i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _timestamp_slave_reply i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;; int64[] _timestamp_master_request_2
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _timestamp_master_request_2 (instantiate integer-vector n))
     (dotimes (i n)
#+(or :alpha :irix6 :x86_64)
      (setf (elt _timestamp_master_request_2 i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _timestamp_master_request_2 i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;;
   self)
  )

(setf (get uwb::UWBMultiRangeRaw :md5sum-) "86220690a868e2b487d423fe45af1477")
(setf (get uwb::UWBMultiRangeRaw :datatype-) "uwb/UWBMultiRangeRaw")
(setf (get uwb::UWBMultiRangeRaw :definition-)
      "Header header
uint8 num_of_units
uint8 address
uint8 remote_address
int64 timestamp_master_request_1_recv
int64 timestamp_slave_reply_send
int64 timestamp_master_request_2_recv
int64[] timestamp_master_request_1
int64[] timestamp_slave_reply
int64[] timestamp_master_request_2

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :uwb/UWBMultiRangeRaw "86220690a868e2b487d423fe45af1477")


