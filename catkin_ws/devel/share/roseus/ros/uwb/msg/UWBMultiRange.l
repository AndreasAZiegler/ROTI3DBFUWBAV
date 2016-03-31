;; Auto-generated. Do not edit!


(when (boundp 'uwb::UWBMultiRange)
  (if (not (find-package "UWB"))
    (make-package "UWB"))
  (shadow 'UWBMultiRange (find-package "UWB")))
(unless (find-package "UWB::UWBMULTIRANGE")
  (make-package "UWB::UWBMULTIRANGE"))

(in-package "ROS")
;;//! \htmlinclude UWBMultiRange.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass uwb::UWBMultiRange
  :super ros::object
  :slots (_header _num_of_units _address _remote_address _tofs _ranges ))

(defmethod uwb::UWBMultiRange
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:num_of_units __num_of_units) 0)
    ((:address __address) 0)
    ((:remote_address __remote_address) 0)
    ((:tofs __tofs) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:ranges __ranges) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _num_of_units (round __num_of_units))
   (setq _address (round __address))
   (setq _remote_address (round __remote_address))
   (setq _tofs __tofs)
   (setq _ranges __ranges)
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
  (:tofs
   (&optional __tofs)
   (if __tofs (setq _tofs __tofs)) _tofs)
  (:ranges
   (&optional __ranges)
   (if __ranges (setq _ranges __ranges)) _ranges)
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
    ;; float32[] _tofs
    (* 4    (length _tofs)) 4
    ;; float32[] _ranges
    (* 4    (length _ranges)) 4
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
     ;; float32[] _tofs
     (write-long (length _tofs) s)
     (dotimes (i (length _tofs))
       (sys::poke (elt _tofs i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _ranges
     (write-long (length _ranges) s)
     (dotimes (i (length _ranges))
       (sys::poke (elt _ranges i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32[] _tofs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tofs (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _tofs i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _ranges
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ranges (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _ranges i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get uwb::UWBMultiRange :md5sum-) "1d5b8085d2d9728e3c5038f539f7e4f5")
(setf (get uwb::UWBMultiRange :datatype-) "uwb/UWBMultiRange")
(setf (get uwb::UWBMultiRange :definition-)
      "Header header
uint8 num_of_units
uint8 address
uint8 remote_address
float32[] tofs
float32[] ranges

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



(provide :uwb/UWBMultiRange "1d5b8085d2d9728e3c5038f539f7e4f5")


