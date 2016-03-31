;; Auto-generated. Do not edit!


(when (boundp 'uwb::UWBRange)
  (if (not (find-package "UWB"))
    (make-package "UWB"))
  (shadow 'UWBRange (find-package "UWB")))
(unless (find-package "UWB::UWBRANGE")
  (make-package "UWB::UWBRANGE"))

(in-package "ROS")
;;//! \htmlinclude UWBRange.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass uwb::UWBRange
  :super ros::object
  :slots (_header _address _remote_address _stamp_us _round_trip_time _range ))

(defmethod uwb::UWBRange
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:address __address) 0)
    ((:remote_address __remote_address) 0)
    ((:stamp_us __stamp_us) 0)
    ((:round_trip_time __round_trip_time) 0.0)
    ((:range __range) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _address (round __address))
   (setq _remote_address (round __remote_address))
   (setq _stamp_us (round __stamp_us))
   (setq _round_trip_time (float __round_trip_time))
   (setq _range (float __range))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:address
   (&optional __address)
   (if __address (setq _address __address)) _address)
  (:remote_address
   (&optional __remote_address)
   (if __remote_address (setq _remote_address __remote_address)) _remote_address)
  (:stamp_us
   (&optional __stamp_us)
   (if __stamp_us (setq _stamp_us __stamp_us)) _stamp_us)
  (:round_trip_time
   (&optional __round_trip_time)
   (if __round_trip_time (setq _round_trip_time __round_trip_time)) _round_trip_time)
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _address
    1
    ;; uint8 _remote_address
    1
    ;; uint32 _stamp_us
    4
    ;; float32 _round_trip_time
    4
    ;; float32 _range
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _address
       (write-byte _address s)
     ;; uint8 _remote_address
       (write-byte _remote_address s)
     ;; uint32 _stamp_us
       (write-long _stamp_us s)
     ;; float32 _round_trip_time
       (sys::poke _round_trip_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _range
       (sys::poke _range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _address
     (setq _address (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _remote_address
     (setq _remote_address (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _stamp_us
     (setq _stamp_us (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _round_trip_time
     (setq _round_trip_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _range
     (setq _range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get uwb::UWBRange :md5sum-) "59f7e9a9fe44ac74dcea199a0dda5d66")
(setf (get uwb::UWBRange :datatype-) "uwb/UWBRange")
(setf (get uwb::UWBRange :definition-)
      "Header header
uint8 address
uint8 remote_address
uint32 stamp_us
float32 round_trip_time
float32 range

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



(provide :uwb/UWBRange "59f7e9a9fe44ac74dcea199a0dda5d66")


