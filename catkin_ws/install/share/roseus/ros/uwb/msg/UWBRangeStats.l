;; Auto-generated. Do not edit!


(when (boundp 'uwb::UWBRangeStats)
  (if (not (find-package "UWB"))
    (make-package "UWB"))
  (shadow 'UWBRangeStats (find-package "UWB")))
(unless (find-package "UWB::UWBRANGESTATS")
  (make-package "UWB::UWBRANGESTATS"))

(in-package "ROS")
;;//! \htmlinclude UWBRangeStats.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass uwb::UWBRangeStats
  :super ros::object
  :slots (_header _address _remote_address _stamp_us _round_trip_time _range _std_noise _preamble_acc_count _first_path_index _first_path_amp_1 _first_path_amp_2 _first_path_amp_3 _channel_impulse_response_power _prf ))

(defmethod uwb::UWBRangeStats
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:address __address) 0)
    ((:remote_address __remote_address) 0)
    ((:stamp_us __stamp_us) 0)
    ((:round_trip_time __round_trip_time) 0.0)
    ((:range __range) 0.0)
    ((:std_noise __std_noise) (make-array 3 :initial-element 0 :element-type :integer))
    ((:preamble_acc_count __preamble_acc_count) (make-array 3 :initial-element 0 :element-type :integer))
    ((:first_path_index __first_path_index) (make-array 3 :initial-element 0 :element-type :integer))
    ((:first_path_amp_1 __first_path_amp_1) (make-array 3 :initial-element 0 :element-type :integer))
    ((:first_path_amp_2 __first_path_amp_2) (make-array 3 :initial-element 0 :element-type :integer))
    ((:first_path_amp_3 __first_path_amp_3) (make-array 3 :initial-element 0 :element-type :integer))
    ((:channel_impulse_response_power __channel_impulse_response_power) (make-array 3 :initial-element 0 :element-type :integer))
    ((:prf __prf) (make-array 3 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _address (round __address))
   (setq _remote_address (round __remote_address))
   (setq _stamp_us (round __stamp_us))
   (setq _round_trip_time (float __round_trip_time))
   (setq _range (float __range))
   (setq _std_noise __std_noise)
   (setq _preamble_acc_count __preamble_acc_count)
   (setq _first_path_index __first_path_index)
   (setq _first_path_amp_1 __first_path_amp_1)
   (setq _first_path_amp_2 __first_path_amp_2)
   (setq _first_path_amp_3 __first_path_amp_3)
   (setq _channel_impulse_response_power __channel_impulse_response_power)
   (setq _prf __prf)
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
  (:std_noise
   (&optional __std_noise)
   (if __std_noise (setq _std_noise __std_noise)) _std_noise)
  (:preamble_acc_count
   (&optional __preamble_acc_count)
   (if __preamble_acc_count (setq _preamble_acc_count __preamble_acc_count)) _preamble_acc_count)
  (:first_path_index
   (&optional __first_path_index)
   (if __first_path_index (setq _first_path_index __first_path_index)) _first_path_index)
  (:first_path_amp_1
   (&optional __first_path_amp_1)
   (if __first_path_amp_1 (setq _first_path_amp_1 __first_path_amp_1)) _first_path_amp_1)
  (:first_path_amp_2
   (&optional __first_path_amp_2)
   (if __first_path_amp_2 (setq _first_path_amp_2 __first_path_amp_2)) _first_path_amp_2)
  (:first_path_amp_3
   (&optional __first_path_amp_3)
   (if __first_path_amp_3 (setq _first_path_amp_3 __first_path_amp_3)) _first_path_amp_3)
  (:channel_impulse_response_power
   (&optional __channel_impulse_response_power)
   (if __channel_impulse_response_power (setq _channel_impulse_response_power __channel_impulse_response_power)) _channel_impulse_response_power)
  (:prf
   (&optional __prf)
   (if __prf (setq _prf __prf)) _prf)
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
    ;; uint16[3] _std_noise
    (* 2    3)
    ;; uint16[3] _preamble_acc_count
    (* 2    3)
    ;; uint16[3] _first_path_index
    (* 2    3)
    ;; uint16[3] _first_path_amp_1
    (* 2    3)
    ;; uint16[3] _first_path_amp_2
    (* 2    3)
    ;; uint16[3] _first_path_amp_3
    (* 2    3)
    ;; uint16[3] _channel_impulse_response_power
    (* 2    3)
    ;; uint16[3] _prf
    (* 2    3)
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
     ;; uint16[3] _std_noise
     (dotimes (i 3)
       (write-word (elt _std_noise i) s)
       )
     ;; uint16[3] _preamble_acc_count
     (dotimes (i 3)
       (write-word (elt _preamble_acc_count i) s)
       )
     ;; uint16[3] _first_path_index
     (dotimes (i 3)
       (write-word (elt _first_path_index i) s)
       )
     ;; uint16[3] _first_path_amp_1
     (dotimes (i 3)
       (write-word (elt _first_path_amp_1 i) s)
       )
     ;; uint16[3] _first_path_amp_2
     (dotimes (i 3)
       (write-word (elt _first_path_amp_2 i) s)
       )
     ;; uint16[3] _first_path_amp_3
     (dotimes (i 3)
       (write-word (elt _first_path_amp_3 i) s)
       )
     ;; uint16[3] _channel_impulse_response_power
     (dotimes (i 3)
       (write-word (elt _channel_impulse_response_power i) s)
       )
     ;; uint16[3] _prf
     (dotimes (i 3)
       (write-word (elt _prf i) s)
       )
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
   ;; uint16[3] _std_noise
   (dotimes (i (length _std_noise))
     (setf (elt _std_noise i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _preamble_acc_count
   (dotimes (i (length _preamble_acc_count))
     (setf (elt _preamble_acc_count i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _first_path_index
   (dotimes (i (length _first_path_index))
     (setf (elt _first_path_index i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _first_path_amp_1
   (dotimes (i (length _first_path_amp_1))
     (setf (elt _first_path_amp_1 i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _first_path_amp_2
   (dotimes (i (length _first_path_amp_2))
     (setf (elt _first_path_amp_2 i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _first_path_amp_3
   (dotimes (i (length _first_path_amp_3))
     (setf (elt _first_path_amp_3 i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _channel_impulse_response_power
   (dotimes (i (length _channel_impulse_response_power))
     (setf (elt _channel_impulse_response_power i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint16[3] _prf
   (dotimes (i (length _prf))
     (setf (elt _prf i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get uwb::UWBRangeStats :md5sum-) "b6ca04d144ef745d3c9c8be35d4fa277")
(setf (get uwb::UWBRangeStats :datatype-) "uwb/UWBRangeStats")
(setf (get uwb::UWBRangeStats :definition-)
      "Header header
uint8 address
uint8 remote_address
uint32 stamp_us
float32 round_trip_time
float32 range
uint16[3] std_noise
uint16[3] preamble_acc_count
uint16[3] first_path_index
uint16[3] first_path_amp_1
uint16[3] first_path_amp_2
uint16[3] first_path_amp_3
uint16[3] channel_impulse_response_power
uint16[3] prf

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



(provide :uwb/UWBRangeStats "b6ca04d144ef745d3c9c8be35d4fa277")


