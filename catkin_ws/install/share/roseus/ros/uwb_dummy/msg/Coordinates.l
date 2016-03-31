;; Auto-generated. Do not edit!


(when (boundp 'uwb_dummy::Coordinates)
  (if (not (find-package "UWB_DUMMY"))
    (make-package "UWB_DUMMY"))
  (shadow 'Coordinates (find-package "UWB_DUMMY")))
(unless (find-package "UWB_DUMMY::COORDINATES")
  (make-package "UWB_DUMMY::COORDINATES"))

(in-package "ROS")
;;//! \htmlinclude Coordinates.msg.html


(defclass uwb_dummy::Coordinates
  :super ros::object
  :slots (_coordinates ))

(defmethod uwb_dummy::Coordinates
  (:init
   (&key
    ((:coordinates __coordinates) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _coordinates __coordinates)
   self)
  (:coordinates
   (&optional __coordinates)
   (if __coordinates (setq _coordinates __coordinates)) _coordinates)
  (:serialization-length
   ()
   (+
    ;; int32[] _coordinates
    (* 4    (length _coordinates)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _coordinates
     (write-long (length _coordinates) s)
     (dotimes (i (length _coordinates))
       (write-long (elt _coordinates i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _coordinates
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _coordinates (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _coordinates i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get uwb_dummy::Coordinates :md5sum-) "c74c07aeada55d213db43904e2c9d02a")
(setf (get uwb_dummy::Coordinates :datatype-) "uwb_dummy/Coordinates")
(setf (get uwb_dummy::Coordinates :definition-)
      "int32[] coordinates

")



(provide :uwb_dummy/Coordinates "c74c07aeada55d213db43904e2c9d02a")


