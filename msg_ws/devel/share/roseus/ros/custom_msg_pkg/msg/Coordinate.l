;; Auto-generated. Do not edit!


(when (boundp 'custom_msg_pkg::Coordinate)
  (if (not (find-package "CUSTOM_MSG_PKG"))
    (make-package "CUSTOM_MSG_PKG"))
  (shadow 'Coordinate (find-package "CUSTOM_MSG_PKG")))
(unless (find-package "CUSTOM_MSG_PKG::COORDINATE")
  (make-package "CUSTOM_MSG_PKG::COORDINATE"))

(in-package "ROS")
;;//! \htmlinclude Coordinate.msg.html


(defclass custom_msg_pkg::Coordinate
  :super ros::object
  :slots (_start_time _msg_seq _x _y _z ))

(defmethod custom_msg_pkg::Coordinate
  (:init
   (&key
    ((:start_time __start_time) (instance ros::time :init))
    ((:msg_seq __msg_seq) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _start_time __start_time)
   (setq _msg_seq (round __msg_seq))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
  (:msg_seq
   (&optional __msg_seq)
   (if __msg_seq (setq _msg_seq __msg_seq)) _msg_seq)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; time _start_time
    8
    ;; uint16 _msg_seq
    2
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _start_time
       (write-long (send _start_time :sec) s) (write-long (send _start_time :nsec) s)
     ;; uint16 _msg_seq
       (write-word _msg_seq s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _start_time
     (send _start_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _msg_seq
     (setq _msg_seq (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get custom_msg_pkg::Coordinate :md5sum-) "b83f43423351737928ddd0624ca7902e")
(setf (get custom_msg_pkg::Coordinate :datatype-) "custom_msg_pkg/Coordinate")
(setf (get custom_msg_pkg::Coordinate :definition-)
      "time start_time
uint16 msg_seq
float64 x
float64 y
float64 z

")



(provide :custom_msg_pkg/Coordinate "b83f43423351737928ddd0624ca7902e")


