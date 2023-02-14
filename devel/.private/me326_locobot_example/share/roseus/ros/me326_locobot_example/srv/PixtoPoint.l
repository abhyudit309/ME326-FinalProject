;; Auto-generated. Do not edit!


(when (boundp 'me326_locobot_example::PixtoPoint)
  (if (not (find-package "ME326_LOCOBOT_EXAMPLE"))
    (make-package "ME326_LOCOBOT_EXAMPLE"))
  (shadow 'PixtoPoint (find-package "ME326_LOCOBOT_EXAMPLE")))
(unless (find-package "ME326_LOCOBOT_EXAMPLE::PIXTOPOINT")
  (make-package "ME326_LOCOBOT_EXAMPLE::PIXTOPOINT"))
(unless (find-package "ME326_LOCOBOT_EXAMPLE::PIXTOPOINTREQUEST")
  (make-package "ME326_LOCOBOT_EXAMPLE::PIXTOPOINTREQUEST"))
(unless (find-package "ME326_LOCOBOT_EXAMPLE::PIXTOPOINTRESPONSE")
  (make-package "ME326_LOCOBOT_EXAMPLE::PIXTOPOINTRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass me326_locobot_example::PixtoPointRequest
  :super ros::object
  :slots ())

(defmethod me326_locobot_example::PixtoPointRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass me326_locobot_example::PixtoPointResponse
  :super ros::object
  :slots (_ptCld_point ))

(defmethod me326_locobot_example::PixtoPointResponse
  (:init
   (&key
    ((:ptCld_point __ptCld_point) (instance geometry_msgs::PointStamped :init))
    )
   (send-super :init)
   (setq _ptCld_point __ptCld_point)
   self)
  (:ptCld_point
   (&rest __ptCld_point)
   (if (keywordp (car __ptCld_point))
       (send* _ptCld_point __ptCld_point)
     (progn
       (if __ptCld_point (setq _ptCld_point (car __ptCld_point)))
       _ptCld_point)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PointStamped _ptCld_point
    (send _ptCld_point :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PointStamped _ptCld_point
       (send _ptCld_point :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PointStamped _ptCld_point
     (send _ptCld_point :deserialize buf ptr-) (incf ptr- (send _ptCld_point :serialization-length))
   ;;
   self)
  )

(defclass me326_locobot_example::PixtoPoint
  :super ros::object
  :slots ())

(setf (get me326_locobot_example::PixtoPoint :md5sum-) "453b790c7c72ce0c0a5e253a59f6dc48")
(setf (get me326_locobot_example::PixtoPoint :datatype-) "me326_locobot_example/PixtoPoint")
(setf (get me326_locobot_example::PixtoPoint :request) me326_locobot_example::PixtoPointRequest)
(setf (get me326_locobot_example::PixtoPoint :response) me326_locobot_example::PixtoPointResponse)

(defmethod me326_locobot_example::PixtoPointRequest
  (:response () (instance me326_locobot_example::PixtoPointResponse :init)))

(setf (get me326_locobot_example::PixtoPointRequest :md5sum-) "453b790c7c72ce0c0a5e253a59f6dc48")
(setf (get me326_locobot_example::PixtoPointRequest :datatype-) "me326_locobot_example/PixtoPointRequest")
(setf (get me326_locobot_example::PixtoPointRequest :definition-)
      "---
geometry_msgs/PointStamped ptCld_point


================================================================================
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

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
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get me326_locobot_example::PixtoPointResponse :md5sum-) "453b790c7c72ce0c0a5e253a59f6dc48")
(setf (get me326_locobot_example::PixtoPointResponse :datatype-) "me326_locobot_example/PixtoPointResponse")
(setf (get me326_locobot_example::PixtoPointResponse :definition-)
      "---
geometry_msgs/PointStamped ptCld_point


================================================================================
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

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
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :me326_locobot_example/PixtoPoint "453b790c7c72ce0c0a5e253a59f6dc48")


