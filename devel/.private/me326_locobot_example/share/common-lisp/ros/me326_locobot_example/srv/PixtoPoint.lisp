; Auto-generated. Do not edit!


(cl:in-package me326_locobot_example-srv)


;//! \htmlinclude PixtoPoint-request.msg.html

(cl:defclass <PixtoPoint-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass PixtoPoint-request (<PixtoPoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PixtoPoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PixtoPoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name me326_locobot_example-srv:<PixtoPoint-request> is deprecated: use me326_locobot_example-srv:PixtoPoint-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PixtoPoint-request>) ostream)
  "Serializes a message object of type '<PixtoPoint-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PixtoPoint-request>) istream)
  "Deserializes a message object of type '<PixtoPoint-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PixtoPoint-request>)))
  "Returns string type for a service object of type '<PixtoPoint-request>"
  "me326_locobot_example/PixtoPointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PixtoPoint-request)))
  "Returns string type for a service object of type 'PixtoPoint-request"
  "me326_locobot_example/PixtoPointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PixtoPoint-request>)))
  "Returns md5sum for a message object of type '<PixtoPoint-request>"
  "453b790c7c72ce0c0a5e253a59f6dc48")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PixtoPoint-request)))
  "Returns md5sum for a message object of type 'PixtoPoint-request"
  "453b790c7c72ce0c0a5e253a59f6dc48")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PixtoPoint-request>)))
  "Returns full string definition for message of type '<PixtoPoint-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PixtoPoint-request)))
  "Returns full string definition for message of type 'PixtoPoint-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PixtoPoint-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PixtoPoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PixtoPoint-request
))
;//! \htmlinclude PixtoPoint-response.msg.html

(cl:defclass <PixtoPoint-response> (roslisp-msg-protocol:ros-message)
  ((ptCld_point
    :reader ptCld_point
    :initarg :ptCld_point
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped)))
)

(cl:defclass PixtoPoint-response (<PixtoPoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PixtoPoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PixtoPoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name me326_locobot_example-srv:<PixtoPoint-response> is deprecated: use me326_locobot_example-srv:PixtoPoint-response instead.")))

(cl:ensure-generic-function 'ptCld_point-val :lambda-list '(m))
(cl:defmethod ptCld_point-val ((m <PixtoPoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader me326_locobot_example-srv:ptCld_point-val is deprecated.  Use me326_locobot_example-srv:ptCld_point instead.")
  (ptCld_point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PixtoPoint-response>) ostream)
  "Serializes a message object of type '<PixtoPoint-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ptCld_point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PixtoPoint-response>) istream)
  "Deserializes a message object of type '<PixtoPoint-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ptCld_point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PixtoPoint-response>)))
  "Returns string type for a service object of type '<PixtoPoint-response>"
  "me326_locobot_example/PixtoPointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PixtoPoint-response)))
  "Returns string type for a service object of type 'PixtoPoint-response"
  "me326_locobot_example/PixtoPointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PixtoPoint-response>)))
  "Returns md5sum for a message object of type '<PixtoPoint-response>"
  "453b790c7c72ce0c0a5e253a59f6dc48")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PixtoPoint-response)))
  "Returns md5sum for a message object of type 'PixtoPoint-response"
  "453b790c7c72ce0c0a5e253a59f6dc48")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PixtoPoint-response>)))
  "Returns full string definition for message of type '<PixtoPoint-response>"
  (cl:format cl:nil "geometry_msgs/PointStamped ptCld_point~%~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PixtoPoint-response)))
  "Returns full string definition for message of type 'PixtoPoint-response"
  (cl:format cl:nil "geometry_msgs/PointStamped ptCld_point~%~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PixtoPoint-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ptCld_point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PixtoPoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PixtoPoint-response
    (cl:cons ':ptCld_point (ptCld_point msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PixtoPoint)))
  'PixtoPoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PixtoPoint)))
  'PixtoPoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PixtoPoint)))
  "Returns string type for a service object of type '<PixtoPoint>"
  "me326_locobot_example/PixtoPoint")