; Auto-generated. Do not edit!


(cl:in-package interbotix_perception_modules-srv)


;//! \htmlinclude ClusterInfoArray-request.msg.html

(cl:defclass <ClusterInfoArray-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ClusterInfoArray-request (<ClusterInfoArray-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ClusterInfoArray-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ClusterInfoArray-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interbotix_perception_modules-srv:<ClusterInfoArray-request> is deprecated: use interbotix_perception_modules-srv:ClusterInfoArray-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ClusterInfoArray-request>) ostream)
  "Serializes a message object of type '<ClusterInfoArray-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ClusterInfoArray-request>) istream)
  "Deserializes a message object of type '<ClusterInfoArray-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ClusterInfoArray-request>)))
  "Returns string type for a service object of type '<ClusterInfoArray-request>"
  "interbotix_perception_modules/ClusterInfoArrayRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClusterInfoArray-request)))
  "Returns string type for a service object of type 'ClusterInfoArray-request"
  "interbotix_perception_modules/ClusterInfoArrayRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ClusterInfoArray-request>)))
  "Returns md5sum for a message object of type '<ClusterInfoArray-request>"
  "911bd211bd6fd7e568c4562cf40f28fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ClusterInfoArray-request)))
  "Returns md5sum for a message object of type 'ClusterInfoArray-request"
  "911bd211bd6fd7e568c4562cf40f28fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ClusterInfoArray-request>)))
  "Returns full string definition for message of type '<ClusterInfoArray-request>"
  (cl:format cl:nil "# This service is used specifically in the interbotix_perception_modules package~%#~%# List of ClusterInfo messages. The length of the list should correspond to the~%# number of objects seen by the camera~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ClusterInfoArray-request)))
  "Returns full string definition for message of type 'ClusterInfoArray-request"
  (cl:format cl:nil "# This service is used specifically in the interbotix_perception_modules package~%#~%# List of ClusterInfo messages. The length of the list should correspond to the~%# number of objects seen by the camera~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ClusterInfoArray-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ClusterInfoArray-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ClusterInfoArray-request
))
;//! \htmlinclude ClusterInfoArray-response.msg.html

(cl:defclass <ClusterInfoArray-response> (roslisp-msg-protocol:ros-message)
  ((clusters
    :reader clusters
    :initarg :clusters
    :type (cl:vector interbotix_perception_modules-msg:ClusterInfo)
   :initform (cl:make-array 0 :element-type 'interbotix_perception_modules-msg:ClusterInfo :initial-element (cl:make-instance 'interbotix_perception_modules-msg:ClusterInfo))))
)

(cl:defclass ClusterInfoArray-response (<ClusterInfoArray-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ClusterInfoArray-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ClusterInfoArray-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interbotix_perception_modules-srv:<ClusterInfoArray-response> is deprecated: use interbotix_perception_modules-srv:ClusterInfoArray-response instead.")))

(cl:ensure-generic-function 'clusters-val :lambda-list '(m))
(cl:defmethod clusters-val ((m <ClusterInfoArray-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:clusters-val is deprecated.  Use interbotix_perception_modules-srv:clusters instead.")
  (clusters m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ClusterInfoArray-response>) ostream)
  "Serializes a message object of type '<ClusterInfoArray-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'clusters))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'clusters))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ClusterInfoArray-response>) istream)
  "Deserializes a message object of type '<ClusterInfoArray-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'clusters) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'clusters)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'interbotix_perception_modules-msg:ClusterInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ClusterInfoArray-response>)))
  "Returns string type for a service object of type '<ClusterInfoArray-response>"
  "interbotix_perception_modules/ClusterInfoArrayResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClusterInfoArray-response)))
  "Returns string type for a service object of type 'ClusterInfoArray-response"
  "interbotix_perception_modules/ClusterInfoArrayResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ClusterInfoArray-response>)))
  "Returns md5sum for a message object of type '<ClusterInfoArray-response>"
  "911bd211bd6fd7e568c4562cf40f28fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ClusterInfoArray-response)))
  "Returns md5sum for a message object of type 'ClusterInfoArray-response"
  "911bd211bd6fd7e568c4562cf40f28fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ClusterInfoArray-response>)))
  "Returns full string definition for message of type '<ClusterInfoArray-response>"
  (cl:format cl:nil "interbotix_perception_modules/ClusterInfo[] clusters~%~%~%================================================================================~%MSG: interbotix_perception_modules/ClusterInfo~%# This message is used specifically in the interbotix_perception_modules package~%#~%# Message that holds information about each cluster~%~%string frame_id                     # parent frame of the cluster (usually 'camera_depth_optical_frame' or similar)~%geometry_msgs/Point position        # x, y, z position of the cluster~%float32 yaw                         # yaw [rad] of the cluster; x-axis of the cluster should align~%                                    # with the major-axis of a best-fit ellipse (currently not supported)~%std_msgs/ColorRGBA color            # average RGB values (0 - 255) for the whole cluster~%geometry_msgs/Point min_z_point     # point with the min 'z' value of the cluster~%int32 num_points                    # number of points in the cluster~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ClusterInfoArray-response)))
  "Returns full string definition for message of type 'ClusterInfoArray-response"
  (cl:format cl:nil "interbotix_perception_modules/ClusterInfo[] clusters~%~%~%================================================================================~%MSG: interbotix_perception_modules/ClusterInfo~%# This message is used specifically in the interbotix_perception_modules package~%#~%# Message that holds information about each cluster~%~%string frame_id                     # parent frame of the cluster (usually 'camera_depth_optical_frame' or similar)~%geometry_msgs/Point position        # x, y, z position of the cluster~%float32 yaw                         # yaw [rad] of the cluster; x-axis of the cluster should align~%                                    # with the major-axis of a best-fit ellipse (currently not supported)~%std_msgs/ColorRGBA color            # average RGB values (0 - 255) for the whole cluster~%geometry_msgs/Point min_z_point     # point with the min 'z' value of the cluster~%int32 num_points                    # number of points in the cluster~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ClusterInfoArray-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'clusters) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ClusterInfoArray-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ClusterInfoArray-response
    (cl:cons ':clusters (clusters msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ClusterInfoArray)))
  'ClusterInfoArray-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ClusterInfoArray)))
  'ClusterInfoArray-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClusterInfoArray)))
  "Returns string type for a service object of type '<ClusterInfoArray>"
  "interbotix_perception_modules/ClusterInfoArray")