; Auto-generated. Do not edit!


(cl:in-package interbotix_perception_modules-msg)


;//! \htmlinclude ClusterInfo.msg.html

(cl:defclass <ClusterInfo> (roslisp-msg-protocol:ros-message)
  ((frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:string
    :initform "")
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (color
    :reader color
    :initarg :color
    :type std_msgs-msg:ColorRGBA
    :initform (cl:make-instance 'std_msgs-msg:ColorRGBA))
   (min_z_point
    :reader min_z_point
    :initarg :min_z_point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (num_points
    :reader num_points
    :initarg :num_points
    :type cl:integer
    :initform 0))
)

(cl:defclass ClusterInfo (<ClusterInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ClusterInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ClusterInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interbotix_perception_modules-msg:<ClusterInfo> is deprecated: use interbotix_perception_modules-msg:ClusterInfo instead.")))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <ClusterInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-msg:frame_id-val is deprecated.  Use interbotix_perception_modules-msg:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <ClusterInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-msg:position-val is deprecated.  Use interbotix_perception_modules-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <ClusterInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-msg:yaw-val is deprecated.  Use interbotix_perception_modules-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <ClusterInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-msg:color-val is deprecated.  Use interbotix_perception_modules-msg:color instead.")
  (color m))

(cl:ensure-generic-function 'min_z_point-val :lambda-list '(m))
(cl:defmethod min_z_point-val ((m <ClusterInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-msg:min_z_point-val is deprecated.  Use interbotix_perception_modules-msg:min_z_point instead.")
  (min_z_point m))

(cl:ensure-generic-function 'num_points-val :lambda-list '(m))
(cl:defmethod num_points-val ((m <ClusterInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-msg:num_points-val is deprecated.  Use interbotix_perception_modules-msg:num_points instead.")
  (num_points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ClusterInfo>) ostream)
  "Serializes a message object of type '<ClusterInfo>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'frame_id))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'color) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min_z_point) ostream)
  (cl:let* ((signed (cl:slot-value msg 'num_points)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ClusterInfo>) istream)
  "Deserializes a message object of type '<ClusterInfo>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'color) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min_z_point) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_points) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ClusterInfo>)))
  "Returns string type for a message object of type '<ClusterInfo>"
  "interbotix_perception_modules/ClusterInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClusterInfo)))
  "Returns string type for a message object of type 'ClusterInfo"
  "interbotix_perception_modules/ClusterInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ClusterInfo>)))
  "Returns md5sum for a message object of type '<ClusterInfo>"
  "9627d2d019ccc4ed2077673b4b0f3b40")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ClusterInfo)))
  "Returns md5sum for a message object of type 'ClusterInfo"
  "9627d2d019ccc4ed2077673b4b0f3b40")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ClusterInfo>)))
  "Returns full string definition for message of type '<ClusterInfo>"
  (cl:format cl:nil "# This message is used specifically in the interbotix_perception_modules package~%#~%# Message that holds information about each cluster~%~%string frame_id                     # parent frame of the cluster (usually 'camera_depth_optical_frame' or similar)~%geometry_msgs/Point position        # x, y, z position of the cluster~%float32 yaw                         # yaw [rad] of the cluster; x-axis of the cluster should align~%                                    # with the major-axis of a best-fit ellipse (currently not supported)~%std_msgs/ColorRGBA color            # average RGB values (0 - 255) for the whole cluster~%geometry_msgs/Point min_z_point     # point with the min 'z' value of the cluster~%int32 num_points                    # number of points in the cluster~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ClusterInfo)))
  "Returns full string definition for message of type 'ClusterInfo"
  (cl:format cl:nil "# This message is used specifically in the interbotix_perception_modules package~%#~%# Message that holds information about each cluster~%~%string frame_id                     # parent frame of the cluster (usually 'camera_depth_optical_frame' or similar)~%geometry_msgs/Point position        # x, y, z position of the cluster~%float32 yaw                         # yaw [rad] of the cluster; x-axis of the cluster should align~%                                    # with the major-axis of a best-fit ellipse (currently not supported)~%std_msgs/ColorRGBA color            # average RGB values (0 - 255) for the whole cluster~%geometry_msgs/Point min_z_point     # point with the min 'z' value of the cluster~%int32 num_points                    # number of points in the cluster~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ClusterInfo>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'frame_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'color))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min_z_point))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ClusterInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'ClusterInfo
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':position (position msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':color (color msg))
    (cl:cons ':min_z_point (min_z_point msg))
    (cl:cons ':num_points (num_points msg))
))
