; Auto-generated. Do not edit!


(cl:in-package interbotix_perception_modules-srv)


;//! \htmlinclude SnapPicture-request.msg.html

(cl:defclass <SnapPicture-request> (roslisp-msg-protocol:ros-message)
  ((filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform ""))
)

(cl:defclass SnapPicture-request (<SnapPicture-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SnapPicture-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SnapPicture-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interbotix_perception_modules-srv:<SnapPicture-request> is deprecated: use interbotix_perception_modules-srv:SnapPicture-request instead.")))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <SnapPicture-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:filename-val is deprecated.  Use interbotix_perception_modules-srv:filename instead.")
  (filename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SnapPicture-request>) ostream)
  "Serializes a message object of type '<SnapPicture-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SnapPicture-request>) istream)
  "Deserializes a message object of type '<SnapPicture-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SnapPicture-request>)))
  "Returns string type for a service object of type '<SnapPicture-request>"
  "interbotix_perception_modules/SnapPictureRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SnapPicture-request)))
  "Returns string type for a service object of type 'SnapPicture-request"
  "interbotix_perception_modules/SnapPictureRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SnapPicture-request>)))
  "Returns md5sum for a message object of type '<SnapPicture-request>"
  "a8777594bfb7f85102a1426edcb42742")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SnapPicture-request)))
  "Returns md5sum for a message object of type 'SnapPicture-request"
  "a8777594bfb7f85102a1426edcb42742")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SnapPicture-request>)))
  "Returns full string definition for message of type '<SnapPicture-request>"
  (cl:format cl:nil "# This service is used specifically in the interbotix_perception_modules package~%#~%# Save the latest rgb picture with the specified name~%#~%# Request consists of:~%#   filename : name of file in which to save image including extension (.jpg)~%#~%# Response consists of:~%#    success : boolean indication of service success~%~%string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SnapPicture-request)))
  "Returns full string definition for message of type 'SnapPicture-request"
  (cl:format cl:nil "# This service is used specifically in the interbotix_perception_modules package~%#~%# Save the latest rgb picture with the specified name~%#~%# Request consists of:~%#   filename : name of file in which to save image including extension (.jpg)~%#~%# Response consists of:~%#    success : boolean indication of service success~%~%string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SnapPicture-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SnapPicture-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SnapPicture-request
    (cl:cons ':filename (filename msg))
))
;//! \htmlinclude SnapPicture-response.msg.html

(cl:defclass <SnapPicture-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (filepath
    :reader filepath
    :initarg :filepath
    :type cl:string
    :initform ""))
)

(cl:defclass SnapPicture-response (<SnapPicture-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SnapPicture-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SnapPicture-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interbotix_perception_modules-srv:<SnapPicture-response> is deprecated: use interbotix_perception_modules-srv:SnapPicture-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SnapPicture-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:success-val is deprecated.  Use interbotix_perception_modules-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'filepath-val :lambda-list '(m))
(cl:defmethod filepath-val ((m <SnapPicture-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:filepath-val is deprecated.  Use interbotix_perception_modules-srv:filepath instead.")
  (filepath m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SnapPicture-response>) ostream)
  "Serializes a message object of type '<SnapPicture-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filepath))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filepath))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SnapPicture-response>) istream)
  "Deserializes a message object of type '<SnapPicture-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filepath) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filepath) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SnapPicture-response>)))
  "Returns string type for a service object of type '<SnapPicture-response>"
  "interbotix_perception_modules/SnapPictureResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SnapPicture-response)))
  "Returns string type for a service object of type 'SnapPicture-response"
  "interbotix_perception_modules/SnapPictureResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SnapPicture-response>)))
  "Returns md5sum for a message object of type '<SnapPicture-response>"
  "a8777594bfb7f85102a1426edcb42742")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SnapPicture-response)))
  "Returns md5sum for a message object of type 'SnapPicture-response"
  "a8777594bfb7f85102a1426edcb42742")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SnapPicture-response>)))
  "Returns full string definition for message of type '<SnapPicture-response>"
  (cl:format cl:nil "bool success~%string filepath~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SnapPicture-response)))
  "Returns full string definition for message of type 'SnapPicture-response"
  (cl:format cl:nil "bool success~%string filepath~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SnapPicture-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'filepath))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SnapPicture-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SnapPicture-response
    (cl:cons ':success (success msg))
    (cl:cons ':filepath (filepath msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SnapPicture)))
  'SnapPicture-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SnapPicture)))
  'SnapPicture-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SnapPicture)))
  "Returns string type for a service object of type '<SnapPicture>"
  "interbotix_perception_modules/SnapPicture")