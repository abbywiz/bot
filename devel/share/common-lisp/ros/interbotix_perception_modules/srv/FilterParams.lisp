; Auto-generated. Do not edit!


(cl:in-package interbotix_perception_modules-srv)


;//! \htmlinclude FilterParams-request.msg.html

(cl:defclass <FilterParams-request> (roslisp-msg-protocol:ros-message)
  ((x_filter_min
    :reader x_filter_min
    :initarg :x_filter_min
    :type cl:float
    :initform 0.0)
   (x_filter_max
    :reader x_filter_max
    :initarg :x_filter_max
    :type cl:float
    :initform 0.0)
   (y_filter_min
    :reader y_filter_min
    :initarg :y_filter_min
    :type cl:float
    :initform 0.0)
   (y_filter_max
    :reader y_filter_max
    :initarg :y_filter_max
    :type cl:float
    :initform 0.0)
   (z_filter_min
    :reader z_filter_min
    :initarg :z_filter_min
    :type cl:float
    :initform 0.0)
   (z_filter_max
    :reader z_filter_max
    :initarg :z_filter_max
    :type cl:float
    :initform 0.0)
   (voxel_leaf_size
    :reader voxel_leaf_size
    :initarg :voxel_leaf_size
    :type cl:float
    :initform 0.0)
   (plane_max_iter
    :reader plane_max_iter
    :initarg :plane_max_iter
    :type cl:integer
    :initform 0)
   (plane_dist_thresh
    :reader plane_dist_thresh
    :initarg :plane_dist_thresh
    :type cl:float
    :initform 0.0)
   (ror_radius_search
    :reader ror_radius_search
    :initarg :ror_radius_search
    :type cl:float
    :initform 0.0)
   (ror_min_neighbors
    :reader ror_min_neighbors
    :initarg :ror_min_neighbors
    :type cl:integer
    :initform 0)
   (cluster_tol
    :reader cluster_tol
    :initarg :cluster_tol
    :type cl:float
    :initform 0.0)
   (cluster_min_size
    :reader cluster_min_size
    :initarg :cluster_min_size
    :type cl:integer
    :initform 0)
   (cluster_max_size
    :reader cluster_max_size
    :initarg :cluster_max_size
    :type cl:integer
    :initform 0))
)

(cl:defclass FilterParams-request (<FilterParams-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FilterParams-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FilterParams-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interbotix_perception_modules-srv:<FilterParams-request> is deprecated: use interbotix_perception_modules-srv:FilterParams-request instead.")))

(cl:ensure-generic-function 'x_filter_min-val :lambda-list '(m))
(cl:defmethod x_filter_min-val ((m <FilterParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:x_filter_min-val is deprecated.  Use interbotix_perception_modules-srv:x_filter_min instead.")
  (x_filter_min m))

(cl:ensure-generic-function 'x_filter_max-val :lambda-list '(m))
(cl:defmethod x_filter_max-val ((m <FilterParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:x_filter_max-val is deprecated.  Use interbotix_perception_modules-srv:x_filter_max instead.")
  (x_filter_max m))

(cl:ensure-generic-function 'y_filter_min-val :lambda-list '(m))
(cl:defmethod y_filter_min-val ((m <FilterParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:y_filter_min-val is deprecated.  Use interbotix_perception_modules-srv:y_filter_min instead.")
  (y_filter_min m))

(cl:ensure-generic-function 'y_filter_max-val :lambda-list '(m))
(cl:defmethod y_filter_max-val ((m <FilterParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:y_filter_max-val is deprecated.  Use interbotix_perception_modules-srv:y_filter_max instead.")
  (y_filter_max m))

(cl:ensure-generic-function 'z_filter_min-val :lambda-list '(m))
(cl:defmethod z_filter_min-val ((m <FilterParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:z_filter_min-val is deprecated.  Use interbotix_perception_modules-srv:z_filter_min instead.")
  (z_filter_min m))

(cl:ensure-generic-function 'z_filter_max-val :lambda-list '(m))
(cl:defmethod z_filter_max-val ((m <FilterParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:z_filter_max-val is deprecated.  Use interbotix_perception_modules-srv:z_filter_max instead.")
  (z_filter_max m))

(cl:ensure-generic-function 'voxel_leaf_size-val :lambda-list '(m))
(cl:defmethod voxel_leaf_size-val ((m <FilterParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:voxel_leaf_size-val is deprecated.  Use interbotix_perception_modules-srv:voxel_leaf_size instead.")
  (voxel_leaf_size m))

(cl:ensure-generic-function 'plane_max_iter-val :lambda-list '(m))
(cl:defmethod plane_max_iter-val ((m <FilterParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:plane_max_iter-val is deprecated.  Use interbotix_perception_modules-srv:plane_max_iter instead.")
  (plane_max_iter m))

(cl:ensure-generic-function 'plane_dist_thresh-val :lambda-list '(m))
(cl:defmethod plane_dist_thresh-val ((m <FilterParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:plane_dist_thresh-val is deprecated.  Use interbotix_perception_modules-srv:plane_dist_thresh instead.")
  (plane_dist_thresh m))

(cl:ensure-generic-function 'ror_radius_search-val :lambda-list '(m))
(cl:defmethod ror_radius_search-val ((m <FilterParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:ror_radius_search-val is deprecated.  Use interbotix_perception_modules-srv:ror_radius_search instead.")
  (ror_radius_search m))

(cl:ensure-generic-function 'ror_min_neighbors-val :lambda-list '(m))
(cl:defmethod ror_min_neighbors-val ((m <FilterParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:ror_min_neighbors-val is deprecated.  Use interbotix_perception_modules-srv:ror_min_neighbors instead.")
  (ror_min_neighbors m))

(cl:ensure-generic-function 'cluster_tol-val :lambda-list '(m))
(cl:defmethod cluster_tol-val ((m <FilterParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:cluster_tol-val is deprecated.  Use interbotix_perception_modules-srv:cluster_tol instead.")
  (cluster_tol m))

(cl:ensure-generic-function 'cluster_min_size-val :lambda-list '(m))
(cl:defmethod cluster_min_size-val ((m <FilterParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:cluster_min_size-val is deprecated.  Use interbotix_perception_modules-srv:cluster_min_size instead.")
  (cluster_min_size m))

(cl:ensure-generic-function 'cluster_max_size-val :lambda-list '(m))
(cl:defmethod cluster_max_size-val ((m <FilterParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interbotix_perception_modules-srv:cluster_max_size-val is deprecated.  Use interbotix_perception_modules-srv:cluster_max_size instead.")
  (cluster_max_size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FilterParams-request>) ostream)
  "Serializes a message object of type '<FilterParams-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x_filter_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x_filter_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y_filter_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y_filter_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z_filter_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z_filter_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'voxel_leaf_size))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'plane_max_iter)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'plane_dist_thresh))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ror_radius_search))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'ror_min_neighbors)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cluster_tol))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'cluster_min_size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cluster_max_size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FilterParams-request>) istream)
  "Deserializes a message object of type '<FilterParams-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_filter_min) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_filter_max) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_filter_min) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_filter_max) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_filter_min) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_filter_max) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'voxel_leaf_size) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'plane_max_iter) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'plane_dist_thresh) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ror_radius_search) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ror_min_neighbors) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cluster_tol) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cluster_min_size) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cluster_max_size) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FilterParams-request>)))
  "Returns string type for a service object of type '<FilterParams-request>"
  "interbotix_perception_modules/FilterParamsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FilterParams-request)))
  "Returns string type for a service object of type 'FilterParams-request"
  "interbotix_perception_modules/FilterParamsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FilterParams-request>)))
  "Returns md5sum for a message object of type '<FilterParams-request>"
  "57d51b939efff834ad5e55a181a7a0a0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FilterParams-request)))
  "Returns md5sum for a message object of type 'FilterParams-request"
  "57d51b939efff834ad5e55a181a7a0a0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FilterParams-request>)))
  "Returns full string definition for message of type '<FilterParams-request>"
  (cl:format cl:nil "# This service is used specifically in the interbotix_perception_modules package~%#~%# Parameters used to tune the various PCL Pointcloud filters~%~%float32 x_filter_min            # minimum value [m] along the x axis beyond which all points will be cropped when doing Crop Box Filtering~%float32 x_filter_max            # maximum value [m] along the x axis beyond which all points will be cropped when doing Crop Box Filtering~%float32 y_filter_min            # minimum value [m] along the y axis beyond which all points will be cropped when doing Crop Box Filtering~%float32 y_filter_max            # maximum value [m] along the y axis beyond which all points will be cropped when doing Crop Box Filtering~%float32 z_filter_min            # minimum value [m] along the z axis beyond which all points will be cropped when doing Crop Box Filtering~%float32 z_filter_max            # maximum value [m] along the z axis beyond which all points will be cropped when doing Crop Box Filtering~%float32 voxel_leaf_size         # voxel leaf size [m] for the x, y, and z axes when doing Voxel Filtering~%int32 plane_max_iter            # maximum number of iterations the algorithm should go when when doing Planar Segmentation~%float32 plane_dist_thresh       # distance [m] perpendicular from the planar model outside of which points should not be segmented out when doing Planar Segmentation~%float32 ror_radius_search       # desired search radius when doing Radius Outlier Removal~%int32 ror_min_neighbors         # minimum number of neighbors a point should have to not be cropped out when doing Radius Outlier Removal~%float32 cluster_tol             # any point within this distance [m] will be considered part of the same cluster~%int32 cluster_min_size          # minimum number of points a cluster must have to be considered a cluster~%int32 cluster_max_size          # maximum number of points a cluster can have to be considered a cluster~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FilterParams-request)))
  "Returns full string definition for message of type 'FilterParams-request"
  (cl:format cl:nil "# This service is used specifically in the interbotix_perception_modules package~%#~%# Parameters used to tune the various PCL Pointcloud filters~%~%float32 x_filter_min            # minimum value [m] along the x axis beyond which all points will be cropped when doing Crop Box Filtering~%float32 x_filter_max            # maximum value [m] along the x axis beyond which all points will be cropped when doing Crop Box Filtering~%float32 y_filter_min            # minimum value [m] along the y axis beyond which all points will be cropped when doing Crop Box Filtering~%float32 y_filter_max            # maximum value [m] along the y axis beyond which all points will be cropped when doing Crop Box Filtering~%float32 z_filter_min            # minimum value [m] along the z axis beyond which all points will be cropped when doing Crop Box Filtering~%float32 z_filter_max            # maximum value [m] along the z axis beyond which all points will be cropped when doing Crop Box Filtering~%float32 voxel_leaf_size         # voxel leaf size [m] for the x, y, and z axes when doing Voxel Filtering~%int32 plane_max_iter            # maximum number of iterations the algorithm should go when when doing Planar Segmentation~%float32 plane_dist_thresh       # distance [m] perpendicular from the planar model outside of which points should not be segmented out when doing Planar Segmentation~%float32 ror_radius_search       # desired search radius when doing Radius Outlier Removal~%int32 ror_min_neighbors         # minimum number of neighbors a point should have to not be cropped out when doing Radius Outlier Removal~%float32 cluster_tol             # any point within this distance [m] will be considered part of the same cluster~%int32 cluster_min_size          # minimum number of points a cluster must have to be considered a cluster~%int32 cluster_max_size          # maximum number of points a cluster can have to be considered a cluster~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FilterParams-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FilterParams-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FilterParams-request
    (cl:cons ':x_filter_min (x_filter_min msg))
    (cl:cons ':x_filter_max (x_filter_max msg))
    (cl:cons ':y_filter_min (y_filter_min msg))
    (cl:cons ':y_filter_max (y_filter_max msg))
    (cl:cons ':z_filter_min (z_filter_min msg))
    (cl:cons ':z_filter_max (z_filter_max msg))
    (cl:cons ':voxel_leaf_size (voxel_leaf_size msg))
    (cl:cons ':plane_max_iter (plane_max_iter msg))
    (cl:cons ':plane_dist_thresh (plane_dist_thresh msg))
    (cl:cons ':ror_radius_search (ror_radius_search msg))
    (cl:cons ':ror_min_neighbors (ror_min_neighbors msg))
    (cl:cons ':cluster_tol (cluster_tol msg))
    (cl:cons ':cluster_min_size (cluster_min_size msg))
    (cl:cons ':cluster_max_size (cluster_max_size msg))
))
;//! \htmlinclude FilterParams-response.msg.html

(cl:defclass <FilterParams-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FilterParams-response (<FilterParams-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FilterParams-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FilterParams-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interbotix_perception_modules-srv:<FilterParams-response> is deprecated: use interbotix_perception_modules-srv:FilterParams-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FilterParams-response>) ostream)
  "Serializes a message object of type '<FilterParams-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FilterParams-response>) istream)
  "Deserializes a message object of type '<FilterParams-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FilterParams-response>)))
  "Returns string type for a service object of type '<FilterParams-response>"
  "interbotix_perception_modules/FilterParamsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FilterParams-response)))
  "Returns string type for a service object of type 'FilterParams-response"
  "interbotix_perception_modules/FilterParamsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FilterParams-response>)))
  "Returns md5sum for a message object of type '<FilterParams-response>"
  "57d51b939efff834ad5e55a181a7a0a0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FilterParams-response)))
  "Returns md5sum for a message object of type 'FilterParams-response"
  "57d51b939efff834ad5e55a181a7a0a0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FilterParams-response>)))
  "Returns full string definition for message of type '<FilterParams-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FilterParams-response)))
  "Returns full string definition for message of type 'FilterParams-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FilterParams-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FilterParams-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FilterParams-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FilterParams)))
  'FilterParams-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FilterParams)))
  'FilterParams-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FilterParams)))
  "Returns string type for a service object of type '<FilterParams>"
  "interbotix_perception_modules/FilterParams")