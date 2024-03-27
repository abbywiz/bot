;; Auto-generated. Do not edit!


(when (boundp 'interbotix_perception_modules::FilterParams)
  (if (not (find-package "INTERBOTIX_PERCEPTION_MODULES"))
    (make-package "INTERBOTIX_PERCEPTION_MODULES"))
  (shadow 'FilterParams (find-package "INTERBOTIX_PERCEPTION_MODULES")))
(unless (find-package "INTERBOTIX_PERCEPTION_MODULES::FILTERPARAMS")
  (make-package "INTERBOTIX_PERCEPTION_MODULES::FILTERPARAMS"))
(unless (find-package "INTERBOTIX_PERCEPTION_MODULES::FILTERPARAMSREQUEST")
  (make-package "INTERBOTIX_PERCEPTION_MODULES::FILTERPARAMSREQUEST"))
(unless (find-package "INTERBOTIX_PERCEPTION_MODULES::FILTERPARAMSRESPONSE")
  (make-package "INTERBOTIX_PERCEPTION_MODULES::FILTERPARAMSRESPONSE"))

(in-package "ROS")





(defclass interbotix_perception_modules::FilterParamsRequest
  :super ros::object
  :slots (_x_filter_min _x_filter_max _y_filter_min _y_filter_max _z_filter_min _z_filter_max _voxel_leaf_size _plane_max_iter _plane_dist_thresh _ror_radius_search _ror_min_neighbors _cluster_tol _cluster_min_size _cluster_max_size ))

(defmethod interbotix_perception_modules::FilterParamsRequest
  (:init
   (&key
    ((:x_filter_min __x_filter_min) 0.0)
    ((:x_filter_max __x_filter_max) 0.0)
    ((:y_filter_min __y_filter_min) 0.0)
    ((:y_filter_max __y_filter_max) 0.0)
    ((:z_filter_min __z_filter_min) 0.0)
    ((:z_filter_max __z_filter_max) 0.0)
    ((:voxel_leaf_size __voxel_leaf_size) 0.0)
    ((:plane_max_iter __plane_max_iter) 0)
    ((:plane_dist_thresh __plane_dist_thresh) 0.0)
    ((:ror_radius_search __ror_radius_search) 0.0)
    ((:ror_min_neighbors __ror_min_neighbors) 0)
    ((:cluster_tol __cluster_tol) 0.0)
    ((:cluster_min_size __cluster_min_size) 0)
    ((:cluster_max_size __cluster_max_size) 0)
    )
   (send-super :init)
   (setq _x_filter_min (float __x_filter_min))
   (setq _x_filter_max (float __x_filter_max))
   (setq _y_filter_min (float __y_filter_min))
   (setq _y_filter_max (float __y_filter_max))
   (setq _z_filter_min (float __z_filter_min))
   (setq _z_filter_max (float __z_filter_max))
   (setq _voxel_leaf_size (float __voxel_leaf_size))
   (setq _plane_max_iter (round __plane_max_iter))
   (setq _plane_dist_thresh (float __plane_dist_thresh))
   (setq _ror_radius_search (float __ror_radius_search))
   (setq _ror_min_neighbors (round __ror_min_neighbors))
   (setq _cluster_tol (float __cluster_tol))
   (setq _cluster_min_size (round __cluster_min_size))
   (setq _cluster_max_size (round __cluster_max_size))
   self)
  (:x_filter_min
   (&optional __x_filter_min)
   (if __x_filter_min (setq _x_filter_min __x_filter_min)) _x_filter_min)
  (:x_filter_max
   (&optional __x_filter_max)
   (if __x_filter_max (setq _x_filter_max __x_filter_max)) _x_filter_max)
  (:y_filter_min
   (&optional __y_filter_min)
   (if __y_filter_min (setq _y_filter_min __y_filter_min)) _y_filter_min)
  (:y_filter_max
   (&optional __y_filter_max)
   (if __y_filter_max (setq _y_filter_max __y_filter_max)) _y_filter_max)
  (:z_filter_min
   (&optional __z_filter_min)
   (if __z_filter_min (setq _z_filter_min __z_filter_min)) _z_filter_min)
  (:z_filter_max
   (&optional __z_filter_max)
   (if __z_filter_max (setq _z_filter_max __z_filter_max)) _z_filter_max)
  (:voxel_leaf_size
   (&optional __voxel_leaf_size)
   (if __voxel_leaf_size (setq _voxel_leaf_size __voxel_leaf_size)) _voxel_leaf_size)
  (:plane_max_iter
   (&optional __plane_max_iter)
   (if __plane_max_iter (setq _plane_max_iter __plane_max_iter)) _plane_max_iter)
  (:plane_dist_thresh
   (&optional __plane_dist_thresh)
   (if __plane_dist_thresh (setq _plane_dist_thresh __plane_dist_thresh)) _plane_dist_thresh)
  (:ror_radius_search
   (&optional __ror_radius_search)
   (if __ror_radius_search (setq _ror_radius_search __ror_radius_search)) _ror_radius_search)
  (:ror_min_neighbors
   (&optional __ror_min_neighbors)
   (if __ror_min_neighbors (setq _ror_min_neighbors __ror_min_neighbors)) _ror_min_neighbors)
  (:cluster_tol
   (&optional __cluster_tol)
   (if __cluster_tol (setq _cluster_tol __cluster_tol)) _cluster_tol)
  (:cluster_min_size
   (&optional __cluster_min_size)
   (if __cluster_min_size (setq _cluster_min_size __cluster_min_size)) _cluster_min_size)
  (:cluster_max_size
   (&optional __cluster_max_size)
   (if __cluster_max_size (setq _cluster_max_size __cluster_max_size)) _cluster_max_size)
  (:serialization-length
   ()
   (+
    ;; float32 _x_filter_min
    4
    ;; float32 _x_filter_max
    4
    ;; float32 _y_filter_min
    4
    ;; float32 _y_filter_max
    4
    ;; float32 _z_filter_min
    4
    ;; float32 _z_filter_max
    4
    ;; float32 _voxel_leaf_size
    4
    ;; int32 _plane_max_iter
    4
    ;; float32 _plane_dist_thresh
    4
    ;; float32 _ror_radius_search
    4
    ;; int32 _ror_min_neighbors
    4
    ;; float32 _cluster_tol
    4
    ;; int32 _cluster_min_size
    4
    ;; int32 _cluster_max_size
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x_filter_min
       (sys::poke _x_filter_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x_filter_max
       (sys::poke _x_filter_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_filter_min
       (sys::poke _y_filter_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_filter_max
       (sys::poke _y_filter_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_filter_min
       (sys::poke _z_filter_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_filter_max
       (sys::poke _z_filter_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _voxel_leaf_size
       (sys::poke _voxel_leaf_size (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _plane_max_iter
       (write-long _plane_max_iter s)
     ;; float32 _plane_dist_thresh
       (sys::poke _plane_dist_thresh (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ror_radius_search
       (sys::poke _ror_radius_search (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _ror_min_neighbors
       (write-long _ror_min_neighbors s)
     ;; float32 _cluster_tol
       (sys::poke _cluster_tol (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _cluster_min_size
       (write-long _cluster_min_size s)
     ;; int32 _cluster_max_size
       (write-long _cluster_max_size s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x_filter_min
     (setq _x_filter_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x_filter_max
     (setq _x_filter_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_filter_min
     (setq _y_filter_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_filter_max
     (setq _y_filter_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_filter_min
     (setq _z_filter_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_filter_max
     (setq _z_filter_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _voxel_leaf_size
     (setq _voxel_leaf_size (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _plane_max_iter
     (setq _plane_max_iter (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _plane_dist_thresh
     (setq _plane_dist_thresh (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ror_radius_search
     (setq _ror_radius_search (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _ror_min_neighbors
     (setq _ror_min_neighbors (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _cluster_tol
     (setq _cluster_tol (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _cluster_min_size
     (setq _cluster_min_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _cluster_max_size
     (setq _cluster_max_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass interbotix_perception_modules::FilterParamsResponse
  :super ros::object
  :slots ())

(defmethod interbotix_perception_modules::FilterParamsResponse
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

(defclass interbotix_perception_modules::FilterParams
  :super ros::object
  :slots ())

(setf (get interbotix_perception_modules::FilterParams :md5sum-) "57d51b939efff834ad5e55a181a7a0a0")
(setf (get interbotix_perception_modules::FilterParams :datatype-) "interbotix_perception_modules/FilterParams")
(setf (get interbotix_perception_modules::FilterParams :request) interbotix_perception_modules::FilterParamsRequest)
(setf (get interbotix_perception_modules::FilterParams :response) interbotix_perception_modules::FilterParamsResponse)

(defmethod interbotix_perception_modules::FilterParamsRequest
  (:response () (instance interbotix_perception_modules::FilterParamsResponse :init)))

(setf (get interbotix_perception_modules::FilterParamsRequest :md5sum-) "57d51b939efff834ad5e55a181a7a0a0")
(setf (get interbotix_perception_modules::FilterParamsRequest :datatype-) "interbotix_perception_modules/FilterParamsRequest")
(setf (get interbotix_perception_modules::FilterParamsRequest :definition-)
      "# This service is used specifically in the interbotix_perception_modules package
#
# Parameters used to tune the various PCL Pointcloud filters

float32 x_filter_min            # minimum value [m] along the x axis beyond which all points will be cropped when doing Crop Box Filtering
float32 x_filter_max            # maximum value [m] along the x axis beyond which all points will be cropped when doing Crop Box Filtering
float32 y_filter_min            # minimum value [m] along the y axis beyond which all points will be cropped when doing Crop Box Filtering
float32 y_filter_max            # maximum value [m] along the y axis beyond which all points will be cropped when doing Crop Box Filtering
float32 z_filter_min            # minimum value [m] along the z axis beyond which all points will be cropped when doing Crop Box Filtering
float32 z_filter_max            # maximum value [m] along the z axis beyond which all points will be cropped when doing Crop Box Filtering
float32 voxel_leaf_size         # voxel leaf size [m] for the x, y, and z axes when doing Voxel Filtering
int32 plane_max_iter            # maximum number of iterations the algorithm should go when when doing Planar Segmentation
float32 plane_dist_thresh       # distance [m] perpendicular from the planar model outside of which points should not be segmented out when doing Planar Segmentation
float32 ror_radius_search       # desired search radius when doing Radius Outlier Removal
int32 ror_min_neighbors         # minimum number of neighbors a point should have to not be cropped out when doing Radius Outlier Removal
float32 cluster_tol             # any point within this distance [m] will be considered part of the same cluster
int32 cluster_min_size          # minimum number of points a cluster must have to be considered a cluster
int32 cluster_max_size          # maximum number of points a cluster can have to be considered a cluster
---

")

(setf (get interbotix_perception_modules::FilterParamsResponse :md5sum-) "57d51b939efff834ad5e55a181a7a0a0")
(setf (get interbotix_perception_modules::FilterParamsResponse :datatype-) "interbotix_perception_modules/FilterParamsResponse")
(setf (get interbotix_perception_modules::FilterParamsResponse :definition-)
      "# This service is used specifically in the interbotix_perception_modules package
#
# Parameters used to tune the various PCL Pointcloud filters

float32 x_filter_min            # minimum value [m] along the x axis beyond which all points will be cropped when doing Crop Box Filtering
float32 x_filter_max            # maximum value [m] along the x axis beyond which all points will be cropped when doing Crop Box Filtering
float32 y_filter_min            # minimum value [m] along the y axis beyond which all points will be cropped when doing Crop Box Filtering
float32 y_filter_max            # maximum value [m] along the y axis beyond which all points will be cropped when doing Crop Box Filtering
float32 z_filter_min            # minimum value [m] along the z axis beyond which all points will be cropped when doing Crop Box Filtering
float32 z_filter_max            # maximum value [m] along the z axis beyond which all points will be cropped when doing Crop Box Filtering
float32 voxel_leaf_size         # voxel leaf size [m] for the x, y, and z axes when doing Voxel Filtering
int32 plane_max_iter            # maximum number of iterations the algorithm should go when when doing Planar Segmentation
float32 plane_dist_thresh       # distance [m] perpendicular from the planar model outside of which points should not be segmented out when doing Planar Segmentation
float32 ror_radius_search       # desired search radius when doing Radius Outlier Removal
int32 ror_min_neighbors         # minimum number of neighbors a point should have to not be cropped out when doing Radius Outlier Removal
float32 cluster_tol             # any point within this distance [m] will be considered part of the same cluster
int32 cluster_min_size          # minimum number of points a cluster must have to be considered a cluster
int32 cluster_max_size          # maximum number of points a cluster can have to be considered a cluster
---

")



(provide :interbotix_perception_modules/FilterParams "57d51b939efff834ad5e55a181a7a0a0")


