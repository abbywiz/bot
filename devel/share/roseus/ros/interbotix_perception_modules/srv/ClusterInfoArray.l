;; Auto-generated. Do not edit!


(when (boundp 'interbotix_perception_modules::ClusterInfoArray)
  (if (not (find-package "INTERBOTIX_PERCEPTION_MODULES"))
    (make-package "INTERBOTIX_PERCEPTION_MODULES"))
  (shadow 'ClusterInfoArray (find-package "INTERBOTIX_PERCEPTION_MODULES")))
(unless (find-package "INTERBOTIX_PERCEPTION_MODULES::CLUSTERINFOARRAY")
  (make-package "INTERBOTIX_PERCEPTION_MODULES::CLUSTERINFOARRAY"))
(unless (find-package "INTERBOTIX_PERCEPTION_MODULES::CLUSTERINFOARRAYREQUEST")
  (make-package "INTERBOTIX_PERCEPTION_MODULES::CLUSTERINFOARRAYREQUEST"))
(unless (find-package "INTERBOTIX_PERCEPTION_MODULES::CLUSTERINFOARRAYRESPONSE")
  (make-package "INTERBOTIX_PERCEPTION_MODULES::CLUSTERINFOARRAYRESPONSE"))

(in-package "ROS")





(defclass interbotix_perception_modules::ClusterInfoArrayRequest
  :super ros::object
  :slots ())

(defmethod interbotix_perception_modules::ClusterInfoArrayRequest
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

(defclass interbotix_perception_modules::ClusterInfoArrayResponse
  :super ros::object
  :slots (_clusters ))

(defmethod interbotix_perception_modules::ClusterInfoArrayResponse
  (:init
   (&key
    ((:clusters __clusters) ())
    )
   (send-super :init)
   (setq _clusters __clusters)
   self)
  (:clusters
   (&rest __clusters)
   (if (keywordp (car __clusters))
       (send* _clusters __clusters)
     (progn
       (if __clusters (setq _clusters (car __clusters)))
       _clusters)))
  (:serialization-length
   ()
   (+
    ;; interbotix_perception_modules/ClusterInfo[] _clusters
    (apply #'+ (send-all _clusters :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; interbotix_perception_modules/ClusterInfo[] _clusters
     (write-long (length _clusters) s)
     (dolist (elem _clusters)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; interbotix_perception_modules/ClusterInfo[] _clusters
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _clusters (let (r) (dotimes (i n) (push (instance interbotix_perception_modules::ClusterInfo :init) r)) r))
     (dolist (elem- _clusters)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass interbotix_perception_modules::ClusterInfoArray
  :super ros::object
  :slots ())

(setf (get interbotix_perception_modules::ClusterInfoArray :md5sum-) "911bd211bd6fd7e568c4562cf40f28fd")
(setf (get interbotix_perception_modules::ClusterInfoArray :datatype-) "interbotix_perception_modules/ClusterInfoArray")
(setf (get interbotix_perception_modules::ClusterInfoArray :request) interbotix_perception_modules::ClusterInfoArrayRequest)
(setf (get interbotix_perception_modules::ClusterInfoArray :response) interbotix_perception_modules::ClusterInfoArrayResponse)

(defmethod interbotix_perception_modules::ClusterInfoArrayRequest
  (:response () (instance interbotix_perception_modules::ClusterInfoArrayResponse :init)))

(setf (get interbotix_perception_modules::ClusterInfoArrayRequest :md5sum-) "911bd211bd6fd7e568c4562cf40f28fd")
(setf (get interbotix_perception_modules::ClusterInfoArrayRequest :datatype-) "interbotix_perception_modules/ClusterInfoArrayRequest")
(setf (get interbotix_perception_modules::ClusterInfoArrayRequest :definition-)
      "# This service is used specifically in the interbotix_perception_modules package
#
# List of ClusterInfo messages. The length of the list should correspond to the
# number of objects seen by the camera

---
interbotix_perception_modules/ClusterInfo[] clusters


================================================================================
MSG: interbotix_perception_modules/ClusterInfo
# This message is used specifically in the interbotix_perception_modules package
#
# Message that holds information about each cluster

string frame_id                     # parent frame of the cluster (usually 'camera_depth_optical_frame' or similar)
geometry_msgs/Point position        # x, y, z position of the cluster
float32 yaw                         # yaw [rad] of the cluster; x-axis of the cluster should align
                                    # with the major-axis of a best-fit ellipse (currently not supported)
std_msgs/ColorRGBA color            # average RGB values (0 - 255) for the whole cluster
geometry_msgs/Point min_z_point     # point with the min 'z' value of the cluster
int32 num_points                    # number of points in the cluster

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
")

(setf (get interbotix_perception_modules::ClusterInfoArrayResponse :md5sum-) "911bd211bd6fd7e568c4562cf40f28fd")
(setf (get interbotix_perception_modules::ClusterInfoArrayResponse :datatype-) "interbotix_perception_modules/ClusterInfoArrayResponse")
(setf (get interbotix_perception_modules::ClusterInfoArrayResponse :definition-)
      "# This service is used specifically in the interbotix_perception_modules package
#
# List of ClusterInfo messages. The length of the list should correspond to the
# number of objects seen by the camera

---
interbotix_perception_modules/ClusterInfo[] clusters


================================================================================
MSG: interbotix_perception_modules/ClusterInfo
# This message is used specifically in the interbotix_perception_modules package
#
# Message that holds information about each cluster

string frame_id                     # parent frame of the cluster (usually 'camera_depth_optical_frame' or similar)
geometry_msgs/Point position        # x, y, z position of the cluster
float32 yaw                         # yaw [rad] of the cluster; x-axis of the cluster should align
                                    # with the major-axis of a best-fit ellipse (currently not supported)
std_msgs/ColorRGBA color            # average RGB values (0 - 255) for the whole cluster
geometry_msgs/Point min_z_point     # point with the min 'z' value of the cluster
int32 num_points                    # number of points in the cluster

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
")



(provide :interbotix_perception_modules/ClusterInfoArray "911bd211bd6fd7e568c4562cf40f28fd")


