;; Auto-generated. Do not edit!


(when (boundp 'interbotix_perception_modules::ClusterInfo)
  (if (not (find-package "INTERBOTIX_PERCEPTION_MODULES"))
    (make-package "INTERBOTIX_PERCEPTION_MODULES"))
  (shadow 'ClusterInfo (find-package "INTERBOTIX_PERCEPTION_MODULES")))
(unless (find-package "INTERBOTIX_PERCEPTION_MODULES::CLUSTERINFO")
  (make-package "INTERBOTIX_PERCEPTION_MODULES::CLUSTERINFO"))

(in-package "ROS")
;;//! \htmlinclude ClusterInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass interbotix_perception_modules::ClusterInfo
  :super ros::object
  :slots (_frame_id _position _yaw _color _min_z_point _num_points ))

(defmethod interbotix_perception_modules::ClusterInfo
  (:init
   (&key
    ((:frame_id __frame_id) "")
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:yaw __yaw) 0.0)
    ((:color __color) (instance std_msgs::ColorRGBA :init))
    ((:min_z_point __min_z_point) (instance geometry_msgs::Point :init))
    ((:num_points __num_points) 0)
    )
   (send-super :init)
   (setq _frame_id (string __frame_id))
   (setq _position __position)
   (setq _yaw (float __yaw))
   (setq _color __color)
   (setq _min_z_point __min_z_point)
   (setq _num_points (round __num_points))
   self)
  (:frame_id
   (&optional __frame_id)
   (if __frame_id (setq _frame_id __frame_id)) _frame_id)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:color
   (&rest __color)
   (if (keywordp (car __color))
       (send* _color __color)
     (progn
       (if __color (setq _color (car __color)))
       _color)))
  (:min_z_point
   (&rest __min_z_point)
   (if (keywordp (car __min_z_point))
       (send* _min_z_point __min_z_point)
     (progn
       (if __min_z_point (setq _min_z_point (car __min_z_point)))
       _min_z_point)))
  (:num_points
   (&optional __num_points)
   (if __num_points (setq _num_points __num_points)) _num_points)
  (:serialization-length
   ()
   (+
    ;; string _frame_id
    4 (length _frame_id)
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; float32 _yaw
    4
    ;; std_msgs/ColorRGBA _color
    (send _color :serialization-length)
    ;; geometry_msgs/Point _min_z_point
    (send _min_z_point :serialization-length)
    ;; int32 _num_points
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _frame_id
       (write-long (length _frame_id) s) (princ _frame_id s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; std_msgs/ColorRGBA _color
       (send _color :serialize s)
     ;; geometry_msgs/Point _min_z_point
       (send _min_z_point :serialize s)
     ;; int32 _num_points
       (write-long _num_points s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; std_msgs/ColorRGBA _color
     (send _color :deserialize buf ptr-) (incf ptr- (send _color :serialization-length))
   ;; geometry_msgs/Point _min_z_point
     (send _min_z_point :deserialize buf ptr-) (incf ptr- (send _min_z_point :serialization-length))
   ;; int32 _num_points
     (setq _num_points (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get interbotix_perception_modules::ClusterInfo :md5sum-) "9627d2d019ccc4ed2077673b4b0f3b40")
(setf (get interbotix_perception_modules::ClusterInfo :datatype-) "interbotix_perception_modules/ClusterInfo")
(setf (get interbotix_perception_modules::ClusterInfo :definition-)
      "# This message is used specifically in the interbotix_perception_modules package
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



(provide :interbotix_perception_modules/ClusterInfo "9627d2d019ccc4ed2077673b4b0f3b40")


