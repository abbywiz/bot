;; Auto-generated. Do not edit!


(when (boundp 'interbotix_perception_modules::SnapPicture)
  (if (not (find-package "INTERBOTIX_PERCEPTION_MODULES"))
    (make-package "INTERBOTIX_PERCEPTION_MODULES"))
  (shadow 'SnapPicture (find-package "INTERBOTIX_PERCEPTION_MODULES")))
(unless (find-package "INTERBOTIX_PERCEPTION_MODULES::SNAPPICTURE")
  (make-package "INTERBOTIX_PERCEPTION_MODULES::SNAPPICTURE"))
(unless (find-package "INTERBOTIX_PERCEPTION_MODULES::SNAPPICTUREREQUEST")
  (make-package "INTERBOTIX_PERCEPTION_MODULES::SNAPPICTUREREQUEST"))
(unless (find-package "INTERBOTIX_PERCEPTION_MODULES::SNAPPICTURERESPONSE")
  (make-package "INTERBOTIX_PERCEPTION_MODULES::SNAPPICTURERESPONSE"))

(in-package "ROS")





(defclass interbotix_perception_modules::SnapPictureRequest
  :super ros::object
  :slots (_filename ))

(defmethod interbotix_perception_modules::SnapPictureRequest
  (:init
   (&key
    ((:filename __filename) "")
    )
   (send-super :init)
   (setq _filename (string __filename))
   self)
  (:filename
   (&optional __filename)
   (if __filename (setq _filename __filename)) _filename)
  (:serialization-length
   ()
   (+
    ;; string _filename
    4 (length _filename)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _filename
       (write-long (length _filename) s) (princ _filename s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _filename
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _filename (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass interbotix_perception_modules::SnapPictureResponse
  :super ros::object
  :slots (_success _filepath ))

(defmethod interbotix_perception_modules::SnapPictureResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:filepath __filepath) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _filepath (string __filepath))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:filepath
   (&optional __filepath)
   (if __filepath (setq _filepath __filepath)) _filepath)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _filepath
    4 (length _filepath)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _filepath
       (write-long (length _filepath) s) (princ _filepath s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _filepath
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _filepath (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass interbotix_perception_modules::SnapPicture
  :super ros::object
  :slots ())

(setf (get interbotix_perception_modules::SnapPicture :md5sum-) "a8777594bfb7f85102a1426edcb42742")
(setf (get interbotix_perception_modules::SnapPicture :datatype-) "interbotix_perception_modules/SnapPicture")
(setf (get interbotix_perception_modules::SnapPicture :request) interbotix_perception_modules::SnapPictureRequest)
(setf (get interbotix_perception_modules::SnapPicture :response) interbotix_perception_modules::SnapPictureResponse)

(defmethod interbotix_perception_modules::SnapPictureRequest
  (:response () (instance interbotix_perception_modules::SnapPictureResponse :init)))

(setf (get interbotix_perception_modules::SnapPictureRequest :md5sum-) "a8777594bfb7f85102a1426edcb42742")
(setf (get interbotix_perception_modules::SnapPictureRequest :datatype-) "interbotix_perception_modules/SnapPictureRequest")
(setf (get interbotix_perception_modules::SnapPictureRequest :definition-)
      "# This service is used specifically in the interbotix_perception_modules package
#
# Save the latest rgb picture with the specified name
#
# Request consists of:
#   filename : name of file in which to save image including extension (.jpg)
#
# Response consists of:
#    success : boolean indication of service success

string filename
---
bool success
string filepath
")

(setf (get interbotix_perception_modules::SnapPictureResponse :md5sum-) "a8777594bfb7f85102a1426edcb42742")
(setf (get interbotix_perception_modules::SnapPictureResponse :datatype-) "interbotix_perception_modules/SnapPictureResponse")
(setf (get interbotix_perception_modules::SnapPictureResponse :definition-)
      "# This service is used specifically in the interbotix_perception_modules package
#
# Save the latest rgb picture with the specified name
#
# Request consists of:
#   filename : name of file in which to save image including extension (.jpg)
#
# Response consists of:
#    success : boolean indication of service success

string filename
---
bool success
string filepath
")



(provide :interbotix_perception_modules/SnapPicture "a8777594bfb7f85102a1426edcb42742")


