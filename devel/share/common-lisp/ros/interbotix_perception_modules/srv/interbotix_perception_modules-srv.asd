
(cl:in-package :asdf)

(defsystem "interbotix_perception_modules-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :interbotix_perception_modules-msg
)
  :components ((:file "_package")
    (:file "ClusterInfoArray" :depends-on ("_package_ClusterInfoArray"))
    (:file "_package_ClusterInfoArray" :depends-on ("_package"))
    (:file "FilterParams" :depends-on ("_package_FilterParams"))
    (:file "_package_FilterParams" :depends-on ("_package"))
    (:file "SnapPicture" :depends-on ("_package_SnapPicture"))
    (:file "_package_SnapPicture" :depends-on ("_package"))
  ))