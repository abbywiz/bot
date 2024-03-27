
(cl:in-package :asdf)

(defsystem "interbotix_perception_modules-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ClusterInfo" :depends-on ("_package_ClusterInfo"))
    (:file "_package_ClusterInfo" :depends-on ("_package"))
  ))