
(cl:in-package :asdf)

(defsystem "me326_locobot_example-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "PixtoPoint" :depends-on ("_package_PixtoPoint"))
    (:file "_package_PixtoPoint" :depends-on ("_package"))
  ))