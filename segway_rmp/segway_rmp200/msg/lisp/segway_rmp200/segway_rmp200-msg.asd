
(in-package :asdf)

(defsystem "segway_rmp200-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "SegwayStatus" :depends-on ("_package"))
    (:file "_package_SegwayStatus" :depends-on ("_package"))
    ))
