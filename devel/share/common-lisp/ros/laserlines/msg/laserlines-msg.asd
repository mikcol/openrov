
(cl:in-package :asdf)

(defsystem "laserlines-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "LaserMsg" :depends-on ("_package_LaserMsg"))
    (:file "_package_LaserMsg" :depends-on ("_package"))
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
  ))