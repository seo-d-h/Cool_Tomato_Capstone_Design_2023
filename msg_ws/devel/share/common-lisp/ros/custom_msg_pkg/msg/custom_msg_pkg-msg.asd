
(cl:in-package :asdf)

(defsystem "custom_msg_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Coordinate" :depends-on ("_package_Coordinate"))
    (:file "_package_Coordinate" :depends-on ("_package"))
  ))