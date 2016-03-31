
(cl:in-package :asdf)

(defsystem "uwb_dummy-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Coordinates" :depends-on ("_package_Coordinates"))
    (:file "_package_Coordinates" :depends-on ("_package"))
  ))