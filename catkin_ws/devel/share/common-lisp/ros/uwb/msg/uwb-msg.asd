
(cl:in-package :asdf)

(defsystem "uwb-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "UWBMultiRange" :depends-on ("_package_UWBMultiRange"))
    (:file "_package_UWBMultiRange" :depends-on ("_package"))
    (:file "UWBMultiRangeRaw" :depends-on ("_package_UWBMultiRangeRaw"))
    (:file "_package_UWBMultiRangeRaw" :depends-on ("_package"))
    (:file "UWBRange" :depends-on ("_package_UWBRange"))
    (:file "_package_UWBRange" :depends-on ("_package"))
    (:file "UWBRangeStats" :depends-on ("_package_UWBRangeStats"))
    (:file "_package_UWBRangeStats" :depends-on ("_package"))
    (:file "UWBRangeStatsSimple" :depends-on ("_package_UWBRangeStatsSimple"))
    (:file "_package_UWBRangeStatsSimple" :depends-on ("_package"))
    (:file "UWBTracker" :depends-on ("_package_UWBTracker"))
    (:file "_package_UWBTracker" :depends-on ("_package"))
    (:file "UWBTrackerRaw" :depends-on ("_package_UWBTrackerRaw"))
    (:file "_package_UWBTrackerRaw" :depends-on ("_package"))
  ))