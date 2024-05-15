
(cl:in-package :asdf)

(defsystem "ari_pkg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msgPOI" :depends-on ("_package_msgPOI"))
    (:file "_package_msgPOI" :depends-on ("_package"))
    (:file "msgs" :depends-on ("_package_msgs"))
    (:file "_package_msgs" :depends-on ("_package"))
    (:file "wavs_msg" :depends-on ("_package_wavs_msg"))
    (:file "_package_wavs_msg" :depends-on ("_package"))
  ))