;; Auto-generated. Do not edit!


(when (boundp 'ari_pkg::msgs)
  (if (not (find-package "ARI_PKG"))
    (make-package "ARI_PKG"))
  (shadow 'msgs (find-package "ARI_PKG")))
(unless (find-package "ARI_PKG::MSGS")
  (make-package "ARI_PKG::MSGS"))
(unless (find-package "ARI_PKG::MSGSREQUEST")
  (make-package "ARI_PKG::MSGSREQUEST"))
(unless (find-package "ARI_PKG::MSGSRESPONSE")
  (make-package "ARI_PKG::MSGSRESPONSE"))

(in-package "ROS")





(defclass ari_pkg::msgsRequest
  :super ros::object
  :slots (_msg ))

(defmethod ari_pkg::msgsRequest
  (:init
   (&key
    ((:msg __msg) "")
    )
   (send-super :init)
   (setq _msg (string __msg))
   self)
  (:msg
   (&optional __msg)
   (if __msg (setq _msg __msg)) _msg)
  (:serialization-length
   ()
   (+
    ;; string _msg
    4 (length _msg)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _msg
       (write-long (length _msg) s) (princ _msg s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ari_pkg::msgsResponse
  :super ros::object
  :slots (_msg ))

(defmethod ari_pkg::msgsResponse
  (:init
   (&key
    ((:msg __msg) "")
    )
   (send-super :init)
   (setq _msg (string __msg))
   self)
  (:msg
   (&optional __msg)
   (if __msg (setq _msg __msg)) _msg)
  (:serialization-length
   ()
   (+
    ;; string _msg
    4 (length _msg)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _msg
       (write-long (length _msg) s) (princ _msg s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ari_pkg::msgs
  :super ros::object
  :slots ())

(setf (get ari_pkg::msgs :md5sum-) "2b2d0cfdeb1bcfaa2e895bdcca41ec33")
(setf (get ari_pkg::msgs :datatype-) "ari_pkg/msgs")
(setf (get ari_pkg::msgs :request) ari_pkg::msgsRequest)
(setf (get ari_pkg::msgs :response) ari_pkg::msgsResponse)

(defmethod ari_pkg::msgsRequest
  (:response () (instance ari_pkg::msgsResponse :init)))

(setf (get ari_pkg::msgsRequest :md5sum-) "2b2d0cfdeb1bcfaa2e895bdcca41ec33")
(setf (get ari_pkg::msgsRequest :datatype-) "ari_pkg/msgsRequest")
(setf (get ari_pkg::msgsRequest :definition-)
      "string msg
---
string msg

")

(setf (get ari_pkg::msgsResponse :md5sum-) "2b2d0cfdeb1bcfaa2e895bdcca41ec33")
(setf (get ari_pkg::msgsResponse :datatype-) "ari_pkg/msgsResponse")
(setf (get ari_pkg::msgsResponse :definition-)
      "string msg
---
string msg

")



(provide :ari_pkg/msgs "2b2d0cfdeb1bcfaa2e895bdcca41ec33")


