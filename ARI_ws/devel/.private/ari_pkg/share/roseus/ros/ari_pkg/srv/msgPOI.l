;; Auto-generated. Do not edit!


(when (boundp 'ari_pkg::msgPOI)
  (if (not (find-package "ARI_PKG"))
    (make-package "ARI_PKG"))
  (shadow 'msgPOI (find-package "ARI_PKG")))
(unless (find-package "ARI_PKG::MSGPOI")
  (make-package "ARI_PKG::MSGPOI"))
(unless (find-package "ARI_PKG::MSGPOIREQUEST")
  (make-package "ARI_PKG::MSGPOIREQUEST"))
(unless (find-package "ARI_PKG::MSGPOIRESPONSE")
  (make-package "ARI_PKG::MSGPOIRESPONSE"))

(in-package "ROS")





(defclass ari_pkg::msgPOIRequest
  :super ros::object
  :slots (_poi_names ))

(defmethod ari_pkg::msgPOIRequest
  (:init
   (&key
    ((:poi_names __poi_names) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _poi_names __poi_names)
   self)
  (:poi_names
   (&optional __poi_names)
   (if __poi_names (setq _poi_names __poi_names)) _poi_names)
  (:serialization-length
   ()
   (+
    ;; string[] _poi_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _poi_names)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _poi_names
     (write-long (length _poi_names) s)
     (dolist (elem _poi_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _poi_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poi_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _poi_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass ari_pkg::msgPOIResponse
  :super ros::object
  :slots (_found _poi_name_out ))

(defmethod ari_pkg::msgPOIResponse
  (:init
   (&key
    ((:found __found) nil)
    ((:poi_name_out __poi_name_out) "")
    )
   (send-super :init)
   (setq _found __found)
   (setq _poi_name_out (string __poi_name_out))
   self)
  (:found
   (&optional __found)
   (if __found (setq _found __found)) _found)
  (:poi_name_out
   (&optional __poi_name_out)
   (if __poi_name_out (setq _poi_name_out __poi_name_out)) _poi_name_out)
  (:serialization-length
   ()
   (+
    ;; bool _found
    1
    ;; string _poi_name_out
    4 (length _poi_name_out)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _found
       (if _found (write-byte -1 s) (write-byte 0 s))
     ;; string _poi_name_out
       (write-long (length _poi_name_out) s) (princ _poi_name_out s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _found
     (setq _found (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _poi_name_out
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _poi_name_out (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ari_pkg::msgPOI
  :super ros::object
  :slots ())

(setf (get ari_pkg::msgPOI :md5sum-) "c796f839409775c93131126db79153f5")
(setf (get ari_pkg::msgPOI :datatype-) "ari_pkg/msgPOI")
(setf (get ari_pkg::msgPOI :request) ari_pkg::msgPOIRequest)
(setf (get ari_pkg::msgPOI :response) ari_pkg::msgPOIResponse)

(defmethod ari_pkg::msgPOIRequest
  (:response () (instance ari_pkg::msgPOIResponse :init)))

(setf (get ari_pkg::msgPOIRequest :md5sum-) "c796f839409775c93131126db79153f5")
(setf (get ari_pkg::msgPOIRequest :datatype-) "ari_pkg/msgPOIRequest")
(setf (get ari_pkg::msgPOIRequest :definition-)
      "string[] poi_names
---
bool found
string poi_name_out
")

(setf (get ari_pkg::msgPOIResponse :md5sum-) "c796f839409775c93131126db79153f5")
(setf (get ari_pkg::msgPOIResponse :datatype-) "ari_pkg/msgPOIResponse")
(setf (get ari_pkg::msgPOIResponse :definition-)
      "string[] poi_names
---
bool found
string poi_name_out
")



(provide :ari_pkg/msgPOI "c796f839409775c93131126db79153f5")


