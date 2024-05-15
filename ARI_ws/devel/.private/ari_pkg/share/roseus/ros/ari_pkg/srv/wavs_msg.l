;; Auto-generated. Do not edit!


(when (boundp 'ari_pkg::wavs_msg)
  (if (not (find-package "ARI_PKG"))
    (make-package "ARI_PKG"))
  (shadow 'wavs_msg (find-package "ARI_PKG")))
(unless (find-package "ARI_PKG::WAVS_MSG")
  (make-package "ARI_PKG::WAVS_MSG"))
(unless (find-package "ARI_PKG::WAVS_MSGREQUEST")
  (make-package "ARI_PKG::WAVS_MSGREQUEST"))
(unless (find-package "ARI_PKG::WAVS_MSGRESPONSE")
  (make-package "ARI_PKG::WAVS_MSGRESPONSE"))

(in-package "ROS")





(defclass ari_pkg::wavs_msgRequest
  :super ros::object
  :slots (_fileName _text ))

(defmethod ari_pkg::wavs_msgRequest
  (:init
   (&key
    ((:fileName __fileName) "")
    ((:text __text) "")
    )
   (send-super :init)
   (setq _fileName (string __fileName))
   (setq _text (string __text))
   self)
  (:fileName
   (&optional __fileName)
   (if __fileName (setq _fileName __fileName)) _fileName)
  (:text
   (&optional __text)
   (if __text (setq _text __text)) _text)
  (:serialization-length
   ()
   (+
    ;; string _fileName
    4 (length _fileName)
    ;; string _text
    4 (length _text)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _fileName
       (write-long (length _fileName) s) (princ _fileName s)
     ;; string _text
       (write-long (length _text) s) (princ _text s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _fileName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fileName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ari_pkg::wavs_msgResponse
  :super ros::object
  :slots (_msgResp ))

(defmethod ari_pkg::wavs_msgResponse
  (:init
   (&key
    ((:msgResp __msgResp) "")
    )
   (send-super :init)
   (setq _msgResp (string __msgResp))
   self)
  (:msgResp
   (&optional __msgResp)
   (if __msgResp (setq _msgResp __msgResp)) _msgResp)
  (:serialization-length
   ()
   (+
    ;; string _msgResp
    4 (length _msgResp)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _msgResp
       (write-long (length _msgResp) s) (princ _msgResp s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _msgResp
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msgResp (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ari_pkg::wavs_msg
  :super ros::object
  :slots ())

(setf (get ari_pkg::wavs_msg :md5sum-) "e6a881b942915d9139f8909f61208275")
(setf (get ari_pkg::wavs_msg :datatype-) "ari_pkg/wavs_msg")
(setf (get ari_pkg::wavs_msg :request) ari_pkg::wavs_msgRequest)
(setf (get ari_pkg::wavs_msg :response) ari_pkg::wavs_msgResponse)

(defmethod ari_pkg::wavs_msgRequest
  (:response () (instance ari_pkg::wavs_msgResponse :init)))

(setf (get ari_pkg::wavs_msgRequest :md5sum-) "e6a881b942915d9139f8909f61208275")
(setf (get ari_pkg::wavs_msgRequest :datatype-) "ari_pkg/wavs_msgRequest")
(setf (get ari_pkg::wavs_msgRequest :definition-)
      "string fileName
string text
---
string msgResp
")

(setf (get ari_pkg::wavs_msgResponse :md5sum-) "e6a881b942915d9139f8909f61208275")
(setf (get ari_pkg::wavs_msgResponse :datatype-) "ari_pkg/wavs_msgResponse")
(setf (get ari_pkg::wavs_msgResponse :definition-)
      "string fileName
string text
---
string msgResp
")



(provide :ari_pkg/wavs_msg "e6a881b942915d9139f8909f61208275")


