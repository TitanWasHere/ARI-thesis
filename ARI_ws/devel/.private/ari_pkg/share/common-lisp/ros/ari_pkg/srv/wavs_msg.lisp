; Auto-generated. Do not edit!


(cl:in-package ari_pkg-srv)


;//! \htmlinclude wavs_msg-request.msg.html

(cl:defclass <wavs_msg-request> (roslisp-msg-protocol:ros-message)
  ((fileName
    :reader fileName
    :initarg :fileName
    :type cl:string
    :initform "")
   (text
    :reader text
    :initarg :text
    :type cl:string
    :initform ""))
)

(cl:defclass wavs_msg-request (<wavs_msg-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <wavs_msg-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'wavs_msg-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ari_pkg-srv:<wavs_msg-request> is deprecated: use ari_pkg-srv:wavs_msg-request instead.")))

(cl:ensure-generic-function 'fileName-val :lambda-list '(m))
(cl:defmethod fileName-val ((m <wavs_msg-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ari_pkg-srv:fileName-val is deprecated.  Use ari_pkg-srv:fileName instead.")
  (fileName m))

(cl:ensure-generic-function 'text-val :lambda-list '(m))
(cl:defmethod text-val ((m <wavs_msg-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ari_pkg-srv:text-val is deprecated.  Use ari_pkg-srv:text instead.")
  (text m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <wavs_msg-request>) ostream)
  "Serializes a message object of type '<wavs_msg-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'fileName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'fileName))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'text))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <wavs_msg-request>) istream)
  "Deserializes a message object of type '<wavs_msg-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fileName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'fileName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<wavs_msg-request>)))
  "Returns string type for a service object of type '<wavs_msg-request>"
  "ari_pkg/wavs_msgRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'wavs_msg-request)))
  "Returns string type for a service object of type 'wavs_msg-request"
  "ari_pkg/wavs_msgRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<wavs_msg-request>)))
  "Returns md5sum for a message object of type '<wavs_msg-request>"
  "e6a881b942915d9139f8909f61208275")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'wavs_msg-request)))
  "Returns md5sum for a message object of type 'wavs_msg-request"
  "e6a881b942915d9139f8909f61208275")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<wavs_msg-request>)))
  "Returns full string definition for message of type '<wavs_msg-request>"
  (cl:format cl:nil "string fileName~%string text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'wavs_msg-request)))
  "Returns full string definition for message of type 'wavs_msg-request"
  (cl:format cl:nil "string fileName~%string text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <wavs_msg-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'fileName))
     4 (cl:length (cl:slot-value msg 'text))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <wavs_msg-request>))
  "Converts a ROS message object to a list"
  (cl:list 'wavs_msg-request
    (cl:cons ':fileName (fileName msg))
    (cl:cons ':text (text msg))
))
;//! \htmlinclude wavs_msg-response.msg.html

(cl:defclass <wavs_msg-response> (roslisp-msg-protocol:ros-message)
  ((msgResp
    :reader msgResp
    :initarg :msgResp
    :type cl:string
    :initform ""))
)

(cl:defclass wavs_msg-response (<wavs_msg-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <wavs_msg-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'wavs_msg-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ari_pkg-srv:<wavs_msg-response> is deprecated: use ari_pkg-srv:wavs_msg-response instead.")))

(cl:ensure-generic-function 'msgResp-val :lambda-list '(m))
(cl:defmethod msgResp-val ((m <wavs_msg-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ari_pkg-srv:msgResp-val is deprecated.  Use ari_pkg-srv:msgResp instead.")
  (msgResp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <wavs_msg-response>) ostream)
  "Serializes a message object of type '<wavs_msg-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msgResp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msgResp))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <wavs_msg-response>) istream)
  "Deserializes a message object of type '<wavs_msg-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msgResp) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msgResp) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<wavs_msg-response>)))
  "Returns string type for a service object of type '<wavs_msg-response>"
  "ari_pkg/wavs_msgResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'wavs_msg-response)))
  "Returns string type for a service object of type 'wavs_msg-response"
  "ari_pkg/wavs_msgResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<wavs_msg-response>)))
  "Returns md5sum for a message object of type '<wavs_msg-response>"
  "e6a881b942915d9139f8909f61208275")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'wavs_msg-response)))
  "Returns md5sum for a message object of type 'wavs_msg-response"
  "e6a881b942915d9139f8909f61208275")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<wavs_msg-response>)))
  "Returns full string definition for message of type '<wavs_msg-response>"
  (cl:format cl:nil "string msgResp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'wavs_msg-response)))
  "Returns full string definition for message of type 'wavs_msg-response"
  (cl:format cl:nil "string msgResp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <wavs_msg-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msgResp))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <wavs_msg-response>))
  "Converts a ROS message object to a list"
  (cl:list 'wavs_msg-response
    (cl:cons ':msgResp (msgResp msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'wavs_msg)))
  'wavs_msg-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'wavs_msg)))
  'wavs_msg-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'wavs_msg)))
  "Returns string type for a service object of type '<wavs_msg>"
  "ari_pkg/wavs_msg")