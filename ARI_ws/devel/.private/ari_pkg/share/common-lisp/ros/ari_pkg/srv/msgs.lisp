; Auto-generated. Do not edit!


(cl:in-package ari_pkg-srv)


;//! \htmlinclude msgs-request.msg.html

(cl:defclass <msgs-request> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass msgs-request (<msgs-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msgs-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msgs-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ari_pkg-srv:<msgs-request> is deprecated: use ari_pkg-srv:msgs-request instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <msgs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ari_pkg-srv:msg-val is deprecated.  Use ari_pkg-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msgs-request>) ostream)
  "Serializes a message object of type '<msgs-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msgs-request>) istream)
  "Deserializes a message object of type '<msgs-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msgs-request>)))
  "Returns string type for a service object of type '<msgs-request>"
  "ari_pkg/msgsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msgs-request)))
  "Returns string type for a service object of type 'msgs-request"
  "ari_pkg/msgsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msgs-request>)))
  "Returns md5sum for a message object of type '<msgs-request>"
  "2b2d0cfdeb1bcfaa2e895bdcca41ec33")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msgs-request)))
  "Returns md5sum for a message object of type 'msgs-request"
  "2b2d0cfdeb1bcfaa2e895bdcca41ec33")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msgs-request>)))
  "Returns full string definition for message of type '<msgs-request>"
  (cl:format cl:nil "string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msgs-request)))
  "Returns full string definition for message of type 'msgs-request"
  (cl:format cl:nil "string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msgs-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msgs-request>))
  "Converts a ROS message object to a list"
  (cl:list 'msgs-request
    (cl:cons ':msg (msg msg))
))
;//! \htmlinclude msgs-response.msg.html

(cl:defclass <msgs-response> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass msgs-response (<msgs-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msgs-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msgs-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ari_pkg-srv:<msgs-response> is deprecated: use ari_pkg-srv:msgs-response instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <msgs-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ari_pkg-srv:msg-val is deprecated.  Use ari_pkg-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msgs-response>) ostream)
  "Serializes a message object of type '<msgs-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msgs-response>) istream)
  "Deserializes a message object of type '<msgs-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msgs-response>)))
  "Returns string type for a service object of type '<msgs-response>"
  "ari_pkg/msgsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msgs-response)))
  "Returns string type for a service object of type 'msgs-response"
  "ari_pkg/msgsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msgs-response>)))
  "Returns md5sum for a message object of type '<msgs-response>"
  "2b2d0cfdeb1bcfaa2e895bdcca41ec33")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msgs-response)))
  "Returns md5sum for a message object of type 'msgs-response"
  "2b2d0cfdeb1bcfaa2e895bdcca41ec33")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msgs-response>)))
  "Returns full string definition for message of type '<msgs-response>"
  (cl:format cl:nil "string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msgs-response)))
  "Returns full string definition for message of type 'msgs-response"
  (cl:format cl:nil "string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msgs-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msgs-response>))
  "Converts a ROS message object to a list"
  (cl:list 'msgs-response
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'msgs)))
  'msgs-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'msgs)))
  'msgs-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msgs)))
  "Returns string type for a service object of type '<msgs>"
  "ari_pkg/msgs")