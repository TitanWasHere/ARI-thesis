; Auto-generated. Do not edit!


(cl:in-package ari_pkg-srv)


;//! \htmlinclude msgPOI-request.msg.html

(cl:defclass <msgPOI-request> (roslisp-msg-protocol:ros-message)
  ((poi_names
    :reader poi_names
    :initarg :poi_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass msgPOI-request (<msgPOI-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msgPOI-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msgPOI-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ari_pkg-srv:<msgPOI-request> is deprecated: use ari_pkg-srv:msgPOI-request instead.")))

(cl:ensure-generic-function 'poi_names-val :lambda-list '(m))
(cl:defmethod poi_names-val ((m <msgPOI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ari_pkg-srv:poi_names-val is deprecated.  Use ari_pkg-srv:poi_names instead.")
  (poi_names m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msgPOI-request>) ostream)
  "Serializes a message object of type '<msgPOI-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poi_names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'poi_names))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msgPOI-request>) istream)
  "Deserializes a message object of type '<msgPOI-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poi_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poi_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msgPOI-request>)))
  "Returns string type for a service object of type '<msgPOI-request>"
  "ari_pkg/msgPOIRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msgPOI-request)))
  "Returns string type for a service object of type 'msgPOI-request"
  "ari_pkg/msgPOIRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msgPOI-request>)))
  "Returns md5sum for a message object of type '<msgPOI-request>"
  "c796f839409775c93131126db79153f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msgPOI-request)))
  "Returns md5sum for a message object of type 'msgPOI-request"
  "c796f839409775c93131126db79153f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msgPOI-request>)))
  "Returns full string definition for message of type '<msgPOI-request>"
  (cl:format cl:nil "string[] poi_names~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msgPOI-request)))
  "Returns full string definition for message of type 'msgPOI-request"
  (cl:format cl:nil "string[] poi_names~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msgPOI-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poi_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msgPOI-request>))
  "Converts a ROS message object to a list"
  (cl:list 'msgPOI-request
    (cl:cons ':poi_names (poi_names msg))
))
;//! \htmlinclude msgPOI-response.msg.html

(cl:defclass <msgPOI-response> (roslisp-msg-protocol:ros-message)
  ((found
    :reader found
    :initarg :found
    :type cl:boolean
    :initform cl:nil)
   (poi_name_out
    :reader poi_name_out
    :initarg :poi_name_out
    :type cl:string
    :initform ""))
)

(cl:defclass msgPOI-response (<msgPOI-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msgPOI-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msgPOI-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ari_pkg-srv:<msgPOI-response> is deprecated: use ari_pkg-srv:msgPOI-response instead.")))

(cl:ensure-generic-function 'found-val :lambda-list '(m))
(cl:defmethod found-val ((m <msgPOI-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ari_pkg-srv:found-val is deprecated.  Use ari_pkg-srv:found instead.")
  (found m))

(cl:ensure-generic-function 'poi_name_out-val :lambda-list '(m))
(cl:defmethod poi_name_out-val ((m <msgPOI-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ari_pkg-srv:poi_name_out-val is deprecated.  Use ari_pkg-srv:poi_name_out instead.")
  (poi_name_out m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msgPOI-response>) ostream)
  "Serializes a message object of type '<msgPOI-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'found) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'poi_name_out))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'poi_name_out))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msgPOI-response>) istream)
  "Deserializes a message object of type '<msgPOI-response>"
    (cl:setf (cl:slot-value msg 'found) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'poi_name_out) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'poi_name_out) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msgPOI-response>)))
  "Returns string type for a service object of type '<msgPOI-response>"
  "ari_pkg/msgPOIResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msgPOI-response)))
  "Returns string type for a service object of type 'msgPOI-response"
  "ari_pkg/msgPOIResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msgPOI-response>)))
  "Returns md5sum for a message object of type '<msgPOI-response>"
  "c796f839409775c93131126db79153f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msgPOI-response)))
  "Returns md5sum for a message object of type 'msgPOI-response"
  "c796f839409775c93131126db79153f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msgPOI-response>)))
  "Returns full string definition for message of type '<msgPOI-response>"
  (cl:format cl:nil "bool found~%string poi_name_out~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msgPOI-response)))
  "Returns full string definition for message of type 'msgPOI-response"
  (cl:format cl:nil "bool found~%string poi_name_out~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msgPOI-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'poi_name_out))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msgPOI-response>))
  "Converts a ROS message object to a list"
  (cl:list 'msgPOI-response
    (cl:cons ':found (found msg))
    (cl:cons ':poi_name_out (poi_name_out msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'msgPOI)))
  'msgPOI-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'msgPOI)))
  'msgPOI-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msgPOI)))
  "Returns string type for a service object of type '<msgPOI>"
  "ari_pkg/msgPOI")