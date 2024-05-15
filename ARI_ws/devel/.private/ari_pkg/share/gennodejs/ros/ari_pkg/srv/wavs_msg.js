// Auto-generated. Do not edit!

// (in-package ari_pkg.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class wavs_msgRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.fileName = null;
      this.text = null;
    }
    else {
      if (initObj.hasOwnProperty('fileName')) {
        this.fileName = initObj.fileName
      }
      else {
        this.fileName = '';
      }
      if (initObj.hasOwnProperty('text')) {
        this.text = initObj.text
      }
      else {
        this.text = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type wavs_msgRequest
    // Serialize message field [fileName]
    bufferOffset = _serializer.string(obj.fileName, buffer, bufferOffset);
    // Serialize message field [text]
    bufferOffset = _serializer.string(obj.text, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type wavs_msgRequest
    let len;
    let data = new wavs_msgRequest(null);
    // Deserialize message field [fileName]
    data.fileName = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [text]
    data.text = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.fileName.length;
    length += object.text.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ari_pkg/wavs_msgRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6d41350c8cd5607eba91f1184ee4a180';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string fileName
    string text
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new wavs_msgRequest(null);
    if (msg.fileName !== undefined) {
      resolved.fileName = msg.fileName;
    }
    else {
      resolved.fileName = ''
    }

    if (msg.text !== undefined) {
      resolved.text = msg.text;
    }
    else {
      resolved.text = ''
    }

    return resolved;
    }
};

class wavs_msgResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.msgResp = null;
    }
    else {
      if (initObj.hasOwnProperty('msgResp')) {
        this.msgResp = initObj.msgResp
      }
      else {
        this.msgResp = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type wavs_msgResponse
    // Serialize message field [msgResp]
    bufferOffset = _serializer.string(obj.msgResp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type wavs_msgResponse
    let len;
    let data = new wavs_msgResponse(null);
    // Deserialize message field [msgResp]
    data.msgResp = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.msgResp.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ari_pkg/wavs_msgResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5bc7275900b5ab950c42dea71449a4d0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string msgResp
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new wavs_msgResponse(null);
    if (msg.msgResp !== undefined) {
      resolved.msgResp = msg.msgResp;
    }
    else {
      resolved.msgResp = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: wavs_msgRequest,
  Response: wavs_msgResponse,
  md5sum() { return 'e6a881b942915d9139f8909f61208275'; },
  datatype() { return 'ari_pkg/wavs_msg'; }
};
