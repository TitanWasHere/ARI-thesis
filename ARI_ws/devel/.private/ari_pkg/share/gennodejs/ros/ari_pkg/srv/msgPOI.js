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

class msgPOIRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.poi_names = null;
    }
    else {
      if (initObj.hasOwnProperty('poi_names')) {
        this.poi_names = initObj.poi_names
      }
      else {
        this.poi_names = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type msgPOIRequest
    // Serialize message field [poi_names]
    bufferOffset = _arraySerializer.string(obj.poi_names, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type msgPOIRequest
    let len;
    let data = new msgPOIRequest(null);
    // Deserialize message field [poi_names]
    data.poi_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.poi_names.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ari_pkg/msgPOIRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '26916b073f3bd9eee4b179ce0676ffa2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] poi_names
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new msgPOIRequest(null);
    if (msg.poi_names !== undefined) {
      resolved.poi_names = msg.poi_names;
    }
    else {
      resolved.poi_names = []
    }

    return resolved;
    }
};

class msgPOIResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.found = null;
      this.poi_name_out = null;
    }
    else {
      if (initObj.hasOwnProperty('found')) {
        this.found = initObj.found
      }
      else {
        this.found = false;
      }
      if (initObj.hasOwnProperty('poi_name_out')) {
        this.poi_name_out = initObj.poi_name_out
      }
      else {
        this.poi_name_out = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type msgPOIResponse
    // Serialize message field [found]
    bufferOffset = _serializer.bool(obj.found, buffer, bufferOffset);
    // Serialize message field [poi_name_out]
    bufferOffset = _serializer.string(obj.poi_name_out, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type msgPOIResponse
    let len;
    let data = new msgPOIResponse(null);
    // Deserialize message field [found]
    data.found = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [poi_name_out]
    data.poi_name_out = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.poi_name_out.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ari_pkg/msgPOIResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3d958e07ec49ea85622945a237e0c05b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool found
    string poi_name_out
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new msgPOIResponse(null);
    if (msg.found !== undefined) {
      resolved.found = msg.found;
    }
    else {
      resolved.found = false
    }

    if (msg.poi_name_out !== undefined) {
      resolved.poi_name_out = msg.poi_name_out;
    }
    else {
      resolved.poi_name_out = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: msgPOIRequest,
  Response: msgPOIResponse,
  md5sum() { return 'c796f839409775c93131126db79153f5'; },
  datatype() { return 'ari_pkg/msgPOI'; }
};
