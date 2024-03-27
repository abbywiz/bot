// Auto-generated. Do not edit!

// (in-package interbotix_perception_modules.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SnapPictureRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.filename = null;
    }
    else {
      if (initObj.hasOwnProperty('filename')) {
        this.filename = initObj.filename
      }
      else {
        this.filename = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SnapPictureRequest
    // Serialize message field [filename]
    bufferOffset = _serializer.string(obj.filename, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SnapPictureRequest
    let len;
    let data = new SnapPictureRequest(null);
    // Deserialize message field [filename]
    data.filename = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.filename);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'interbotix_perception_modules/SnapPictureRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '030824f52a0628ead956fb9d67e66ae9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This service is used specifically in the interbotix_perception_modules package
    #
    # Save the latest rgb picture with the specified name
    #
    # Request consists of:
    #   filename : name of file in which to save image including extension (.jpg)
    #
    # Response consists of:
    #    success : boolean indication of service success
    
    string filename
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SnapPictureRequest(null);
    if (msg.filename !== undefined) {
      resolved.filename = msg.filename;
    }
    else {
      resolved.filename = ''
    }

    return resolved;
    }
};

class SnapPictureResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.filepath = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('filepath')) {
        this.filepath = initObj.filepath
      }
      else {
        this.filepath = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SnapPictureResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [filepath]
    bufferOffset = _serializer.string(obj.filepath, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SnapPictureResponse
    let len;
    let data = new SnapPictureResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [filepath]
    data.filepath = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.filepath);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'interbotix_perception_modules/SnapPictureResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1af3a23d59f26a54fd4c297f9a0f8ee4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string filepath
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SnapPictureResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.filepath !== undefined) {
      resolved.filepath = msg.filepath;
    }
    else {
      resolved.filepath = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: SnapPictureRequest,
  Response: SnapPictureResponse,
  md5sum() { return 'a8777594bfb7f85102a1426edcb42742'; },
  datatype() { return 'interbotix_perception_modules/SnapPicture'; }
};
