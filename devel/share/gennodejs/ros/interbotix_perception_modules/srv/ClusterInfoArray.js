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

let ClusterInfo = require('../msg/ClusterInfo.js');

//-----------------------------------------------------------

class ClusterInfoArrayRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ClusterInfoArrayRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ClusterInfoArrayRequest
    let len;
    let data = new ClusterInfoArrayRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'interbotix_perception_modules/ClusterInfoArrayRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This service is used specifically in the interbotix_perception_modules package
    #
    # List of ClusterInfo messages. The length of the list should correspond to the
    # number of objects seen by the camera
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ClusterInfoArrayRequest(null);
    return resolved;
    }
};

class ClusterInfoArrayResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.clusters = null;
    }
    else {
      if (initObj.hasOwnProperty('clusters')) {
        this.clusters = initObj.clusters
      }
      else {
        this.clusters = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ClusterInfoArrayResponse
    // Serialize message field [clusters]
    // Serialize the length for message field [clusters]
    bufferOffset = _serializer.uint32(obj.clusters.length, buffer, bufferOffset);
    obj.clusters.forEach((val) => {
      bufferOffset = ClusterInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ClusterInfoArrayResponse
    let len;
    let data = new ClusterInfoArrayResponse(null);
    // Deserialize message field [clusters]
    // Deserialize array length for message field [clusters]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.clusters = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.clusters[i] = ClusterInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.clusters.forEach((val) => {
      length += ClusterInfo.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'interbotix_perception_modules/ClusterInfoArrayResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '911bd211bd6fd7e568c4562cf40f28fd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    interbotix_perception_modules/ClusterInfo[] clusters
    
    
    ================================================================================
    MSG: interbotix_perception_modules/ClusterInfo
    # This message is used specifically in the interbotix_perception_modules package
    #
    # Message that holds information about each cluster
    
    string frame_id                     # parent frame of the cluster (usually 'camera_depth_optical_frame' or similar)
    geometry_msgs/Point position        # x, y, z position of the cluster
    float32 yaw                         # yaw [rad] of the cluster; x-axis of the cluster should align
                                        # with the major-axis of a best-fit ellipse (currently not supported)
    std_msgs/ColorRGBA color            # average RGB values (0 - 255) for the whole cluster
    geometry_msgs/Point min_z_point     # point with the min 'z' value of the cluster
    int32 num_points                    # number of points in the cluster
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: std_msgs/ColorRGBA
    float32 r
    float32 g
    float32 b
    float32 a
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ClusterInfoArrayResponse(null);
    if (msg.clusters !== undefined) {
      resolved.clusters = new Array(msg.clusters.length);
      for (let i = 0; i < resolved.clusters.length; ++i) {
        resolved.clusters[i] = ClusterInfo.Resolve(msg.clusters[i]);
      }
    }
    else {
      resolved.clusters = []
    }

    return resolved;
    }
};

module.exports = {
  Request: ClusterInfoArrayRequest,
  Response: ClusterInfoArrayResponse,
  md5sum() { return '911bd211bd6fd7e568c4562cf40f28fd'; },
  datatype() { return 'interbotix_perception_modules/ClusterInfoArray'; }
};
