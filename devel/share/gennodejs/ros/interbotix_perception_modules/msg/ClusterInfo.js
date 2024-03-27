// Auto-generated. Do not edit!

// (in-package interbotix_perception_modules.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ClusterInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.frame_id = null;
      this.position = null;
      this.yaw = null;
      this.color = null;
      this.min_z_point = null;
      this.num_points = null;
    }
    else {
      if (initObj.hasOwnProperty('frame_id')) {
        this.frame_id = initObj.frame_id
      }
      else {
        this.frame_id = '';
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('color')) {
        this.color = initObj.color
      }
      else {
        this.color = new std_msgs.msg.ColorRGBA();
      }
      if (initObj.hasOwnProperty('min_z_point')) {
        this.min_z_point = initObj.min_z_point
      }
      else {
        this.min_z_point = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('num_points')) {
        this.num_points = initObj.num_points
      }
      else {
        this.num_points = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ClusterInfo
    // Serialize message field [frame_id]
    bufferOffset = _serializer.string(obj.frame_id, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [color]
    bufferOffset = std_msgs.msg.ColorRGBA.serialize(obj.color, buffer, bufferOffset);
    // Serialize message field [min_z_point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.min_z_point, buffer, bufferOffset);
    // Serialize message field [num_points]
    bufferOffset = _serializer.int32(obj.num_points, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ClusterInfo
    let len;
    let data = new ClusterInfo(null);
    // Deserialize message field [frame_id]
    data.frame_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [color]
    data.color = std_msgs.msg.ColorRGBA.deserialize(buffer, bufferOffset);
    // Deserialize message field [min_z_point]
    data.min_z_point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [num_points]
    data.num_points = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.frame_id);
    return length + 76;
  }

  static datatype() {
    // Returns string type for a message object
    return 'interbotix_perception_modules/ClusterInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9627d2d019ccc4ed2077673b4b0f3b40';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new ClusterInfo(null);
    if (msg.frame_id !== undefined) {
      resolved.frame_id = msg.frame_id;
    }
    else {
      resolved.frame_id = ''
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.color !== undefined) {
      resolved.color = std_msgs.msg.ColorRGBA.Resolve(msg.color)
    }
    else {
      resolved.color = new std_msgs.msg.ColorRGBA()
    }

    if (msg.min_z_point !== undefined) {
      resolved.min_z_point = geometry_msgs.msg.Point.Resolve(msg.min_z_point)
    }
    else {
      resolved.min_z_point = new geometry_msgs.msg.Point()
    }

    if (msg.num_points !== undefined) {
      resolved.num_points = msg.num_points;
    }
    else {
      resolved.num_points = 0
    }

    return resolved;
    }
};

module.exports = ClusterInfo;
