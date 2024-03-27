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

class FilterParamsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x_filter_min = null;
      this.x_filter_max = null;
      this.y_filter_min = null;
      this.y_filter_max = null;
      this.z_filter_min = null;
      this.z_filter_max = null;
      this.voxel_leaf_size = null;
      this.plane_max_iter = null;
      this.plane_dist_thresh = null;
      this.ror_radius_search = null;
      this.ror_min_neighbors = null;
      this.cluster_tol = null;
      this.cluster_min_size = null;
      this.cluster_max_size = null;
    }
    else {
      if (initObj.hasOwnProperty('x_filter_min')) {
        this.x_filter_min = initObj.x_filter_min
      }
      else {
        this.x_filter_min = 0.0;
      }
      if (initObj.hasOwnProperty('x_filter_max')) {
        this.x_filter_max = initObj.x_filter_max
      }
      else {
        this.x_filter_max = 0.0;
      }
      if (initObj.hasOwnProperty('y_filter_min')) {
        this.y_filter_min = initObj.y_filter_min
      }
      else {
        this.y_filter_min = 0.0;
      }
      if (initObj.hasOwnProperty('y_filter_max')) {
        this.y_filter_max = initObj.y_filter_max
      }
      else {
        this.y_filter_max = 0.0;
      }
      if (initObj.hasOwnProperty('z_filter_min')) {
        this.z_filter_min = initObj.z_filter_min
      }
      else {
        this.z_filter_min = 0.0;
      }
      if (initObj.hasOwnProperty('z_filter_max')) {
        this.z_filter_max = initObj.z_filter_max
      }
      else {
        this.z_filter_max = 0.0;
      }
      if (initObj.hasOwnProperty('voxel_leaf_size')) {
        this.voxel_leaf_size = initObj.voxel_leaf_size
      }
      else {
        this.voxel_leaf_size = 0.0;
      }
      if (initObj.hasOwnProperty('plane_max_iter')) {
        this.plane_max_iter = initObj.plane_max_iter
      }
      else {
        this.plane_max_iter = 0;
      }
      if (initObj.hasOwnProperty('plane_dist_thresh')) {
        this.plane_dist_thresh = initObj.plane_dist_thresh
      }
      else {
        this.plane_dist_thresh = 0.0;
      }
      if (initObj.hasOwnProperty('ror_radius_search')) {
        this.ror_radius_search = initObj.ror_radius_search
      }
      else {
        this.ror_radius_search = 0.0;
      }
      if (initObj.hasOwnProperty('ror_min_neighbors')) {
        this.ror_min_neighbors = initObj.ror_min_neighbors
      }
      else {
        this.ror_min_neighbors = 0;
      }
      if (initObj.hasOwnProperty('cluster_tol')) {
        this.cluster_tol = initObj.cluster_tol
      }
      else {
        this.cluster_tol = 0.0;
      }
      if (initObj.hasOwnProperty('cluster_min_size')) {
        this.cluster_min_size = initObj.cluster_min_size
      }
      else {
        this.cluster_min_size = 0;
      }
      if (initObj.hasOwnProperty('cluster_max_size')) {
        this.cluster_max_size = initObj.cluster_max_size
      }
      else {
        this.cluster_max_size = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FilterParamsRequest
    // Serialize message field [x_filter_min]
    bufferOffset = _serializer.float32(obj.x_filter_min, buffer, bufferOffset);
    // Serialize message field [x_filter_max]
    bufferOffset = _serializer.float32(obj.x_filter_max, buffer, bufferOffset);
    // Serialize message field [y_filter_min]
    bufferOffset = _serializer.float32(obj.y_filter_min, buffer, bufferOffset);
    // Serialize message field [y_filter_max]
    bufferOffset = _serializer.float32(obj.y_filter_max, buffer, bufferOffset);
    // Serialize message field [z_filter_min]
    bufferOffset = _serializer.float32(obj.z_filter_min, buffer, bufferOffset);
    // Serialize message field [z_filter_max]
    bufferOffset = _serializer.float32(obj.z_filter_max, buffer, bufferOffset);
    // Serialize message field [voxel_leaf_size]
    bufferOffset = _serializer.float32(obj.voxel_leaf_size, buffer, bufferOffset);
    // Serialize message field [plane_max_iter]
    bufferOffset = _serializer.int32(obj.plane_max_iter, buffer, bufferOffset);
    // Serialize message field [plane_dist_thresh]
    bufferOffset = _serializer.float32(obj.plane_dist_thresh, buffer, bufferOffset);
    // Serialize message field [ror_radius_search]
    bufferOffset = _serializer.float32(obj.ror_radius_search, buffer, bufferOffset);
    // Serialize message field [ror_min_neighbors]
    bufferOffset = _serializer.int32(obj.ror_min_neighbors, buffer, bufferOffset);
    // Serialize message field [cluster_tol]
    bufferOffset = _serializer.float32(obj.cluster_tol, buffer, bufferOffset);
    // Serialize message field [cluster_min_size]
    bufferOffset = _serializer.int32(obj.cluster_min_size, buffer, bufferOffset);
    // Serialize message field [cluster_max_size]
    bufferOffset = _serializer.int32(obj.cluster_max_size, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FilterParamsRequest
    let len;
    let data = new FilterParamsRequest(null);
    // Deserialize message field [x_filter_min]
    data.x_filter_min = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x_filter_max]
    data.x_filter_max = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_filter_min]
    data.y_filter_min = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_filter_max]
    data.y_filter_max = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z_filter_min]
    data.z_filter_min = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z_filter_max]
    data.z_filter_max = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [voxel_leaf_size]
    data.voxel_leaf_size = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [plane_max_iter]
    data.plane_max_iter = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [plane_dist_thresh]
    data.plane_dist_thresh = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ror_radius_search]
    data.ror_radius_search = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ror_min_neighbors]
    data.ror_min_neighbors = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [cluster_tol]
    data.cluster_tol = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cluster_min_size]
    data.cluster_min_size = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [cluster_max_size]
    data.cluster_max_size = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'interbotix_perception_modules/FilterParamsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '57d51b939efff834ad5e55a181a7a0a0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This service is used specifically in the interbotix_perception_modules package
    #
    # Parameters used to tune the various PCL Pointcloud filters
    
    float32 x_filter_min            # minimum value [m] along the x axis beyond which all points will be cropped when doing Crop Box Filtering
    float32 x_filter_max            # maximum value [m] along the x axis beyond which all points will be cropped when doing Crop Box Filtering
    float32 y_filter_min            # minimum value [m] along the y axis beyond which all points will be cropped when doing Crop Box Filtering
    float32 y_filter_max            # maximum value [m] along the y axis beyond which all points will be cropped when doing Crop Box Filtering
    float32 z_filter_min            # minimum value [m] along the z axis beyond which all points will be cropped when doing Crop Box Filtering
    float32 z_filter_max            # maximum value [m] along the z axis beyond which all points will be cropped when doing Crop Box Filtering
    float32 voxel_leaf_size         # voxel leaf size [m] for the x, y, and z axes when doing Voxel Filtering
    int32 plane_max_iter            # maximum number of iterations the algorithm should go when when doing Planar Segmentation
    float32 plane_dist_thresh       # distance [m] perpendicular from the planar model outside of which points should not be segmented out when doing Planar Segmentation
    float32 ror_radius_search       # desired search radius when doing Radius Outlier Removal
    int32 ror_min_neighbors         # minimum number of neighbors a point should have to not be cropped out when doing Radius Outlier Removal
    float32 cluster_tol             # any point within this distance [m] will be considered part of the same cluster
    int32 cluster_min_size          # minimum number of points a cluster must have to be considered a cluster
    int32 cluster_max_size          # maximum number of points a cluster can have to be considered a cluster
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FilterParamsRequest(null);
    if (msg.x_filter_min !== undefined) {
      resolved.x_filter_min = msg.x_filter_min;
    }
    else {
      resolved.x_filter_min = 0.0
    }

    if (msg.x_filter_max !== undefined) {
      resolved.x_filter_max = msg.x_filter_max;
    }
    else {
      resolved.x_filter_max = 0.0
    }

    if (msg.y_filter_min !== undefined) {
      resolved.y_filter_min = msg.y_filter_min;
    }
    else {
      resolved.y_filter_min = 0.0
    }

    if (msg.y_filter_max !== undefined) {
      resolved.y_filter_max = msg.y_filter_max;
    }
    else {
      resolved.y_filter_max = 0.0
    }

    if (msg.z_filter_min !== undefined) {
      resolved.z_filter_min = msg.z_filter_min;
    }
    else {
      resolved.z_filter_min = 0.0
    }

    if (msg.z_filter_max !== undefined) {
      resolved.z_filter_max = msg.z_filter_max;
    }
    else {
      resolved.z_filter_max = 0.0
    }

    if (msg.voxel_leaf_size !== undefined) {
      resolved.voxel_leaf_size = msg.voxel_leaf_size;
    }
    else {
      resolved.voxel_leaf_size = 0.0
    }

    if (msg.plane_max_iter !== undefined) {
      resolved.plane_max_iter = msg.plane_max_iter;
    }
    else {
      resolved.plane_max_iter = 0
    }

    if (msg.plane_dist_thresh !== undefined) {
      resolved.plane_dist_thresh = msg.plane_dist_thresh;
    }
    else {
      resolved.plane_dist_thresh = 0.0
    }

    if (msg.ror_radius_search !== undefined) {
      resolved.ror_radius_search = msg.ror_radius_search;
    }
    else {
      resolved.ror_radius_search = 0.0
    }

    if (msg.ror_min_neighbors !== undefined) {
      resolved.ror_min_neighbors = msg.ror_min_neighbors;
    }
    else {
      resolved.ror_min_neighbors = 0
    }

    if (msg.cluster_tol !== undefined) {
      resolved.cluster_tol = msg.cluster_tol;
    }
    else {
      resolved.cluster_tol = 0.0
    }

    if (msg.cluster_min_size !== undefined) {
      resolved.cluster_min_size = msg.cluster_min_size;
    }
    else {
      resolved.cluster_min_size = 0
    }

    if (msg.cluster_max_size !== undefined) {
      resolved.cluster_max_size = msg.cluster_max_size;
    }
    else {
      resolved.cluster_max_size = 0
    }

    return resolved;
    }
};

class FilterParamsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FilterParamsResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FilterParamsResponse
    let len;
    let data = new FilterParamsResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'interbotix_perception_modules/FilterParamsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FilterParamsResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: FilterParamsRequest,
  Response: FilterParamsResponse,
  md5sum() { return '57d51b939efff834ad5e55a181a7a0a0'; },
  datatype() { return 'interbotix_perception_modules/FilterParams'; }
};
