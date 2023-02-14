// Auto-generated. Do not edit!

// (in-package me326_locobot_example.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class PixtoPointRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PixtoPointRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PixtoPointRequest
    let len;
    let data = new PixtoPointRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'me326_locobot_example/PixtoPointRequest';
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
    const resolved = new PixtoPointRequest(null);
    return resolved;
    }
};

class PixtoPointResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ptCld_point = null;
    }
    else {
      if (initObj.hasOwnProperty('ptCld_point')) {
        this.ptCld_point = initObj.ptCld_point
      }
      else {
        this.ptCld_point = new geometry_msgs.msg.PointStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PixtoPointResponse
    // Serialize message field [ptCld_point]
    bufferOffset = geometry_msgs.msg.PointStamped.serialize(obj.ptCld_point, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PixtoPointResponse
    let len;
    let data = new PixtoPointResponse(null);
    // Deserialize message field [ptCld_point]
    data.ptCld_point = geometry_msgs.msg.PointStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PointStamped.getMessageSize(object.ptCld_point);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'me326_locobot_example/PixtoPointResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '453b790c7c72ce0c0a5e253a59f6dc48';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/PointStamped ptCld_point
    
    
    ================================================================================
    MSG: geometry_msgs/PointStamped
    # This represents a Point with reference coordinate frame and timestamp
    Header header
    Point point
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PixtoPointResponse(null);
    if (msg.ptCld_point !== undefined) {
      resolved.ptCld_point = geometry_msgs.msg.PointStamped.Resolve(msg.ptCld_point)
    }
    else {
      resolved.ptCld_point = new geometry_msgs.msg.PointStamped()
    }

    return resolved;
    }
};

module.exports = {
  Request: PixtoPointRequest,
  Response: PixtoPointResponse,
  md5sum() { return '453b790c7c72ce0c0a5e253a59f6dc48'; },
  datatype() { return 'me326_locobot_example/PixtoPoint'; }
};
