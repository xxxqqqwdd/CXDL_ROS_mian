// Auto-generated. Do not edit!

// (in-package cx_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class joint_angle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left_arm_joint = null;
      this.right_arm_joint = null;
    }
    else {
      if (initObj.hasOwnProperty('left_arm_joint')) {
        this.left_arm_joint = initObj.left_arm_joint
      }
      else {
        this.left_arm_joint = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('right_arm_joint')) {
        this.right_arm_joint = initObj.right_arm_joint
      }
      else {
        this.right_arm_joint = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type joint_angle
    // Check that the constant length array field [left_arm_joint] has the right length
    if (obj.left_arm_joint.length !== 6) {
      throw new Error('Unable to serialize array field left_arm_joint - length must be 6')
    }
    // Serialize message field [left_arm_joint]
    bufferOffset = _arraySerializer.float64(obj.left_arm_joint, buffer, bufferOffset, 6);
    // Check that the constant length array field [right_arm_joint] has the right length
    if (obj.right_arm_joint.length !== 6) {
      throw new Error('Unable to serialize array field right_arm_joint - length must be 6')
    }
    // Serialize message field [right_arm_joint]
    bufferOffset = _arraySerializer.float64(obj.right_arm_joint, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type joint_angle
    let len;
    let data = new joint_angle(null);
    // Deserialize message field [left_arm_joint]
    data.left_arm_joint = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [right_arm_joint]
    data.right_arm_joint = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    return 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cx_driver/joint_angle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '990341802786d12533e60a9263f3d7c1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[6] left_arm_joint
    float64[6] right_arm_joint
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new joint_angle(null);
    if (msg.left_arm_joint !== undefined) {
      resolved.left_arm_joint = msg.left_arm_joint;
    }
    else {
      resolved.left_arm_joint = new Array(6).fill(0)
    }

    if (msg.right_arm_joint !== undefined) {
      resolved.right_arm_joint = msg.right_arm_joint;
    }
    else {
      resolved.right_arm_joint = new Array(6).fill(0)
    }

    return resolved;
    }
};

module.exports = joint_angle;
