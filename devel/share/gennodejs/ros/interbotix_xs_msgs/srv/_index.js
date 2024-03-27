
"use strict";

let RobotInfo = require('./RobotInfo.js')
let MotorGains = require('./MotorGains.js')
let OperatingModes = require('./OperatingModes.js')
let Reboot = require('./Reboot.js')
let RegisterValues = require('./RegisterValues.js')
let TorqueEnable = require('./TorqueEnable.js')

module.exports = {
  RobotInfo: RobotInfo,
  MotorGains: MotorGains,
  OperatingModes: OperatingModes,
  Reboot: Reboot,
  RegisterValues: RegisterValues,
  TorqueEnable: TorqueEnable,
};
