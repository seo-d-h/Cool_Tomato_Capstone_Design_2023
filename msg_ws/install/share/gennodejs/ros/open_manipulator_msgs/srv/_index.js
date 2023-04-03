
"use strict";

let GetKinematicsPose = require('./GetKinematicsPose.js')
let SetKinematicsPose = require('./SetKinematicsPose.js')
let SetActuatorState = require('./SetActuatorState.js')
let GetJointPosition = require('./GetJointPosition.js')
let SetJointPosition = require('./SetJointPosition.js')
let SetDrawingTrajectory = require('./SetDrawingTrajectory.js')

module.exports = {
  GetKinematicsPose: GetKinematicsPose,
  SetKinematicsPose: SetKinematicsPose,
  SetActuatorState: SetActuatorState,
  GetJointPosition: GetJointPosition,
  SetJointPosition: SetJointPosition,
  SetDrawingTrajectory: SetDrawingTrajectory,
};
