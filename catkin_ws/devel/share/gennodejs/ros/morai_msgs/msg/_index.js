
"use strict";

let ObjectStatusList = require('./ObjectStatusList.js');
let SensorPosControl = require('./SensorPosControl.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let WaitForTick = require('./WaitForTick.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let PRStatus = require('./PRStatus.js');
let VehicleSpec = require('./VehicleSpec.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let MapSpec = require('./MapSpec.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let GhostMessage = require('./GhostMessage.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let VehicleCollision = require('./VehicleCollision.js');
let EventInfo = require('./EventInfo.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let IntscnTL = require('./IntscnTL.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let RadarDetection = require('./RadarDetection.js');
let SaveSensorData = require('./SaveSensorData.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let ObjectStatus = require('./ObjectStatus.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let PREvent = require('./PREvent.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let GPSMessage = require('./GPSMessage.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let IntersectionControl = require('./IntersectionControl.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let Lamps = require('./Lamps.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let TrafficLight = require('./TrafficLight.js');
let RadarDetections = require('./RadarDetections.js');
let ERP42Info = require('./ERP42Info.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let CollisionData = require('./CollisionData.js');
let ReplayInfo = require('./ReplayInfo.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let CtrlCmd = require('./CtrlCmd.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');

module.exports = {
  ObjectStatusList: ObjectStatusList,
  SensorPosControl: SensorPosControl,
  MoraiSimProcHandle: MoraiSimProcHandle,
  WaitForTick: WaitForTick,
  SyncModeInfo: SyncModeInfo,
  PRStatus: PRStatus,
  VehicleSpec: VehicleSpec,
  MoraiSrvResponse: MoraiSrvResponse,
  DdCtrlCmd: DdCtrlCmd,
  NpcGhostCmd: NpcGhostCmd,
  MapSpec: MapSpec,
  MoraiTLInfo: MoraiTLInfo,
  GhostMessage: GhostMessage,
  GetTrafficLightStatus: GetTrafficLightStatus,
  VehicleCollision: VehicleCollision,
  EventInfo: EventInfo,
  VehicleCollisionData: VehicleCollisionData,
  IntscnTL: IntscnTL,
  MultiEgoSetting: MultiEgoSetting,
  RadarDetection: RadarDetection,
  SaveSensorData: SaveSensorData,
  SyncModeRemoveObject: SyncModeRemoveObject,
  ObjectStatus: ObjectStatus,
  WaitForTickResponse: WaitForTickResponse,
  PREvent: PREvent,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  GPSMessage: GPSMessage,
  MapSpecIndex: MapSpecIndex,
  IntersectionStatus: IntersectionStatus,
  SyncModeCmd: SyncModeCmd,
  IntersectionControl: IntersectionControl,
  SyncModeCmdResponse: SyncModeCmdResponse,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  NpcGhostInfo: NpcGhostInfo,
  MoraiTLIndex: MoraiTLIndex,
  SyncModeResultResponse: SyncModeResultResponse,
  Lamps: Lamps,
  SyncModeSetGear: SyncModeSetGear,
  TrafficLight: TrafficLight,
  RadarDetections: RadarDetections,
  ERP42Info: ERP42Info,
  EgoVehicleStatus: EgoVehicleStatus,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  CollisionData: CollisionData,
  ReplayInfo: ReplayInfo,
  MoraiSimProcStatus: MoraiSimProcStatus,
  SetTrafficLight: SetTrafficLight,
  ScenarioLoad: ScenarioLoad,
  VehicleSpecIndex: VehicleSpecIndex,
  PRCtrlCmd: PRCtrlCmd,
  CtrlCmd: CtrlCmd,
  SyncModeAddObject: SyncModeAddObject,
};
