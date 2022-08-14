/************************************************************************/
/*Page 1                                                                */
/************************************************************************/
#pragma once

/************************************************************************/
/*config                                                                */
/************************************************************************/
#define Protocol_ReadModbusDevices				"ReadModbusDevices"
#define Protocol_SetModbusDevices				"SetModbusDevices"

#define Protocol_GetSerialPortList				"GetSerialPortList"

#define Protocol_ReadRobotiqConfig				"ReadRobotiqConfig"

#define Protocol_ReadForceSensorConfig			"ReadForceSensorConfig"
#define Protocol_SetForceSensorConfig			"SetForceSensorConfig"

#define Protocol_ReadForceSensorRotation			"ReadForceSensorRotation"
#define Protocol_SetForceSensorRotation			"SetForceSensorRotation"

#define Protocol_ReadEndType					"ReadEndType"

#define Protocol_ReadModbusSlavePort			"ReadModbusSlavePort"

#define Protocol_ReadUILock						"ReadUILock"
#define Protocol_SetUILock						"SetUILock"

#define Protocol_ReadEmergencyHandleTime		"ReadEmergencyHandleTime"
#define Protocol_ReadEmergencyMonitor			"ReadEmergencyMonitor"
#define Protocol_SetEmergencyMonitor			"SetEmergencyMonitor"

#define Protocol_ReadEmergencyForResetIO		"ReadEmergencyForResetIO"
#define Protocol_SetEmergencyForResetIO			"SetEmergencyForResetIO"

#define Protocol_ReadEmergencyForResetIOState	"ReadEmergencyForResetIOState"
#define Protocol_SetEmergencyForResetIOState	"SetEmergencyForResetIOState"

#define Protocol_ReadSafetyGuardMonitor			"ReadSafetyGuardMonitor"
#define Protocol_SetSafetyGuardMonitor			"SetSafetyGuardMonitor"

#define Protocol_ReadSafetyGuardHold			"ReadSafetyGuardHold"
#define Protocol_SetSafetyGuardHold				"SetSafetyGuardHold"

#define Protocol_ReadSwitchOnMonitor			"ReadSwitchOnMonitor"
#define Protocol_SetSwitchOnMonitor				"SetSwitchOnMonitor"

#define Protocol_ReadSwitchOnWithBlackout		"ReadSwitchOnWithBlackout"
#define Protocol_SetSwitchOnWithBlackout		"SetSwitchOnWithBlackout"

#define Protocol_ReadTeachModeButton			"ReadTeachModeButton"
#define Protocol_SetTeachModeButton				"SetTeachModeButton"

#define Protocol_ReadOutOfSpaceWithBlackout		"ReadOutOfSpaceWithBlackout"
#define Protocol_SetOutOfSpaceWithBlackout		"SetOutOfSpaceWithBlackout"

#define Protocol_ReadConveyorConfig				"ReadConveyorConfig"
#define Protocol_SetConveyorConfig				"SetConveyorConfig"

#define Protocol_ReadInpositionConfig			"ReadInpositionConfig"
#define Protocol_SetInpositionConfig			"SetInpositionConfig"

#define Protocol_ReadNetworkIP					"ReadNetworkIP"
#define Protocol_SetNetworkIP					"SetNetworkIP"

#define Protocol_ReadAutoTurnRunScript			"ReadAutoTurnRunScript"
#define Protocol_SetAutoTurnRunScript			"SetAutoTurnRunScript"

//////////////////////////////////////////////////////////////////////////
#define Protocol_RbConnect "RbConnect"

#define Protocol_ScriptCompile				"ScriptCompile"
#define Protocol_ScriptRunningTime			"ScriptRunningTime"
#define Protocol_ScriptFunInfo "RbtProtocol_ScriptFunInfo"
#define Protocol_ScriptRead "RbtProtocol_ScriptRead"

#define Protocol_ReadConfig "RbtProtocol_ReadConfig"
#define Protocol_ReadRobotsInfo "RbtProtocol_ReadRobotsInfo"
#define Protocol_SetConfig "RbtProtocol_SetConfig"

#define Protocol_ReadHardConfig "RbtProtocol_ReadHardConfig"
#define Protocol_SetHardConfig "RbtProtocol_SetHardConfig"

#define Protocol_DragMove "RbtProtocol_DragMove"

//应用配置
#define Protocol_ReadRobotsConfig     "RbtProtocol_ReadRobotsConfig"
#define Protocol_SetRobotsConfig     "RbtProtocol_SetRobotsConfig"
#define Protocol_ImportApplicationConfig "RbtProtocol_ImportApplicationConfig"
//硬件配置
#define Protocol_ImportRobotsHardConfig "RbtProtocol_ImportRobotsHardConfig"
#define Protocol_StopImportRobotsHardConfig "RbtProtocol_StopImportRobotsHardConfig"

//读取日志信息
#define Protocol_ReadLogInfo "RbtProtocol_ReadLogInfo"

//
#define Protocol_ReadAcsSafeSpaceLimit "Protocol_ReadAcsSafeSpaceLimit"
#define Protocol_ReadJointSpeedLimit "Protocol_ReadJointSpeedLimit"
/************************************************************************/
/*motion                                                                */
/************************************************************************/
#define Protocol_GrpContinue		"GrpContinue"
#define Protocol_GrpInterrupt		"GrpInterrupt"
#define Protocol_GrpPowerOff		"GrpPowerOff"
#define Protocol_GrpPowerOn			"GrpPowerOn"
#define Protocol_GrpReset			"GrpReset"
#define Protocol_GrpStop			"GrpStop"

#define Protocol_MoveHoming			"MoveHoming"
#define Protocol_MoveP				"MoveP"
#define Protocol_MoveC				"MoveC"
#define Protocol_ShortJogJ			"ShortJogJ"
#define Protocol_ShortJogL			"ShortJogL"
#define Protocol_LongJogJ			"LongJogJ"
#define Protocol_LongJogL			"LongJogL"
#define Protocol_MoveRelJ			"MoveRelJ"
#define Protocol_MoveRelL			"MoveRelL"
#define Protocol_MoveJ				"MoveJ"
#define Protocol_MoveL				"MoveL"
//#define Protocol_MoveC				"MoveC"
#define Protocol_MoveBeltL			"MoveBelt"
#define Protocol_MoveB				"MoveB"
#define Protocol_MoveBJ				"MoveBJ"
#define Protocol_MoveRelL			"MoveRelL"
#define Protocol_MoveRelJ			"MoveRelJ"
#define Protocol_MoveZ				"MoveZ"
#define Protocol_MoveZES			"MoveZES"

#define Protocol_SetOverride				"SetOverride"
#define Protocol_StartAssistiveMode			"StartAssistiveMode"
#define Protocol_CloseAssistiveMode			"CloseAssistiveMode"
#define Protocol_StartAssistiveModeByEnd	"StartAssistiveModeByEnd"

#define Protocol_StartServo "StartServo"
#define Protocol_PushServoP "PushServoP"
#define Protocol_PushServoJ "PushServoJ"

#define Protocol_StartPushBlending "StartPushBlending"
#define Protocol_PushBlendingL "PushBlendingL"
#define Protocol_PushBlendingC "PushBlendingC"
#define Protocol_EndPushBlending "EndPushBlending"

#define Protocol_StartPushDragMove "StartPushDragMove"
#define Protocol_PushDragMove "PushDragMove"
#define Protocol_EndPushDragMove "EndPushDragMove"

#define Protocol_StartSetDragMove "StartSetDragMove"
#define Protocol_AddDragMove "AddDragMove"
#define Protocol_EndSetDragMove "EndSetDragMove"
#define Protocol_DoDragMove "DoDragMove"

#define Protocol_SetSensorForceMoveState "SetSensorForceMoveState"
#define Protocol_SetForceSensorThreshold "SetForceSensorThreshold"
#define Protocol_SetFTSearchSpeed "SetFTSearchSpeed"

#define Protocol_ReadFilteredSensorForce "ReadFilteredSensorForce"
#define Protocol_ReadForceSensorData "ReadForceSensorData"
#define Protocol_ReadSrcForceSensorData "ReadSrcForceSensorData"
#define Protocol_ReadSrcForceSensorState "ReadSrcForceSensorState"
#define Protocol_ReadForceVel	"ReadForceVel"
#define Protocol_ReadForceValue	"ReadForceValue"

#define Protocol_ReadControlBoxVoltag "ReadControlBoxVoltag"
#define Protocol_ReadControlBoxCurrent "ReadControlBoxCurrent"
#define Protocol_ReadJointVoltag "ReadJointVoltag"
#define Protocol_ReadJointCurrent "ReadJointCurrent"
#define Protocol_ReadJointTemperature "ReadJointTemperature"
#define Protocol_ReadJointAcceleration "ReadJointAcceleration"
#define Protocol_ReadJointDcceleration "ReadJointDcceleration"
#define Protocol_ReadJointVelocity "ReadJointVelocity"
#define Protocol_ReadKinematicsParam "ReadKinematicsParam"
#define Protocol_ReadPower "ReadPower"
#define Protocol_WriteModbusRegister  "WriteModbusRegister"
#define Protocol_ReadModbusRegister  "ReadModbusRegister"

#define Protocol_WriteModSig  "WriteModSig"
#define Protocol_ReadModSig  "ReadModSig"
#define Protocol_ReadMultiModSig  "ReadMultiModSig"

/************************************************************************/
/*                                                                      */
/************************************************************************/
#define Protocol_ReadInternalInputCoil  "ReadInternalInputCoil"
#define Protocol_ReadInternalCoil  "ReadInternalCoil"
#define Protocol_ReadInternalCoils  "ReadInternalCoils"
#define Protocol_SetInternalCoil  "SetInternalCoil"
#define Protocol_ReadInternalReg  "ReadInternalReg"
#define Protocol_SetInternalReg  "SetInternalReg"
/************************************************************************/
/*                                                                      */
/************************************************************************/
#define Protocol_SetForceControlState "SetForceControlState"
#define Protocol_SetForceControlAttr "SetForceControlAttr"

#define Protocol_SetFCCmdType "SetFCCmdType"
#define Protocol_SetFCFrameType "SetFCFrameType"
#define Protocol_SetFCEnable "SetFCEnable"
#define Protocol_SetFCFreedom "SetFCFreedom"
#define Protocol_SetFCCmdForce "SetFCCmdForce"
#define Protocol_SetFCCmdPID	"SetFCCmdPID"
#define Protocol_SetFCCmdZero "SetFCCmdZero"

#define Protocol_StartFeedforwardMode "StartFeedforwardMode"
#define Protocol_CloseFeedforwardMode "CloseFeedforwardMode"

#define Protocol_ReadSDO "ReadSDO"
#define Protocol_WriteSDO "WriteSDO"
#define Protocol_ReadDCSStatus "ReadDCSStatus"
/************************************************************************/
/*read state                                                            */
/************************************************************************/
#define Protocol_ReadElectricityBoxDI		"ReadElectricityBoxDI"
#define Protocol_ReadElectricityBoxSI		"ReadElectricityBoxSI"
#define Protocol_ReadElectricityBoxExterDI	"ReadElectricityBoxExterDI"
#define Protocol_ReadElectricityBoxDO		"ReadElectricityBoxDO"
#define Protocol_ReadElectricityBoxSO		"ReadElectricityBoxSO"
#define Protocol_ReadElectricityBoxExterDO	"ReadElectricityBoxExterDO"
#define Protocol_ReadElectricityBoxAnalog	"ReadElectricityBoxAnalog"
#define Protocol_ReadElectricityBoxPower	"ReadElectricityBoxPower"
#define Protocol_ReadScriptState			"ReadScriptState"
#define Protocol_ReadControllerState		"ReadControllerState"
#define Protocol_ReadRTRbtState				"ReadRTRbtState"
#define Protocol_ReadRTRbtError				"ReadRTRbtError"
#define Protocol_ReadRTRbtSpeed				"ReadRTRbtSpeed"
#define Protocol_ReadRTRbtEndIO				"ReadRTRbtEndIO"
#define Protocol_ReadRTRbtEndModbusBTN		"ReadRTRbtEndModbusBTN"
#define Protocol_ReadRTRbtACS				"ReadRTRbtACS"
#define Protocol_ReadRTRbtPCS				"ReadRTRbtPCS"
#define Protocol_ReadRTRbtPCSForBase		"ReadRTRbtPCSForBase"
#define Protocol_ReadRTRbtTOOL				"ReadRTRbtTOOL"
#define Protocol_ReadRTRbtMotorCurrent		"ReadRTRbtMotorCurrent"
#define Protocol_ReadRTRbtMotorTemperature	"ReadRTRbtMotorTemperature"
#define Protocol_ReadRTRbtMotorVoltage		"ReadRTRbtMotorVoltage"
#define Protocol_ReadConveyorPos			"ReadConveyorPos"

// #define Protocol_GrpPowerOn				"GrpPowerOn"
// #define Protocol_GrpPowerOff			"GrpPowerOff"
// #define Protocol_GrpStop				"GrpStop"
// #define Protocol_GrpReset				"GrpReset"
// #define Protocol_GrpInterrupt			"GrpInterrupt"
// #define Protocol_GrpContinue			"GrpContinue"
#define Protocol_GrpStopMoving			"GrpStopMoving"

// #define Protocol_ReadRTRbtState				"ReadRTRbtState"
#define Protocol_ReadRTControlState			"ReadRTControlState"
#define Protocol_ReadRTEtherCatState		"ReadRTEtherCatState"
#define Protocol_ReadErrorCode				"ReadErrorCode"
#define Protocol_ReadActualPos				"ReadActualPos"
#define Protocol_ReadCommandPos				"ReadCommandPos"
#define Protocol_ReadMotorCurrent			"ReadMotorCurrent"
#define Protocol_ReadTorque					"ReadTorque"
#define Protocol_ReadAcsVel					"ReadAcsVel"
#define Protocol_ReadPcsVel					"ReadPcsVel"
#define Protocol_ReadAcsAcc					"ReadAcsAcc"
#define Protocol_ReadPcsAcc					"ReadPcsAcc"
#define Protocol_ReadAbsEncode				"ReadAbsEncode"
// #define Protocol_ReadOverride				"ReadOverride"
#define Protocol_ReadConveyorInfo			"ReadConveyorInfo"
#define Protocol_ReadEndIOInfo				"ReadEndIOInfo"


#define Protocol_ReadCurrentApplicationSize "ReadCurrentApplicationSize"
#define Protocol_ReadCurrentApplication		"ReadCurrentApplication"
#define Protocol_ImportApplication			"ImportApplication"
/************************************************************************/
/*Set cmd                                                               */
/************************************************************************/
#define Protocol_SetOverride							"SetOverride"
#define Protocol_SetHomeEncode							"SetHomeEncode"
#define Protocol_SetEndDOState							"SetEndDOState"
#define Protocol_SetAcsSafeSpaceLimit					"SetAcsSafeSpaceLimit"
#define Protocol_SetPcsSafeSpaceLimit					"SetPcsSafeSpaceLimit"
#define Protocol_SetToolCoord							"SetToolCoord"
#define Protocol_SetUserCoord							"SetUserCoord"
#define Protocol_SetMoveJointMotionLimits				"SetMoveJointMotionLimits"
#define Protocol_SetMovePcsMotionLimits					"SetMovePcsMotionLimits"
#define Protocol_SetStopJointMotionLimits				"SetStopJointMotionLimits"
#define Protocol_SetStopPcsMotionLimits					"SetStopPcsMotionLimits"
//#define Protocol_SetConveyorConfig						"SetConveyorConfig"
//Set Config
#define Protocol_SetVelocityThresholds					"SetVelocityThresholds"
#define Protocol_SetDampConstant						"SetDampConstant"
#define Protocol_SetFrictionCompensationFactor			"SetFrictionCompensationFactor"
#define Protocol_SetMaxEfficiency						"SetMaxEfficiency"
#define Protocol_SetTorqueConstant						"SetTorqueConstant"
#define Protocol_SetKinematicsParameters				"SetKinematicsParameters"
#define Protocol_SetDynamicsParameters1					"SetDynamicsParameters1"
#define Protocol_SetDynamicsParameters2					"SetDynamicsParameters2"
#define Protocol_SetDynamicsParameters3					"SetDynamicsParameters3"	
#define Protocol_SetDynamicsParameters4					"SetDynamicsParameters4"
#define Protocol_SetDynamicsParameters5					"SetDynamicsParameters5"
#define Protocol_SetDynamicsParameters6					"SetDynamicsParameters6"
#define Protocol_SetInitRobotDynamics					"SetInitRobotDynamics"
#define Protocol_SetGravityParam						"SetGravityParam"
#define Protocol_SetMotorCurrentScales					"SetMotorCurrentScales"
#define Protocol_SetMaxActCurrent						"SetMaxActCurrent"
//Set Collision Config
#define Protocol_SetBaseMountingAngle					"SetBaseMountingAngle"
#define Protocol_SetPayload								"SetPayload"
#define Protocol_SetCollideStopThresholds				"SetCollideStopThresholds"
#define Protocol_SetCollisionMomentumThresholds			"SetCollisionMomentumThresholds"
#define Protocol_SetAssistiveModeCollideStopThresholds	"SetAssistiveModeCollideStopThresholds"
//Set Robot Status
#define Protocol_SetCollisionStopMode					"SetCollisionStopMode"
#define Protocol_SetCollisionStopLevel					"SetCollideStopLevel"
#define Protocol_SetFreeDriveMode						"SetFreeDriveMode"
#define Protocol_SetTorqueFeedForwardMode				"SetTorqueFeedForwardMode"
#define Protocol_SetVelocityFeedForwardParam			"SetVelocityFeedForwardParam"
#define Protocol_SetVelocityFeedForwardMode				"SetVelocityFeedForwardMode"

#define Protocol_SetToolCoordinateMotion				"SetToolCoordinateMotion"
#define Protocol_ReadToolMotion							"ReadToolMotion"

//示波器操作
#define Protocol_OnStartScope				            "OnStartScope"
#define Protocol_Test				            "Test"


/************************************************************************/
/*Page 2                                                                */
/************************************************************************/
// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the HANSROBOT_PRO_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// HANSROBOT_PRO_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifndef HANSROBOT_PRO_H_
#define HANSROBOT_PRO_H_

#ifdef HANSROBOT_PRO_EXPORTS
#define HANSROBOT_PRO_CLASS __declspec(dllexport)
#else
#define HANSROBOT_PRO_CLASS __declspec(dllimport)
#endif
#define HANSROBOT_PRO_API extern "C" HANSROBOT_PRO_CLASS

/***********************************************************
**enJogDirec_Negative=¸º·½Ïò
**enJogDirec_Positive=Õý·½Ïò
***********************************************************/
// enum EN_JogDirection
// {
// 	enJogDirec_Negative = 0,
// 	enJogDirec_Positive,
// };

#include "HR_IFPro.h"
#include "HansRobot_STDef.h"

typedef void(*HRIF_ErrorCallback)(int boxID, int rbtID,int nAxisID, int nErrorCode);

HANSROBOT_PRO_API int HRIF_GetRbtErrorCodeStr(int nErrCode, char* szErrMsg, int nLen);
HANSROBOT_PRO_API int HRIF_GetErrorCodeLevel(int nErr);

HANSROBOT_PRO_API int HRIF_SetErrorCallback(HRIF_ErrorCallback pErrCallback);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**hostName:electric box IP
**nPort:electric box PORT, default 10003
connect to Robot
***********************************************************/
HANSROBOT_PRO_API int HRIF_Connect(int boxID, const char* hostName, unsigned short nPort);
HANSROBOT_PRO_API int HRIF_ConnectWithout10004(int boxID, const char* hostName, unsigned short nPort);

/***********************************************************
**boxID: electric box ID , range : [0,5)
disconnect to robot
***********************************************************/
HANSROBOT_PRO_API int HRIF_DisConnect(int boxID);

/***********************************************************
**boxID: electric box ID , range : [0,5)
judge whether connect to robot
***********************************************************/
HANSROBOT_PRO_API bool HRIF_IsConnect(int boxID);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtStatus: elfin`s status sturct.this approach is not recommended
***********************************************************/
HANSROBOT_PRO_API int ReadVersion(int boxID, char* szVersion, int nLen);
/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtStatus: elfin`s status sturct.this approach is not recommended
***********************************************************/
HANSROBOT_PRO_API int ReadRobotCurStatus(int boxID, int* nCurStatus);
//HANSROBOT_PRO_API int ReadRobotStatus(int boxID, ST_RbtsStatu* rbtStatus);

/***********************************************************
**boxID: electric box ID , range : [0,5)
control command
robot electricity
***********************************************************/
HANSROBOT_PRO_API int Electrify(int boxID);
HANSROBOT_PRO_API int ElectrifyEx(int boxID, bool bSynchronized);

/***********************************************************
**boxID: electric box ID , range : [0,5)
control command
robot blackout
***********************************************************/
HANSROBOT_PRO_API int Blackout(int boxID);

/***********************************************************
**boxID: electric box ID , range : [0,5)
control command
connect robot hardware by EtherCAT
***********************************************************/
HANSROBOT_PRO_API int ConnectRobotECAT(int boxID);

/***********************************************************
**boxID: electric box ID , range : [0,5)
control command
disconnect robot hardware by EtherCAT
***********************************************************/
HANSROBOT_PRO_API int DisonnectRobotECAT(int boxID);

/***********************************************************
**boxID: electric box ID , range : [0,5)
script command
switch to free gravity mode(¿ªÁãÁ¦Ê¾½Ì)
***********************************************************/
HANSROBOT_PRO_API int StartAssistiveMode(int boxID, int rbtID);
HANSROBOT_PRO_API int StartAssistiveModeWithoutReturn(int boxID, int rbtID);
/***********************************************************
**boxID: electric box ID , range : [0,5)
script command
close free gravity mode(¹ØÁãÁ¦Ê¾½Ì)
***********************************************************/
HANSROBOT_PRO_API int CloseAssistiveMode(int boxID, int rbtID);
HANSROBOT_PRO_API int CloseAssistiveModeWithoutReturn(int boxID, int rbtID);
/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
made the robot axis power on
***********************************************************/
HANSROBOT_PRO_API int GrpPowerOn(int boxID, int rbtID, bool bSynchronized);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
made the robot axis power off
***********************************************************/
HANSROBOT_PRO_API int GrpPowerOff(int boxID, int rbtID);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
clear robot`s faults
***********************************************************/
HANSROBOT_PRO_API int GrpReset(int boxID, int rbtID);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
stop robot moving and also stop the script
***********************************************************/
HANSROBOT_PRO_API int GrpStop(int boxID, int rbtID);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**X Y Z : Kinematic Coordinate position`s value ,unit mm
**RX RY RZ: Kinematic Coordinate orientation`s value. unit angle
control command
set robot`s Kinematic Coordinate(Tools Coordinate)
***********************************************************/
HANSROBOT_PRO_API int SetKinematicCoordinate(int boxID, int rbtID, double X, double Y, double Z,
	double RX,double RY,double RZ);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**X Y Z : User Coordinate position`s value ,unit mm
**RX RY RZ: User Coordinate orientation`s value. unit angle
control command
set robot`s User Coordinate
***********************************************************/
HANSROBOT_PRO_API int SetUserCoordinate(int boxID, int rbtID, double X, double Y, double Z,
	double RX, double RY, double RZ);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
set robot`s velocity ratio 0~1 meaning 0%~100%
***********************************************************/
HANSROBOT_PRO_API int SetOverride(int boxID, int rbtID, double Override);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
set payload
***********************************************************/
HANSROBOT_PRO_API int SetPayload(int boxID, int rbtID, 
	double dbPayload, double dbCenterX, double dbCenterY, double dbCenterZ);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**TrackSwitch: 0 means close tracking, 1 means open tracking
control command
swtich robot`s tracking, 
***********************************************************/
HANSROBOT_PRO_API int SetTrackingSwitch(int boxID, int rbtID, int TrackSwitch);

/************************************************************************/
/*                                                                      */
/************************************************************************/

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**AxisID: joint ID, range: [0,5] means [J1,J6]
**Derection: 0 means Negative,1 means Positive
control command
jog short Joint coordinates
***********************************************************/
HANSROBOT_PRO_API int ShortJogJ(int boxID, int rbtID, int AxisID, int Derection);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**AxisID: Axis ID, range: [0,5] means [X,Y,Z,RX,RY,RZ]
**Derection: 0 means Negative,1 means Positive
control command
jog short Cartesian coordinates
***********************************************************/
HANSROBOT_PRO_API int ShortJogL(int boxID, int rbtID, int AxisID, int Derection);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**AxisID: joint ID, range: [0,5] means [J1,J6]
**Derection: 0 means Negative,1 means Positive
control command
jog long Joint coordinates. move until call GrpStop
***********************************************************/
HANSROBOT_PRO_API int LongJogJ(int boxID, int rbtID, int AxisID, int Derection);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**AxisID: Axis ID, range: [0,5] means [X,Y,Z,RX,RY,RZ]
**Derection: 0 means Negative,1 means Positive
control command
jog long Cartesian coordinates. move until call GrpStop
***********************************************************/
HANSROBOT_PRO_API int LongJogL(int boxID, int rbtID, int AxisID, int Derection);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**J1 J2 J3 J4 J5 J6: joints , unit angle
**Derection: 0 means Negative,1 means Positive
control command
motion in Joint coordinates. absolute motion
***********************************************************/
HANSROBOT_PRO_API int MoveJ(int boxID, int rbtID, double J1, double J2, double J3,
	double J4, double J5, double J6);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**X Y Z : position value ,unit mm
**RX RY RZ: orientation`s value. unit angle
control command
motion in Cartesian coordinates.linear absolute motion
***********************************************************/
HANSROBOT_PRO_API int MoveL(int boxID, int rbtID, double X, double Y, double Z,
	double RX, double RY, double RZ);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**X Y Z : position value ,unit mm
**RX RY RZ: orientation`s value. unit angle
control command
motion in Cartesian coordinates. absolute motion aborting
***********************************************************/
HANSROBOT_PRO_API int MoveB(int boxID, int rbtID, double X, double Y, double Z,
	double RX, double RY, double RZ);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
motion in Cartesian coordinates.circle absolute motion

XYZ of passing point
double Via_X,double Via_Y,double Via_Z

Target point
double Goal_X,double Goal_Y,double Goal_Z,
double Goal_RX,double Goal_RY,double Goal_RZ

Type 
0:plane circle meaning orientation will not change
1:Stereoscopic circle meaning orientation will change from current point to target point
***********************************************************/
HANSROBOT_PRO_API int MoveC(int boxID, int rbtID, double Via_X, double Via_Y, double Via_Z,
	double Goal_X,double Goal_Y,double Goal_Z,
	double Goal_RX,double Goal_RY,double Goal_RZ, int Type);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
motion in Joint coordinates. move to home position
***********************************************************/
HANSROBOT_PRO_API int MoveHoming(int boxID, int rbtID);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**dServoTime 
**dlookAheadTime 
control command
ready for servoj motion
***********************************************************/
HANSROBOT_PRO_API int StartServo(int boxID, int rbtID, double dServoTime, double dlookAheadTime);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
add ACS position for servoj motion
***********************************************************/
HANSROBOT_PRO_API int PushServoJ(int boxID, int rbtID, double J1, double J2, double J3,
	double J4, double J5, double J6);


/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
add PCS position for servoj motion
***********************************************************/
HANSROBOT_PRO_API int PushServoP(int boxID, int rbtID, double X, double Y, double Z,
	double RX, double RY, double RZ);


/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
ready for add blending motion
***********************************************************/
HANSROBOT_PRO_API int StartPushBlending(int boxID, int rbtID);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
add PCS position for blending linear motion
***********************************************************/
HANSROBOT_PRO_API int PushBlendingL(int boxID, int rbtID, double X, double Y, double Z,
	double RX, double RY, double RZ, double dRadius);
/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
add for blending circular motion
***********************************************************/
HANSROBOT_PRO_API int PushBlendingC(int boxID, int rbtID, 
	double Via_X, double Via_Y, double Via_Z,double Via_RX, double Via_RY, double Via_RZ,
	double Tag_X, double Tag_Y, double Tag_Z, double Tag_RX, double Tag_RY, double Tag_RZ,
	int nFixposure);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
start blending motion
***********************************************************/
HANSROBOT_PRO_API int EndPushBlending(int boxID, int rbtID);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**nMotionType
**dTolerance
control command
ready for add DragMove motion
***********************************************************/
HANSROBOT_PRO_API int StartPushDragMove(int boxID, int rbtID,int nMotionType,double dTolerance);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
add ACS position for DragMove motion
***********************************************************/
HANSROBOT_PRO_API int PushDragMove(int boxID, int rbtID, double J1, double J2, double J3,
	double J4, double J5, double J6);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
start DragMove motion
***********************************************************/
HANSROBOT_PRO_API int EndPushDragMove(int boxID, int rbtID);
/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command

***********************************************************/
HANSROBOT_PRO_API bool IsRobotError(int boxID, int rbtID,int& nErrorCode);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
judge wether elfin is axis enable
***********************************************************/
HANSROBOT_PRO_API bool IsRobotPowerOn(int boxID, int rbtID);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
judge wether elfin is on moving
***********************************************************/
HANSROBOT_PRO_API bool IsRobotMoving(int boxID, int rbtID);

HANSROBOT_PRO_API int ReadRbtCurTCP(int boxID, int rbtID, double* X, double* Y, double* Z,
	double* RX, double* RY, double* RZ);
HANSROBOT_PRO_API int ReadRbtCurUCS(int boxID, int rbtID, double* X, double* Y, double* Z,
	double* RX, double* RY, double* RZ);
HANSROBOT_PRO_API int ReadRbtCurPayload(int boxID, int rbtID, 
	double* dbPayload, double* dbCenterX, double* dbCenterY, double* dbCenterZ);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**J1 J2 J3 J4 J5 J6: joints , unit angle
control command
read elfin current joint angle
***********************************************************/
HANSROBOT_PRO_API int ReadRbtACS(int boxID, int rbtID, double* J1, double* J2, double* J3,
	double* J4, double* J5, double* J6);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**X Y Z : User Coordinate position`s value ,unit mm
**RX RY RZ: User Coordinate orientation`s value. unit angle
control command
read elfin current position
***********************************************************/
HANSROBOT_PRO_API int ReadRbtPCS(int boxID, int rbtID, double* X, double* Y, double* Z,
	double* RX, double* RY, double* RZ);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0

input
**X Y Z : User Coordinate position`s value ,unit mm
**RX RY RZ: User Coordinate orientation`s value. unit angle

output
**J1 J2 J3 J4 J5 J6: joints , unit angle

control command
transfer PCS to ACS
***********************************************************/
HANSROBOT_PRO_API int RobotPCS2ACS(int boxID, int rbtID, 
	double X, double Y, double Z, double RX, double RY, double RZ,
	double* J1, double* J2, double* J3, double* J4, double* J5, double* J6);

HANSROBOT_PRO_API int GetMatrix2PCS(int boxID, int rbtID,
	double dbMatrix00, double dbMatrix01, double dbMatrix02, double dbMatrix03,
	double dbMatrix10, double dbMatrix11, double dbMatrix12, double dbMatrix13,
	double dbMatrix20, double dbMatrix21, double dbMatrix22, double dbMatrix23,
	double dbMatrix30, double dbMatrix31, double dbMatrix32, double dbMatrix33,
	double& X, double& Y, double& Z, double& RX, double& RY, double& RZ);
HANSROBOT_PRO_API int GetPCS2Matrix(int boxID, int rbtID,
	double& X, double Y, double Z, double RX, double RY, double RZ,
	double& dbMatrix00, double& dbMatrix01, double& dbMatrix02, double& dbMatrix03,
	double& dbMatrix10, double& dbMatrix11, double& dbMatrix12, double& dbMatrix13,
	double& dbMatrix20, double& dbMatrix21, double& dbMatrix22, double& dbMatrix23,
	double& dbMatrix30, double& dbMatrix31, double& dbMatrix32, double& dbMatrix33
	);
/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
control command
Read robot`s velocity ratio 0~1 meaning 0%~100%
***********************************************************/
HANSROBOT_PRO_API int ReadOverride(int boxID, int rbtID, double* Override);

/***********************************************************
**boxID: electric box ID , range : [0,5)
script command
switch to auto mode
***********************************************************/
HANSROBOT_PRO_API int SetAutoMode(int boxID);
HANSROBOT_PRO_API int SetAutoModeWithoutReturn(int boxID);
/***********************************************************
**boxID: electric box ID , range : [0,5)
script command
switch to manual mode
***********************************************************/
HANSROBOT_PRO_API int SetManualMode(int boxID);
HANSROBOT_PRO_API int SetManualModeWithoutReturn(int boxID);

/***********************************************************
**boxID: electric box ID , range : [0,5)
script command
start script`s timer
***********************************************************/
HANSROBOT_PRO_API int ReadCurrentApplication(int boxID, char* szApplication, int nBufLen, int& nSize);
HANSROBOT_PRO_API int ImportApplication(int boxID, const char* szApplication);

/***********************************************************
**boxID: electric box ID , range : [0,5)
script command
start script`s timer
***********************************************************/
HANSROBOT_PRO_API int StartUDMTimer(int boxID, bool bWaitReturn);

/***********************************************************
**boxID: electric box ID , range : [0,5)
script command
close script`s timer
***********************************************************/
HANSROBOT_PRO_API int CloseUDMTimer(int boxID, bool bWaitReturn);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**szUDMCmd: UDM cmd, example: udm1,param1,;
**bWaitReturn: wait for UDM return
script command
close script`s timer
***********************************************************/
HANSROBOT_PRO_API int RunUDM(int boxID, const char* szUDMCmd, bool bWaitReturn);

HANSROBOT_PRO_API int RunUDMWithResponse(int boxID, const char* szUDMCmd, vector<string>& vcRespone, long waittime_tv_sec, long waittime_tv_usec);
/***********************************************************
**boxID: electric box ID , range : [0,5)
**szUDMCmd: UDM cmd, example: udm1,param1,;
**bWaitReturn: wait for UDM return
script command
hold script
***********************************************************/
HANSROBOT_PRO_API int HoldUDM(int boxID);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**szUDMCmd: UDM cmd, example: udm1,param1,;
**bWaitReturn: wait for UDM return
script command
Continue script
***********************************************************/
HANSROBOT_PRO_API int ContinueUDM(int boxID);

HANSROBOT_PRO_API int ReadMainFirstPoint(int boxID, double* J1, double* J2, double* J3,
	double* J4, double* J5, double* J6);
/************************************************************************/
/*                                                                      */
/************************************************************************/

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**ioIndex: io index, range [0,3]
**ioState: 0 low , 1 high
control command
set robot`s end`s digital Output IO
***********************************************************/
HANSROBOT_PRO_API int SetEndDOState(int boxID, int rbtID, int ioIndex, int ioState);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**ioIndex: io index, range [0,3]
**state: 0 low , 1 high
control command
Read robot`s End digital Output IO
***********************************************************/
HANSROBOT_PRO_API int ReadEndDOState(int boxID, int rbtID, int ioIndex, int* state);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**ioIndex: io index, range [0,3]
**state: 0 low , 1 high
control command
Read robot`s End digital Input IO
***********************************************************/
HANSROBOT_PRO_API int ReadEndDIState(int boxID, int rbtID, int ioIndex, int* state);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**rbtID: robot ID , default 0
**ioIndex: io index, range [0,3]
**state: 0 low , 1 high
control command
Read robot`s End digital Input IO
***********************************************************/
HANSROBOT_PRO_API int ReadEndBTNState(int boxID, int rbtID,
	int& nEndBTNSquare, int& nEndBTNPlus, int& nEndBTNTriangle, int& nEndBTNRound);

/***********************************************************
**boxID: electric box ID
, range : [0,5)
**bit: io index, range [0,15]
**state: 0 low , 1 high
control command
set robot`s electricity box digital Output IO
***********************************************************/
HANSROBOT_PRO_API int ReadElectricityBoxIO(int boxID, ST_ElectricityBoxInfo& boxState);

/***********************************************************
**boxID: electric box ID
, range : [0,5)
**bit: io index, range [0,15]
**state: 0 low , 1 high
control command
set robot`s electricity box digital Output IO
***********************************************************/
HANSROBOT_PRO_API int SetElectricityBoxDO(int boxID, int bit, int state);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**bit: io index, range [0,15]
**state: 0 low , 1 high
control command
Read robot`s electricity box digital Output IO
***********************************************************/
HANSROBOT_PRO_API int ReadElectricityBoxDO(int boxID, int bit, int* state);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**bit: io index, range [0,15]
**state: 0 low , 1 high
control command
Read robot`s electricity box digital Input IO
***********************************************************/
HANSROBOT_PRO_API int ReadElectricityBoxDI(int boxID, int bit, int* state);

/***********************************************************
**boxID: electric box ID
, range : [0,5)
**bit: io index, range [10000,10063]
**state: 0 low , 1 high
control command
set robot`s Internal modbus coil
***********************************************************/
HANSROBOT_PRO_API int SetInternalMBCoil(int boxID, int bit, int state);

/***********************************************************
**boxID: electric box ID
, range : [0,5)
**bit: io index, range [10000,10063]
**state: 0 low , 1 high
control command
read robot`s Internal modbus coil
***********************************************************/
HANSROBOT_PRO_API int ReadInternalMBCoil(int boxID, int bit, int* state);


/***********************************************************
**boxID: electric box ID , range : [0,5)
**bit: io index, range [0,15]
**dbConveyorPos: conveyor position ,unit mm
control command
Read robot`s ConveyorPos mm
***********************************************************/
HANSROBOT_PRO_API int ReadConveyorPos(int boxID, int rbtID, 
	double* dbConveyorPos,int* nConveyorCount);

/***********************************************************
**boxID: electric box ID , range : [0,5)
**bit: io index, range [0,15]
**state: 0 low , 1 high
control command
Read robot`s electricity box digital Input IO
***********************************************************/
HANSROBOT_PRO_API int GetPoseInterpolate(int boxID, int rbtID, double dFromX, double dFromY, double dFromZ,
	double dFromRx, double dFromRy, double dFromRz, double dToX, double dToY, double dToZ,
	double dToRx, double dToRy, double dToRz, double dAlpha, double& dResultX, double& dResultY, double& dResultZ,
	double& dResultRx, double& dResultRy, double& dResultRz);

// enum EN_SDODataType
// {
// 	enSDODataType_BYTE = 0,
// 	enSDODataType_WORD,
// 	enSDODataType_DWORD,
// };
HANSROBOT_PRO_API int ReadSDO(int boxID, int nSlaveID, int nIndex, int nSubIndex, int nSDODataType, int& nValue);
HANSROBOT_PRO_API int WriteSDO(int boxID, int nSlaveID, int nIndex, int nSubIndex, int nSDODataType, int nValue);

HANSROBOT_PRO_API int ReadModSig(int boxID, const char* szName, int& nValue);
HANSROBOT_PRO_API int ReadMultiModSig(int boxID, const char* szName,int nSize, int* nValue);
HANSROBOT_PRO_API int WriteModSig(int boxID, const char* szName, int nValue);

HANSROBOT_PRO_API int ReadForceVel(int boxID, int rbtID, double& dVelX, double& dVelY, double& dVelZ,double& dVelRx, double& dVelRy, double& dVelRz);
HANSROBOT_PRO_API int ReadForceValue(int boxID, int rbtID, double& dForceX, double& dForceY, double& dForceZ,double& dForceRx, double& dForceRy, double& dForceRz);
HANSROBOT_PRO_API int SetFCCmdType(int boxID, int rbtID, int nType);
HANSROBOT_PRO_API int SetFCFrameType(int boxID, int rbtID, int nType);
HANSROBOT_PRO_API int SetFCEnable(int boxID, int rbtID, int nEnable);
HANSROBOT_PRO_API int SetFCFreedom(int boxID, int rbtID, int nFreedomX, int nFreedomY, int nFreedomZ,int nFreedomRx, int nFreedomRy, int nFreedomRz);
HANSROBOT_PRO_API int SetFCCmdForce(int boxID, int rbtID, double dForceX, double dForceY, double dForceZ,double dForceRx, double dForceRy, double dForceRz);
HANSROBOT_PRO_API int SetForceControlState(int boxID, int rbtID, int nState);

#endif



/************************************************************************/
/*Page 3                                                                */
/************************************************************************/

#pragma once
#include "HansRobot_Pro.h"
#include "HansRobot_STDef.h"
#include <string>
using namespace std;

HANSROBOT_PRO_API string HRIF_GetReturnDCSStateCodeStr(int nErrCode);
HANSROBOT_PRO_API int HRIF_GetReturnDCSStateCode(int nSysState);
HANSROBOT_PRO_API int HRIF_InitxmlrpcClient(string strIP, int nPort);
HANSROBOT_PRO_API int HRIF_DeInitxmlrpcClient();

HANSROBOT_PRO_API int HRRPC_ReadModbusDevices(string& strModbusDevices);
HANSROBOT_PRO_API int HRRPC_SetModbusDevices(string strModbusDevices);
HANSROBOT_PRO_API int HRRPC_ReadModSig(const char* szName, int& nValue);
//HANSROBOT_PRO_API int HRRPC_ReadMultiModSig(const char* szName, int nSize, int* nValue);
HANSROBOT_PRO_API int HRRPC_WriteModSig(const char* szName, int nValue);

HANSROBOT_PRO_API int HRRPC_GetSerialPortList(string& SerialPortList);

HANSROBOT_PRO_API int HRRPC_ReadRobotiqConfig(int& nEnable,int& nInterfaceType,string& strPort,string& strModel/*2F-85 2F-140 Hand-E*/);

HANSROBOT_PRO_API int HRRPC_ReadForceSensorConfig(int& nEnable, int& nInterfaceType, string& strUAR, string& strIP, int& nPort);
HANSROBOT_PRO_API int HRRPC_SetForceSensorConfig(int nEnable, int nInterfaceType, string strUAR, string strIP, int nPort);
HANSROBOT_PRO_API int HRRPC_ReadForceSensorRotation(double& dbRotation);
HANSROBOT_PRO_API int HRRPC_SetForceSensorRotation(double dbRotation);

HANSROBOT_PRO_API int HRRPC_ReadEndType(int& nVal);

HANSROBOT_PRO_API int HRRPC_ReadModbusSlavePort(int& nPort);

HANSROBOT_PRO_API int HRRPC_ReadUILock(int& nVal);
HANSROBOT_PRO_API int HRRPC_SetUILock(int nVal);

HANSROBOT_PRO_API int HRRPC_ReadToolMotionState(int& nState);
HANSROBOT_PRO_API int HRRPC_SetToolMotionState(int nState);

HANSROBOT_PRO_API int HRRPC_ReadEmergencyHandleTime(int& nSec);
HANSROBOT_PRO_API int HRRPC_ReadEmergencyMonitor(bool& bVal);
HANSROBOT_PRO_API int HRRPC_SetEmergencyMonitor(bool bVal);

HANSROBOT_PRO_API int HRRPC_ReadEmergencyForResetIO(bool& bVal);
HANSROBOT_PRO_API int HRRPC_SetEmergencyForResetIO(bool bVal);

HANSROBOT_PRO_API int HRRPC_ReadEmergencyForResetIOState(int& nVal);
HANSROBOT_PRO_API int HRRPC_SetEmergencyForResetIOState(int bVal);

HANSROBOT_PRO_API int HRRPC_ReadSafetyGuardMonitor(bool& bVal);
HANSROBOT_PRO_API int HRRPC_SetSafetyGuardMonitor(bool bVal);

HANSROBOT_PRO_API int HRRPC_ReadSafetyGuardHold(bool& bVal);
HANSROBOT_PRO_API int HRRPC_SetSafetyGuardHold(bool bVal);

HANSROBOT_PRO_API int HRRPC_ReadSwitchOnMonitor(bool& bVal);
HANSROBOT_PRO_API int HRRPC_SetSwitchOnMonitor(bool bVal);

HANSROBOT_PRO_API int HRRPC_ReadEnableTeachModeButton(bool& bVal);
HANSROBOT_PRO_API int HRRPC_SetEnableTeachModeButton(bool bVal);

HANSROBOT_PRO_API int HRRPC_ReadSwitchOnWithBlackout(bool& bVal);
HANSROBOT_PRO_API int HRRPC_SetSwitchOnWithBlackout(bool bVal);

HANSROBOT_PRO_API int HRRPC_ReadOutofSpaceWithBlackout(bool& bVal);
HANSROBOT_PRO_API int HRRPC_SetOutofSpaceWithBlackout(bool bVal);

HANSROBOT_PRO_API int HRRPC_ReadConveyorConfig(int nRbtID, int& nDirection, int& nEnableSimulate, double& dbSimulateSpeed);
HANSROBOT_PRO_API int HRRPC_SetConveyorConfig(int nRbtID, int nDirection, int nEnableSimulate, double dbSimulateSpeed);

HANSROBOT_PRO_API int HRRPC_ReadInpositionConfig(int nRbtID, double& dbJointError, double& dbCartesianError, int& nTimeout);
HANSROBOT_PRO_API int HRRPC_SetInpositionConfig(int nRbtID, double dbJointError, double dbCartesianError, int nTimeout);

HANSROBOT_PRO_API int HRRPC_ReadNetworkIP(
	string&szFriendlyName, string&szGUID, string&szDescription, int& nAutomaticllyIP,
	int& AddrByte1, int& AddrByte2, int& AddrByte3, int& AddrByte4,
	int& MaskByte1, int& MaskByte2, int& MaskByte3, int& MaskByte4,
	int& GatewayByte1, int& GatewayByte2, int& GatewayByte3, int& GatewayByte4);
HANSROBOT_PRO_API int HRRPC_SetNetworkIP(
	string szFriendlyName, string szGUID, int nAutomaticllyIP,
	int AddrByte1, int AddrByte2, int AddrByte3, int AddrByte4,
	int MaskByte1, int MaskByte2, int MaskByte3, int MaskByte4,
	int GatewayByte1, int GatewayByte2, int GatewayByte3, int GatewayByte4);

HANSROBOT_PRO_API int HRRPC_ReadAutoTurnRunScript(bool& bVal);
HANSROBOT_PRO_API int HRRPC_SetAutoTurnRunScript(bool bVal);

HANSROBOT_PRO_API int HRRPC_ReadElectricityBoxState(ST_ElectricityBoxInfo& boxState);
HANSROBOT_PRO_API int HRRPC_ReadScriptState(ST_RbtScriptState& ScriptState);
HANSROBOT_PRO_API int HRRPC_ReadControllerState(ST_ControllerStatu& ControllerState);
HANSROBOT_PRO_API int HRRPC_ReadRobotRealTimeStatus(int nRbtID, ST_RbtRealTimeStatuV4& rbtState);

HANSROBOT_PRO_API int HRRPC_ReadInternalMBCoils(vector<bool>& coils);
HANSROBOT_PRO_API int HRRPC_SetInternalMBCoil(int bit, int state);

HANSROBOT_PRO_API int HRRPC_ReadConveyorPos(int nRbtID, double& dbConveyorPos, int& nConveyor);

HANSROBOT_PRO_API int HRRPC_ReadActualPos(int nRbtID,
	double& J1, double& J2, double& J3,
	double& J4, double& J5, double& J6,
	double& X, double& Y, double& Z,
	double& RX, double& RY, double& RZ);

HANSROBOT_PRO_API int HRRPC_GrpEnable(int nRbtID);
HANSROBOT_PRO_API int HRRPC_GrpDisable(int nRbtID);
HANSROBOT_PRO_API int HRRPC_GrpStop(int nRbtID);
HANSROBOT_PRO_API int HRRPC_GrpReset(int nRbtID);
HANSROBOT_PRO_API int HRRPC_GrpInterrupt(int nRbtID);
HANSROBOT_PRO_API int HRRPC_GrpContinue(int nRbtID);
HANSROBOT_PRO_API int HRRPC_MoveJ(int nRbtID, double J1, double J2, double J3, double J4, double J5, double J6);
HANSROBOT_PRO_API int HRRPC_MoveL(int nRbtID, double X, double Y, double Z, double Rx, double Ry, double Rz);
HANSROBOT_PRO_API int HRRPC_MoveJShortJog(int nRbtID, int nAxisID, int nDerection);
HANSROBOT_PRO_API int HRRPC_MoveLShortJog(int nRbtID, int nAxisID, int nDerection);

HANSROBOT_PRO_API int HRRPC_ReadAcsSafeSpaceLimit(int nRbtID, AcsSafeSpaceLimit& ssLimit);
HANSROBOT_PRO_API int HRRPC_ReadJointSpeedLimit(int nRbtID, RobotMotionLimits& rbtMotionLimits);

/************************************************************************/
/*Page 4                                                                */
/************************************************************************/
#pragma once

#define Max_RTOSRbtCnt 2
#define Max_RTOSSlaveCnt 8

typedef enum _EN_RbtType_V4
{
	enRobotTypeV4_Null = 0,
	enRobotTypeV4_1KW,
	enRobotTypeV4_Delta1300,
	enRobotTypeV4_MK1_5,
	enRobotTypeV4_Elfin,
	enRobotTypeV4_ElfinV5,
	enRobotTypeV4_Cyton,
	enRobotTypeV4_ElfinKPA,
}EN_RbtType_V4;

typedef enum _EN_SlaveType_V4
{
	enSlaveTypeV4_Null = 0,
	enSlaveTypeV4_Elfin,
	enSlaveTypeV4_BeckHoffDO,
	enSlaveTypeV4_BeckHoffDI,
	enSlaveTypeV4_Third,
	enSlaveTypeV4_HMEnd,
	enSlaveTypeV4_Scara,
	enSlaveTypeV4_ElfinKPA,
	enSlaveTypeV4_ElfinKPA485,
}EN_SlaveType_V4;

/************************************************************************/
/*Page 5                                                                */
/************************************************************************/
#pragma once

#define MAX_ScriptSTRING 1024
#define MAX_FUNCIOTNPARAMTERS 20
//////////////////////////////////////////////////////////////////////////
//String类型枚举
//enStringType_Seldefine=自定义
//enStringType_Select=选择的
//////////////////////////////////////////////////////////////////////////

enum EN_WayPointOption
{
	enWayPoint_MoveJ = 0,
	enWayPoint_MoveL,
	enWayPoint_Seek,
	enWayPoint_sequence,
	enWayPoint_Parameter,
	enWayPoint_count
};

enum EN_SeekOption
{
	enSeek_Stack = 0,
	enSeek_DeStack = 1,
	enSeek_Pallet = 2,
};
enum EN_PalletpatternOption
{
	enPattern_Line = 0,
	enPattern_Square = 1,
	enPattern_Box = 2,
	enPattern_List = 3,
};

enum EN_StringType
{
	enStringType_Custom = 0,
	enStringType_GlobalVal,
	enStringType_Spilt,
	enStringType_CustomHex,
	enStringType_GlobalValHex,
	enStringType_Cnt
};

typedef struct _ST_SEND_DATA_
{
	EN_StringType nStringType;
	char sSocketName[MAX_ScriptSTRING];
	char sSendValue[MAX_ScriptSTRING];
	int arrDataType[MAX_FUNCIOTNPARAMTERS];	//0 for const ,1 for var

	char sSpilt[MAX_ScriptSTRING];//分隔符

	char sHeadText[MAX_ScriptSTRING];
	char sEndText[MAX_ScriptSTRING];

	int nDataNum;
}ST_Send_Data;

typedef struct _ST_RECEIVE_DATA_
{
	EN_StringType nStringType;
	char sSocketName[MAX_ScriptSTRING];
	char sReceiveValue[MAX_ScriptSTRING];
	char sSpilt[MAX_ScriptSTRING];//分隔符

	int nDataNum;
	char sBeginText[MAX_ScriptSTRING];
	char sEndText[MAX_ScriptSTRING];

}ST_Receive_Data;


/************************************************************************/
/*Page 6                                                                */
/************************************************************************/
#pragma once

#include "HansRobot_RTOSDef.h"
#include "HR_IFPro.h"


#define ElectricityBoxIO_CNT 8
#define ElectricityBoxIO_Total 16
#define ElectricityBoxIO_Extern 4
#define ElectricityBoxANALOG_CNT 2
#define EndIOCount 4

#define EndANALOG_CNT 2
#define AxisCount 6

enum EN_EndModbusBTN
{
	enModbusBTN_Stop=0,
	enModbusBTN_Add,
	enModbusBTN_Start,
	enModbusBTN_FreeDrive,
	enModbusBTN_Count
};

enum EN_ConfigInIOFunction
{
	enConfInIOFunc_Power = 0,
	enConfInIOFunc_ClearError,
	enConfInIOFunc_SwitchToAuto,
	enConfInIOFunc_PauseScript,
	enConfInIOFunc_DisableSafetyMoncee,//弃用
	enConfInIOFunc_StartFreeDriver,
	enConfInIOFunc_OptTeachMode,//不显示，enConfInIOFunc_StartFreeDriver的边沿触发
	enConfInIOFunc_Stop,
	enConfInIOFunc_ContinueScript,
	enConfInIOFunc_RunScript,
	ConfigInIOFunctionCnt,
};

enum EnConfigInputOperation
{//功能都要成对出现
	enEnableRobot				= 1 << 0,
	enDisableRobot				= 1 << 1,

	enResetRobot				= 1 << 2,
	enResetRobotReserved		= 1 << 3,

	enChangeToAuto				= 1 << 4,
	enChangeToManual			= 1 << 5,

	enPauseScript				= 1 << 6,
	enPauseScriptReserved		= 1 << 7,

	enDisableSafetyMoncee		= 1 << 8,
	enEnableSafetyMoncee		= 1 << 9,

	enStartTeachMode			= 1 << 10,
	enStopTeachMode				= 1 << 11,

	enOptTeachMode				= 1 << 12,
	enOptTeachModeReserved		= 1 << 13,

	enStopRobot					= 1 << 14,
	enStopRobotReserved			= 1 << 15,	

	enContinueScript			= 1 << 16,
	enContinueScriptReserved	= 1 << 17,

	enRunScript					= 1 << 18,
	enRunScriptReserved			= 1 << 19
};

#pragma pack(push,1)
typedef struct _ST_AxisVal_
{
	double axis[AxisCount];
}ST_AxisVal;
typedef struct _ST_AxisFloatVal_
{
	float axis[AxisCount];
}ST_AxisFloatVal;
typedef struct _ST_ControllerConfig_
{
	int nRTOSCycleTime;
	int nRTOSRobotsCNT;
	int nRTOSSlaveCNT;
	int SlavePDOSize[Max_RTOSSlaveCnt];
	char szENIName[256];
	struct RobotConfig
	{
		EN_RbtType_V4 rbtType;
		int nRbtSlavesCNT;
		EN_SlaveType_V4 slavesType[Max_RTOSSlaveCnt];
		ST_AxisVal axisGearRatio;
	}rbt[Max_RTOSRbtCnt];
}ST_ControllerConfig;
/************************************************************************/
/*                                                                      */
/************************************************************************/

typedef struct _ST_ElectricityBoxAnalogOutput_
{
	int nAnalogMode;
	double dbAnalog;
}ST_ElectricityBoxAnalogOutput;

typedef struct _ST_ElectricityBoxInfo_
{
//	int nEmergencyStop;
	int nRemotePoweroff;
	int nDInputIO[ElectricityBoxIO_CNT];
	int nSInputIO[ElectricityBoxIO_CNT];
	int nExterInputIO[ElectricityBoxIO_Extern];//一代电箱有4个
	int nDOutputIO[ElectricityBoxIO_CNT];
	int nSOutputIO[ElectricityBoxIO_CNT];
	int nExterOutputIO[ElectricityBoxIO_Extern];//一代电箱有4个
	ST_ElectricityBoxAnalogOutput stAnalogOutput[ElectricityBoxANALOG_CNT];
	double dbAnalogInput[ElectricityBoxANALOG_CNT];
	double dbRobotSrcVoltage;
	double dbRobotSupplyVoltage;
	double dbRobotSupplyCurrent;
	double dbIOVoltage;
	double dbIOCurrent;
}ST_ElectricityBoxInfo;

typedef struct _ST_ControllerStatu_
{
	int nDCSStatus;
	int nSimulation;
	int nRtosStatus;
	int nMasterStatus;
	int nSlaveDropState;
	int nSlaveDropID;
	int nFrameResponseErrorCnt;
	int nDCSLockUI;
}ST_ControllerStatu;

typedef struct _ST_RbtScriptState_
{
	int nCurUDMDefIndex[enScriptThread_CNT];
	int nScriptRunState;//0xXY X是否运行，Y是否暂停
}ST_RbtScriptState;

typedef struct _ST_RbtRealTimeStatuV4_
{
	int nRSIndex_Moving;
	int nRSIndex_Standby;
	int nRSIndex_Power;
	int nRSIndex_Error;
	int nRSIndex_Breaking;
	int nRSIndex_CmdStop;
	int nRSIndex_AcsTolerance;
	int nRSIndex_PcsTolerance;
	int nRSIndex_Holding;
	int nErrCode;
	int nErrAxisID;
	int nOSS_Direction;
	int nOSS_CoordType;
	int nInDIO[EndIOCount];
	int nOutDIO[EndIOCount];
	int nModbusBTN[enModbusBTN_Count];
	double dbAnalogInput[EndANALOG_CNT];
	double dOverride;
	ST_AxisVal rbtPosACS;
	ST_AxisVal rbtPosPCS;
	ST_AxisVal rbtPosPCSForBase;
	ST_AxisVal rbtPosTOOL;
	ST_AxisVal motorCurrent;
	ST_AxisVal motorTemperature;
	ST_AxisVal motorVoltage;
}ST_RbtRealTimeStatuV4;
/************************************************************************/
/*                                                                      */
/************************************************************************/

typedef struct _ST_RbtRealTimeMBStatu
{
	float dOverride;//register 1000
	ST_AxisFloatVal rbtPosACS;//register 1002
	ST_AxisFloatVal rbtPosPCS;//register 1014
	ST_AxisFloatVal rbtPosPCSForBase;//register 1026
	ST_AxisFloatVal rbtPosTOOL;//register 1038
	ST_AxisFloatVal motorCurrent;//register 1050
	ST_AxisFloatVal motorTemperature;//register 1062
	ST_AxisFloatVal motorVoltage;//register 1074

	unsigned short nDCSStatus;//register 1086

	unsigned short nScriptRunState;//register 1087。  0xXY X是否运行，Y是否暂停

	unsigned short nRSIndex_Moving;//register 1088
	unsigned short nRSIndex_Standby;//register 1089
	unsigned short nRSIndex_Power;//register 1090
	unsigned short nRSIndex_Error;//register 1091
	unsigned short nRSIndex_Breaking;//register 1092
	unsigned short nRSIndex_CmdStop;//register 1093
	//unsigned short nRSIndex_AcsTolerance;//register 1094
	//unsigned short nRSIndex_PcsTolerance;//register 1095
	unsigned short nRSIndex_Holding;//register 1094
	unsigned short nErrCode;//register 1095
	unsigned short nErrAxisID;//register 1096
	unsigned short nOSS_Direction;//register 1097
	unsigned short nOSS_CoordType;//register 1098

	unsigned short nInDIO[EndIOCount];//register 1099
	unsigned short nOutDIO[EndIOCount];//register 1103
	unsigned short nModbusBTN[enModbusBTN_Count];//register 1107
	float dbEndAnalogInput[EndANALOG_CNT];//register 1111

	//电箱
	unsigned short nDInputIO[ElectricityBoxIO_CNT];//register 1115
	unsigned short nSInputIO[ElectricityBoxIO_CNT];//register 1123
	unsigned short nExterInputIO[ElectricityBoxIO_Extern];//register 1131。一代电箱有4个
	unsigned short nDOutputIO[ElectricityBoxIO_CNT];//register 1135
	unsigned short nSOutputIO[ElectricityBoxIO_CNT];//register 1143
	unsigned short nExterOutputIO[ElectricityBoxIO_Extern];//register 1151。一代电箱有4个
	struct _ST_BoxAnalogOutputMB_
	{
		unsigned short nAnalogMode;
		float dbAnalog;
	}stBoxAnalogOutput[ElectricityBoxANALOG_CNT];//register 1155
	float dbBoxAnalogInput[ElectricityBoxANALOG_CNT];//register 1161
	float dbRobotSrcVoltage;//register 1165
	float dbRobotSupplyVoltage;//register 1167
	float dbRobotSupplyCurrent;//register 1169
	float dbIOVoltage;//register 1171
	float dbIOCurrent;//register 1173	
	unsigned short nCurUDMDefIndex[enScriptThread_CNT];//register 1175
	unsigned short usESTOPHandleTime;//register 1181
	unsigned short nRemotePoweroff;//register 1182
	unsigned short nSimulation;//register 1183
	unsigned short nRtosStatus;//register 1184
	unsigned short nMasterStatus;//register 1185
	unsigned short nSlaveDropState;//register 1186
	unsigned short nSlaveDropID;//register 1187
	unsigned short nFrameResponseErrorCnt;//register 1188
	unsigned short usUILock;//register 1189
	unsigned short usRbtForceSensorMode;//register 1190
}ST_RbtRealTimeMBStatu;
#pragma pack(pop)



/************************************************************************/
/*Page 7                                                                */
/************************************************************************/
#pragma once
#include <string>
using namespace std;

#define USERSHM_MAGIC 0x48414E53//"HANSROBOT"
#define Max_RobotCnt 5
#define Max_SlaveCnt 8
#define Max_PDOLen 1024

#define KCMD_IntCnt 32
#define KCMD_DoubleCnt 660
#define DigitalINCnt 18
#define DigitalOutCnt 18
#define AnalogINCnt 2
#define Max_OutIOCnt 8
#define Max_InIOCnt  8

#define Invalid_RobotID -1
#define FuncName_Len 100
#define FuncRltMax_Len 1*1024			// 函数返回结果最大长度
#define MachineOriginCnt 6
#define Max_AxisOutIOCnt 16
#define Max_ActualAxisCnt 6
#define Max_Securitylevel 6	//最安全的等级 
#define MAX_CUSTOM_TCP_CNT 16			// 最大的自定义刀具坐标数量
#define MAX_CUSTOM_USERCOORD_CNT 16		// 最大的自定义用户坐标数量
#define FIXED_TCP_CNT 1					// 固定的TCP坐标数量
#define FIXED_USERCOORD_CNT 1			// 固定的用户坐标
#define JOINTMOTIONLIMITS_CNT 3*Max_ActualAxisCnt	// 关节运动限制参数数量
#define PCSMOTIONLIMITS_CNT 6			// 空间运动限制参数数量
#define MAX_SYSTEMPOINTS_CNT 128
#define Max_RobotNameLen 32				// 机器人名称的最大长度
#define TOTALIO_CNT 20
#define BoxCommIO_CNT 8
#define BoxConfigIO_CNT 8
#define SERIALANALOG_CNT 2
#define SERIALANALOGDO_CNT 2
#define KINEMATICSPARAM_CNT 4			//6自由度Elfin运动学模型参数数量
#define DYNAMICSPARAM_CNT 13			//6自由度Elfin动力学模型参数数量
#define ElfinSlaveCnt 3				//Elfin从站数量
#define PcsTransCnt 3					//XYZ
#define Max_DragMovePointCnt 50  
#define Max_DoubleCnt 300
#define MaxBaseMountingAngle 180
#define MinBaseMountingAngle -180
#define Max_DragMoveDOCnt 4

#define InternalMBIO_CNT 64

#define MAX_SLAVE_CNT 0x0C			// 每台机器人支持的最大从站数量
// 机器手类型
#define RbType_1KW			"1KW"
#define RbType_DELTA1300	"DELTA-1300"
#define RbType_MK1_5		"MK1-5"
#define RbType_Elfin		"MK2-5"
#define RbType_ElfinV5		"ElfinV5"
#define RbType_CYTON		"CYTON"
#define RbType_ElfinKPA	"ElfinKPA"

// 机器手DH参数
#define RobotDH_Elfin3		"Elfin3"
#define RobotDH_Elfin5		"Elfin5"
#define RobotDH_Elfin10		"Elfin10"

#define Coord_Name 64


// 浮点数比较系数
#define DOUBLE_EPSINON 0.005

struct _ST_ScriptFuncInfo;
// 声明函数运行结果回调函数
typedef int(*PFuncRunResultCallBack)(const _ST_ScriptFuncInfo& funcInfo);

enum LockOwnerType
{
	LockOWNER_FREE = 0,		///< The buffer is free for writing
	LockOWNER_RTOS,			///< The RTOS side owns the buffer
	LockOWNER_WINDOWS		///< The Windows side owns the buffer
};

//安全停车模式
enum enSafetyMode
{
	enSafetyMode_StopAndDisable = 0,
	enSafetyMode_StopAndStandby,
	enSafetyMode_StopAndBlackout,
	enSafetyMode_Count
};
/************************************************************************/
/*                                                                      */
/************************************************************************/
//控制器状态机
enum enControllerStatus
{
	enControllerStatus_UnInitialize = 100,
	enControllerStatus_MasterNotStart,
	enControllerStatus_MasterStarting,
	enControllerStatus_MasterClosing,
	enControllerStatus_MasterDisconnected,
	enControllerStatus_InitializeSlaves,
	enControllerStatus_InitializeSlavesFail,
	enControllerStatus_Normal,
	enControllerStatus_Simulation,
	enControllerStatus_Count
};
/************************************************************************/
/*                                                                      */
/************************************************************************/
enum EN_SDODataType
{
	enSDODataType_BYTE = 0,
	enSDODataType_WORD,
	enSDODataType_DWORD,
};

/***********************************************************
**点动方向
**enJogDirec_Negative=负方向
**enJogDirec_Positive=正方向
***********************************************************/
enum EN_JogDirection
{
	enJogDirec_Negative = 0,
	enJogDirec_Positive,
};

/***********************************************************
**坐标系类型定义
**enCoordType_Acs=角度坐标
**enCoordType_Pcs=空间坐标
***********************************************************/
enum EN_CoordType
{
	enCoordType_Acs = 0,
	enCoordType_Pcs,
};

/*************************************************************************
**命令索引定义
**enCmdIndexSet=机器人设置命令索引
**enCmdIndexMove=机器人运动命令索引
**enCmdIndexMotionCtrl=机器人运动控制命令索引
**enCmdIndexPrior=优先命令索引
*************************************************************************/
enum EN_CmdIndex
{
	enCmdIndexSet = 1,
	enCmdIndexMove,
	enCmdIndexMotionCtrl,
	enCmdIndexController,
	enCmdIndexHighFrequency,
	enCmdIndexPrior,
};

enum EN_CmdController
{
	enCmdController_StartMaster = 0,
	enCmdController_CloseMaster,
	enCmdController_StartSmulation,
	enCmdController_CloseSmulation,
	enCmdControllerCnt,
};

enum EN_EndButton
{
	enEndButton_Stop=0,
	enEndButton_Add,
	enEndButton_Run,
	enEndButton_TeachMode
};

/*************************************************************************
**Set命令子索引定义
**enCmdSet_Override=设置速度比
**enCmdSet_Position=设置轴位置
**enCmdSet_HomePosition=设置机械原点
**enCmdSet_OutIOState=设置输出IO状态
**enCmdSet_PcsSafeSpaceLimit=设置空间坐标安全空间
**enCmdSet_AcsSafeSpaceLimit=设置角度坐标安全空间
**enCmdSet_KinematicCoordinate=设置刀具坐标系
**enCmdSet_UserCoordinate=设置用户坐标系
**enCmdSet_ConveyorScale=设置传送带比例
**enCmdSet_PcsPosition=设置空间坐标位置
**enCmdSet_TrackingSwitch=设置跟随开关,
**enCmdSet_SpeedUp=速度增加
**enCmdSet_SpeedDown=速度减少
**enCmdSet_MoveJointMotionLimits=运动关节运动限制
**enCmdSet_MovePcsMotionLimits=运动空间运动限制
**enCmdSet_StopJointMotionLimits=刹车关节运动限制
**enCmdSet_StopPcsMotionLimits=刹车空间运动限制
**enCmdSet_ElfinBrakeOnly=Elfin Brake Only
**enCmdSet_ElfinHomeStill=Elfin Home Still
**enCmdSet_ElfinRelay=Elfin Relay
**enCmdSet_CollideStopThresholds=碰撞检测各关节阈值
**enCmdSet_CollideStopPayload=碰撞检测负载参数
**enCmdSet_AssistiveModeCollideStopThresholds=零力示教时碰撞检测各关节阈值
**enCmdSetCnt=运动命令数量，永远为枚举的最后一个成员
*************************************************************************/
enum EN_CmdSet
{
	enCmdSet_Override = 0,
	enCmdSet_HomePosition,
	enCmdSet_OutIOState,
	enCmdSet_PcsSafeSpaceLimit,
	enCmdSet_AcsSafeSpaceLimit,
	enCmdSet_KinematicCoordinate,
	enCmdSet_UserCoordinate,
	enCmdSet_ConveyorScale,
	enCmdSet_ConveyorConfig,
	enCmdSet_PcsPosition,
	enCmdSet_TrackingSwitch,
	enCmdSet_SpeedUp,
	enCmdSet_SpeedDown,
	enCmdSet_MoveJointMotionLimits,
	enCmdSet_MovePcsMotionLimits,
	enCmdSet_StopJointMotionLimits,
	enCmdSet_StopPcsMotionLimits,
	enCmdSet_ElfinBrakeOnly,
	enCmdSet_ElfinHomeStep2,
	enCmdSet_ElfinRelay,
	enCmdSet_CollideStopThresholds,
	enCmdSet_CollideStopPayload,
	enCmdSet_AssistiveModeCollideStopThresholds,
	enCmdSet_MotorScalesSlave,
	enCmdSet_StartAssistiveMode,
	enCmdSet_CloseAssistiveMode,
	enCmdSet_StartFeedforwardMode,
	enCmdSet_CloseFeedforwardMode,
	enCmdSet_Velocitythresholds,
	enCmdSet_FrictionCompensationFactor,
	enCmdSet_ToolCoordinateMotion,
	enCmdSet_TorqueConstant,
	enCmdSet_MaxEfficiency,
	enCmdSet_BaseMountingAngle,
	enCmdSet_SetKinematicsParameters,
	enCmdSet_DynamicsParameters1,
	enCmdSet_DynamicsParameters2,
	enCmdSet_DynamicsParameters3,
	enCmdSet_DynamicsParameters4,
	enCmdSet_DynamicsParameters5,
	enCmdSet_DynamicsParameters6,
	enCmdSet_InitRobotDynamics,
	enCmdSet_GravityParam,
	enCmdSet_CollideStopLevel,
	enCmdSet_TeachModeType,
	enCmdSet_ClearStopState,
	enCmdSet_DragMoveParameters,
	enCmdSet_SetDragMoveStatus,
	enCmdSet_SetDragMovePoints,
	enCmdSet_MaxActutorCurrents,
	enCmdSet_SetDragMoveIO,
	enCmdSet_SetHMDragMoveStatus,
	enCmdSet_SetHMDragMovePoints,
	enCmdSet_SetDragMoveCalculateType,
	enCmdSet_SetBlendingStatus,
	enCmdSet_PushBlending,
	enCmdSet_ForceSensorMode,
	enCmdSet_ForceSensorParams,
	enCmdSet_ForceSensorStatus,
	enCmdSet_ForceSensorUpdate,
	enCmdSet_InitServoParams,
	enCmdSet_UpdateServoJointParams,
	enCmdSet_FeedForwardParam,
	enCmdSet_FeedforwardVelocityMode,
	enCmdSet_PowerOffNoStop,
	enCmdSet_OperationMode,
	enCmdSet_SetKPASlaveInit,
	enCmdSet_CollisionStopMode,
	enCmdSet_CollisionStopThresholds,
	enCmdSet_DampConstant,
	enCmdSet_Inposition,
	enCmdSet_ForceControlState,
	enCmdSet_ForceControl,
	enCmdSet_FCCmdType,
	enCmdSet_FCFrameType,
	enCmdSet_FCEnable,
	enCmdSet_FCFreedom,
	enCmdSet_FCCmdForce,
	enCmdSet_FCCmdPID,
	enCmdSet_IsAbsoluteEncode,
	enCmdSet_ElectricPower,
	enCmdSet_PrintSDKTime,
	enCmdSet_SetMoveZStatus,
	enCmdSet_SetMoveZPoints,
	enCmdSet_AssistiveCheckTime,
	enCmdSet_FrictionWithTemperature,
	enCmdSet_FTSearchSpeed,
	enCmdSetCnt,
};

/*************************************************************************
**MotionCtrl命令子索引定义
**enCmdMotionCtrl_GrpPowerOn=使能
**enCmdMotionCtrl_GrpPowerOff=去使能
**enCmdMotionCtrl_GrpStop=停止
**enCmdMotionCtrl_GrpReset=复位
**enCmdMotionCtrl_GrpInterrupt=暂停
**enCmdMotionCtrl_GrpContinue=继续
**enCmdMotionCtrlCnt=运动命令数量，永远为枚举的最后一个成员
*************************************************************************/
enum EN_CmdMotionCtrl
{
	enCmdMotionCtrl_GrpPowerOn = 0,
	enCmdMotionCtrl_GrpPowerOff,
	enCmdMotionCtrl_GrpStop,
	enCmdMotionCtrl_GrpReset,
	enCmdMotionCtrl_GrpInterrupt,
	enCmdMotionCtrl_GrpContinue,
	enCmdMotionCtrl_GrpStopMoving,
	enCmdMotionCtrlCnt,
};

/*************************************************************************
**Move命令子索引定义
**enCmdMove_DirAcs=角度坐标运动
**enCmdMove_DirPcs=空间坐标点到点运动
**enCmdMove_LinePcs=空间坐标直线运动
**enCmdMove_Circular=圆弧运动
**enCmdMove_LongJogAcs=角度坐标轴长点动
**enCmdMove_LongJogPcs=空间坐标轴长点动
**enCmdMove_PNP=门型运动
**enCmdMove_MultiPoint=多点运动
**enCmdMoveCnt=运动命令数量，永远为枚举的最后一个成员
*************************************************************************/
enum EN_CmdMove
{
	enCmdMove_DirAcs=0,		
	enCmdMove_DirPcs,
	enCmdMove_LinePcs,
	enCmdMove_MoveCircularCnt,
	enCmdMove_LongJogAcs,
	enCmdMove_LongJogPcs,
	enCmdMove_PNP,
	enCmdMove_LinePcsBlending,
	enCmdMove_MultiPoint,
	enCmdMove_MoveCircular,
	enCmdMove_DirAcsBlending,
	enCmdMove_LineAndCirc,
	enCmdMove_Speed,
	enCmdMove_Vex,
	enCmdMove_DragMove,
	enCmdMove_LineBelt,
	enCmdMove_ShortJogAcs,
	enCmdMove_ShortJogPcs,
	enCmdMove_MoveRelJ,
	enCmdMove_MoveRelL,
	enCmdMoveCnt,
};

/***************************************************************
**控制命令
**enCmdMaster_StartMaster = 启动主站,
**enCmdMaster_CloseMaster =关闭主站,
**enCmdMaster_StartMasterNotInit = 启动主站不初始化从站,
**enCmdMaster_WriteSDO = 写SDO,
**enCmdMaster_ReadSDO	= 读SDO,
***************************************************************/
enum EN_CmdMaster
{
	enCmdMaster_StartMaster = 0,
	enCmdMaster_CloseMaster,
	enCmdMaster_StartMasterNotInit,
	enCmdMaster_WriteSDO,
	enCmdMaster_ReadSDO,
	enCmdMaster_ExitController,
	enCmdMaster_DropNoCloseMaster,
	enCmdMaster_SetKPAInitSlaveFailed,
	enCmdMaster_Cnt,
};

/***************************************************************
**IO类型
**enIOType_Out=数字输出
**enIOType_In=数字输入
***************************************************************/
enum EN_IOType
{
	enIOType_Out = 0,
	enIOType_In,
};

/***************************************************************
**控制器类型
**enCtrlerType_ECWIN=ECWIN
**enCtrlerType_KW=KW
**enCtrlerType_ZWX=众为兴
**enCtrlerType_HansKR=KR
**enCtrlerTypeCnt=控制器类型数量
***************************************************************/
enum EN_CtrlerType
{
	enCtrlerType_ECWIN = 0,
	enCtrlerType_KW,
	enCtrlerType_ZWX,
	enCtrlerType_HansKR,
	enCtrlerTypeCnt,
};
/***************************************************************
**主站操作
**en=未启动
**enMOpt_Start=启动
**enMOpt_Close=关闭
***************************************************************/
enum EN_MasterOpt
{
	enMOpt_Start = 0,
	enMOpt_Close,
};
/***************************************************************
**主站状态
**enMS_NotStart=未启动
**enMS_Starting=启动中
**enMS_Started=已启动
***************************************************************/
enum EN_MasterState
{
	enMS_NotStart = 0,
	enMS_Starting,
	enMS_Started,
	enMS_Err,
	enMS_LoadLicenseErr,
	enMS_InitErr,
	enMS_SlaveCntErr,
	enMS_ENIFileErr,
};

/***************************************************************
**从站状态
**enMS_NotStart=未启动
**enMS_Starting=启动中
**enMS_Started=已启动
***************************************************************/
enum EN_SlavesState
{
	enSlaves_NotInit = 0,
	enSlaves_Initializing,
	enSlaves_Initialized,
	enSlaves_InitErr
};

/***************************************************************
**控制器状态
**enCS_NotStart=未启动
**enCS_Started=已启动
***************************************************************/
enum EN_ControllerState
{
	enCS_NotStart = 0,
	enCS_Started,
};
/***************************************************************
**Initphase状态
**enCS_NotInitphase
**enCS_Initphase
***************************************************************/
enum EN_RobotMasterFail
{
	enCS_MasterSuccess = 0,
	enCS_MasterFail,
};

/***************************************************************
**机器人状态索引
**enRSIndex_Moving=运动状态
**enRSIndex_Standby=Standby状态
**enRSIndex_Power=使能状态
**enRSIndex_Error=错误状态
**enRSIndex_Reserve1=备用1
**enRSIndex_Reserve2=备用2
**enRSIndex_Reserve3=备用3
**enRobotStatusCnt=机器人状态数量
***************************************************************/
enum EN_RobotStateIndex
{
	enRSIndex_Moving = 0,
	enRSIndex_Standby,
	enRSIndex_Power,
	enRSIndex_Error,
	enRSIndex_Breaking,
	enRSIndex_CmdStop,
	enRSIndex_AcsTolerance,
	enRSIndex_PcsTolerance,
	enRSIndex_Holding,
	enRSIndex_ForceControlInPlace,
	enRobotStatusCnt,
};

/***************************************************************
**空间坐标实际位置索引
**enPAPIndex_x=x
**enPAPIndex_y=y
**enPAPIndex_z=z
**enPAPIndex_a=a
**enPAPIndex_b=b
**enPAPIndex_c=c
***************************************************************/
enum EN_PcsActualPosIndex
{
	enPAPIndex_x = 0,
	enPAPIndex_y,
	enPAPIndex_z,
	enPAPIndex_a,
	enPAPIndex_b,
	enPAPIndex_c,
	enPcsActualPosCnt,
};

/***************************************************************
**角度坐标实际位置索引
**enAAPIndex_j1=j1
**enAAPIndex_j2=j2
**enAAPIndex_j3=j3
**enAAPIndex_j4=j4
**enAAPIndex_j5=j5
**enAAPIndex_j6=j6
***************************************************************/
enum EN_AcsActualPosIndex
{
	enAAPIndex_j1 = 0,
	enAAPIndex_j2,
	enAAPIndex_j3,
	enAAPIndex_j4,
	enAAPIndex_j5,
	enAAPIndex_j6
};

/***************************************************************
**错误码索引
**enAAPIndex_j1=j1
**enAAPIndex_j2=j2
**enAAPIndex_j3=j3
**enAAPIndex_j4=j4
**enAAPIndex_j5=j5
**enAAPIndex_j6=j6
**enErrorCodeCnt=角度坐标实际位置数量
***************************************************************/
enum EN_ErrorCodeIndex
{
	enErrCodeIndex_Axis1 = 0,
	enErrCodeIndex_Axis2,
	enErrCodeIndex_Axis3,
	enErrCodeIndex_Axis4,
	enErrCodeIndex_Axis5,
	enErrCodeIndex_Axis6,
	enErrorCodeCnt,
};

/***************************************************************
**空间坐标安全空间索引
**enPSSIndex_xPositive=x轴正向最大值
**enPSSIndex_xNegative=x轴负向最小值
**enPSSIndex_yPositive=y轴正向最大值
**enPSSIndex_yNegative=y轴负向最小值
**enPSSIndex_zPositive=z轴正向最大值
**enPSSIndex_zNegative=z轴负向最小值
**enPSSIndex_aPositive=a轴正向最大值
**enPSSIndex_aNegative=a轴负向最小值
**enPSSIndex_bPositive=b轴正向最大值
**enPSSIndex_bNegative=b轴负向最小值
**enPSSIndex_cPositive=c轴正向最大值
**enPSSIndex_cNegative=c轴负向最小值
**enPcsSafeSpaceCnt=空间坐标安全空间数量
***************************************************************/
enum EN_PcsSafeSpaceIndex
{
	enPSSIndex_xPositive = 0,
	enPSSIndex_xNegative,
	enPSSIndex_yPositive,
	enPSSIndex_yNegative,
	enPSSIndex_zPositive,
	enPSSIndex_zNegative,
	enPSSIndex_aPositive,
	enPSSIndex_aNegative,
	enPSSIndex_bPositive,
	enPSSIndex_bNegative,
	enPSSIndex_cPositive,
	enPSSIndex_cNegative,
	enPcsSafeSpaceCnt,
};

/***************************************************************
**角度坐标安全空间索引
**enASSIndex_j1Positive=j1轴正向最大值
**enASSIndex_j1Negative=j1轴负向最小值
**enASSIndex_j2Positive=j2轴正向最大值
**enASSIndex_j2Negative=j2轴负向最小值
**enASSIndex_j3Positive=j3轴正向最大值
**enASSIndex_j3Negative=j3轴负向最小值
**enASSIndex_j4Positive=j4轴正向最大值
**enASSIndex_j4Negative=j4轴负向最小值
**enASSIndex_j5Positive=j5轴正向最大值
**enASSIndex_j5Negative=j5轴负向最小值
**enASSIndex_j6Positive=j6轴正向最大值
**enASSIndex_j6Negative=j6轴负向最小值
**enAcsSafeSpaceCnt=空间坐标安全空间数量
***************************************************************/
enum EN_AcsSafeSpaceIndex
{
	enASSIndex_j1Positive = 0,
	enASSIndex_j1Negative,
	enASSIndex_j2Positive,
	enASSIndex_j2Negative,
	enASSIndex_j3Positive,
	enASSIndex_j3Negative,
	enASSIndex_j4Positive,
	enASSIndex_j4Negative,
	enASSIndex_j5Positive,
	enASSIndex_j5Negative,
	enASSIndex_j6Positive,
	enASSIndex_j6Negative,
	enAcsSafeSpaceCnt,
};

/***************************************************************
**机器人类型
**enRobotType_Null=未知
**enRobotType_Scara600=Scara
**enRobotType_Delta1300=Delta
**enRobotType_MK1_5=MK1
**enRobotType_MK2_5=MK2
**enRobotType_Cyton=Cyton
***************************************************************/
typedef enum EN_RobotType
{
	enRobotType_Null=0,
	enRobotType_1KW,
	enRobotType_Delta1300,
	enRobotType_MK1_5,
	enRobotType_Elfin,
	enRobotType_ElfinV5,
	enRobotType_Cyton,
	enRobotType_ElfinKPA,
}ENRobotType;
/***************************************************************
**机器人类型
**enRobotType_Null=未知
**enRobotType_Scara600=Scara
**enRobotType_Delta1300=Delta
**enRobotType_MK1_5=MK1
**enRobotType_MK2_5=MK2
**enRobotType_Cyton=Cyton
***************************************************************/
typedef enum EN_EndType
{
	enEndType_Null = 0,
	enEndType_4IN_4OUT,
	enEndType_3IN_3OUT_4BTN,
}ENEndType;

typedef enum EN_SlaveType_V3
{
	enSlaveTypeV3_Null = 0,
	enSlaveTypeV3_Elfin,
	enSlaveTypeV3_BeckHoffDO,
	enSlaveTypeV3_BeckHoffDI,
	enSlaveTypeV3_Third,
	enSlaveTypeV3_HMEnd, 
	enSlaveTypeV3_Scara,
	enSlaveTypeV3_ElfinKPA,
	enSlaveTypeV3_ElfinKPA485,
	enSlaveTypeV3_ElfinKPA2IN1,
	enSlaveTypeV3_Force,
}ENSlaveType_V3;

typedef enum EN_RobotModel
{
	enRobotModel_Elfin3KG = 1,
	enRobotModel_Elfin5KG,
	enRobotModel_Elfin10KG,
	enRobotModel_Elfin5KGExtended,
	enRobotModel_Elfin20KG
}ENRobotModel;



/***************************************************************
**Modbus末端IO的Index
***************************************************************/
typedef enum ENModebusIORxIndex
{
	// 	enModebusIORxIndex_DoCommand = 0x3100,
	// 	enModebusIORxIndex_SlaveID,
	// 	enModebusIORxIndex_ConnectParam,
	// 	enModebusIORxIndex_TimeStamp,
	// 	enModebusIORxIndex_Function,
	// 	enModebusIORxIndex_RegisterAddr,
	// 	enModebusIORxIndex_RegisterCount,
	// 	enModebusIORxIndex_RegisterData0,
	// 	enModebusIORxIndex_RegisterData1,
	// 	enModebusIORxIndex_RegisterData2,
	// 	enModebusIORxIndex_RegisterData3,
	// 	enModebusIORxIndex_RegisterData4,
	// 	enModebusIORxIndex_RegisterData5,
	// 	enModebusIORxIndex_RegisterData6,
	// 	enModebusIORxIndex_RegisterData7,
	// 	enModebusIORxIndex_RegisterData8,
	// 	enModebusIORxIndex_RegisterData9,
	// 	enModebusIORxIndex_RegisterData10,
	// 	enModebusIORxIndex_RegisterData11,
	// 	enModebusIORxIndex_RegisterData12,
	// 	enModebusIORxIndex_RegisterData13,
	// 	enModebusIORxIndex_RegisterData14,
	// 	enModebusIORxIndex_RegisterData15,

	enModebusIORxIndex_DoCommand = 0x3100,
	enModebusIORxIndex_SlaveID_Function,
	enModebusIORxIndex_RegisterAddr_Count,
	enModebusIORxIndex_TimeStamp,
	enModebusIORxIndexReserve_RegisterData1,
	enModebusIORxIndexReserve_RegisterData2,
	enModebusIORxIndexReserve_RegisterData3,
	enModebusIORxIndexReserve_RegisterData4,
	enModebusIORxIndexReserve_RegisterData5,
	enModebusIORxIndexReserve_RegisterData6,
	enModebusIORxIndexReserve_RegisterData7,
	enModebusIORxIndexReserve_RegisterData8,
	enModebusIORxIndex_RegisterData1,
	enModebusIORxIndex_RegisterData2,
	enModebusIORxIndex_RegisterData3,
	enModebusIORxIndex_RegisterData4,
	enModebusIORxIndex_RegisterData5,
	enModebusIORxIndex_RegisterData6,
	enModebusIORxIndex_RegisterData7,
	enModebusIORxIndex_RegisterData8,
	enModebusIORxIndex_RegisterData9,
	enModebusIORxIndex_RegisterData10,
	enModebusIORxIndex_RegisterData11,
	enModebusIORxIndex_RegisterData12,
	enModebusIORxIndex_RegisterData13,
	enModebusIORxIndex_RegisterData14,
	enModebusIORxIndex_RegisterData15,
	enModebusIORxIndex_RegisterData16,

	enEndIORxIndex_DoCommand,
	enEndIORxIndexReserve_RegisterData1,
	enEndIORxIndexReserve_RegisterData2,
	enEndIORxIndexReserve_RegisterData3,
	enEndIORxIndexReserve_RegisterData4,
	enEndIORxIndexReserve_RegisterData5,
	enEndIORxIndexReserve_RegisterData6,
	enEndIORxIndexReserve_RegisterData7,
	enEndIORxIndexReserve_RegisterData8,
	enEndIORxIndex_RegisterData1,
	enEndIORxIndex_RegisterData2,
	enEndIORxIndex_RegisterData3,
	enEndIORxIndex_RegisterData4,
	enEndIORxIndex_RegisterData5,
	enEndIORxIndex_RegisterData6,
	enEndIORxIndex_RegisterData7,
	enEndIORxIndex_RegisterData8,
	enEndIORxIndex_RegisterData9,
	enEndIORxIndex_RegisterData10,

}ENModebusIORxIndex;

/***************************************************************
**Modbus末端IO的Index
***************************************************************/
typedef enum ENModebusIOTxIndex
{
// 	enModebusIOTxIndex_TimeStamp = 0x2100,
// 	enModebusIOTxIndex_ErrorCode,
// 	enModebusIOTxIndex_State,
// 	enModebusIOTxIndex_RegisterCount,
// 	enModebusIOTxIndex_RegisterData1,
// 	enModebusIOTxIndex_RegisterData2,
// 	enModebusIOTxIndex_RegisterData3,
// 	enModebusIOTxIndex_RegisterData4,
// 	enModebusIOTxIndex_RegisterData5,
// 	enModebusIOTxIndex_RegisterData6,
// 	enModebusIOTxIndex_RegisterData7,
// 	enModebusIOTxIndex_RegisterData8,
// 	enModebusIOTxIndex_RegisterData9,
// 	enModebusIOTxIndex_RegisterData10,
// 	enModebusIOTxIndex_RegisterData11,
// 	enModebusIOTxIndex_RegisterData12,
// 	enModebusIOTxIndex_RegisterData13,
// 	enModebusIOTxIndex_RegisterData14,
// 	enModebusIOTxIndex_RegisterData15,

	enModebusIOTxIndex_State = 0x2100,
	enModebusIOTxIndex_ErrorCode,
	enModebusIOTxIndex_RegisterCount,
	enModebusIOTxIndex_TimeStamp,
	enModebusIOTxIndexReserve_RegisterData1,
	enModebusIOTxIndexReserve_RegisterData2,
	enModebusIOTxIndexReserve_RegisterData3,
	enModebusIOTxIndexReserve_RegisterData4,
	enModebusIOTxIndexReserve_RegisterData5,
	enModebusIOTxIndexReserve_RegisterData6,
	enModebusIOTxIndexReserve_RegisterData7,
	enModebusIOTxIndexReserve_RegisterData8,
	enModebusIOTxIndex_RegisterData1,
	enModebusIOTxIndex_RegisterData2,
	enModebusIOTxIndex_RegisterData3,
	enModebusIOTxIndex_RegisterData4,
	enModebusIOTxIndex_RegisterData5,
	enModebusIOTxIndex_RegisterData6,
	enModebusIOTxIndex_RegisterData7,
	enModebusIOTxIndex_RegisterData8,
	enModebusIOTxIndex_RegisterData9,
	enModebusIOTxIndex_RegisterData10,
	enModebusIOTxIndex_RegisterData11,
	enModebusIOTxIndex_RegisterData12,
	enModebusIOTxIndex_RegisterData13,
	enModebusIOTxIndex_RegisterData14,
	enModebusIOTxIndex_RegisterData15,
	enModebusIOTxIndex_RegisterData16,

	enEndIOTxIndex_State,
	enEndIOTxIndex_ErrorCode,
	enEndIOTxIndexReserve_RegisterData1,
	enEndIOTxIndexReserve_RegisterData2,
	enEndIOTxIndexReserve_RegisterData3,
	enEndIOTxIndexReserve_RegisterData4,
	enEndIOTxIndexReserve_RegisterData5,
	enEndIOTxIndexReserve_RegisterData6,
	enEndIOTxIndexReserve_RegisterData7,
	enEndIOTxIndexReserve_RegisterData8,
	enEndIOTxIndex_RegisterData1,
	enEndIOTxIndex_RegisterData2,
	enEndIOTxIndex_RegisterData3,
	enEndIOTxIndex_RegisterData4,
	enEndIOTxIndex_RegisterData5,
	enEndIOTxIndex_RegisterData6,
	enEndIOTxIndex_RegisterData7,
	enEndIOTxIndex_RegisterData8,
	enEndIOTxIndex_RegisterData9,
	enEndIOTxIndex_RegisterData10,
}ENModebusIOTxIndex;

typedef enum EN_SdoDateType
{
	enSdoDateType_Byte = 0,
	enSdoDateType_Word,
	enSdoDateType_DWord,
}ENSdoDateType;

/***************************************************************
**RobotIQ参数
*************************************************************/
#define   MAX_REGIESTER_VALUE     255
#define   MIN_REGIESTER_VALUE       0
#define   MODBUSROBOTIQSLAVEID     0x9
enum
{
	en_gOBJ = 0,
	en_gSTA,
	en_gGTO,
	en_gACT,
	en_kFLT,
	en_gFLT,
	en_gPR,
	en_gPO,
	en_gCU,
	en_gCount
};

enum enGripperMotionType
{
	enMotionType_UnInitialize = 0,
	enMotionType_PosAdd,
	enMotionType_PosPlus,
	enMotionType_ForceAdd,
	enMotionType_ForcePlus,
	enMotionType_SpeedAdd,
	enMotionType_SpeedPlus,
};

enum EN_GripperRegister
{
	enGripperActionRequest = 1000,
	enGripperPosRegister,
	enGripperSpeedForceRegister,

	enGripperStatusRegister = 2000,
	enGripperFaultAndPosRequestRegister,
	enGripperPosAndCurrentRegister
};

enum EN_GripperAction
{
	enRACT = 1,
	enRGTO = 1 << 3,
	enRATR = 1 << 4,
	enRRAD = 1 << 5
};

enum EN_GripperStatus
{
	enGACT = 1,
	enGGTO = 1 << 3,
	enGGSTA = 1 << 4,		//2 bit
	enGGOBJ = 1 << 6		//2 bit
};

enum EN_GOBJ
{
	enMotionTowardsRequest,
	enStoppedBeforeOpenningRequest,
	enStoppedBeforeClosingRequest,
	enStoppedAtRequestPosition
};
/*************************************************************** /

/*************************************************************************
**机器人内核命令结构体
**usStateMachine=状态机
**usInstructionGUID=消息ID
**sCmdIndex=命令索引
**sCmdSubIndex=命令子索引
**nOptResult=操作结果
**nParam=整形参数数组
**dbParam=浮点型参数数组
*************************************************************************/
typedef struct _ST_KernelCmd
{
	unsigned short usStateMachine;
	unsigned short usInstructionGUID;
	short sCmdIndex;
	short sCmdSubIndex;
	int nOptResult;
	int nParam[KCMD_IntCnt];
	double dbParam[KCMD_DoubleCnt];

	_ST_KernelCmd() :usStateMachine(0), usInstructionGUID(0), sCmdIndex(0), 
		sCmdSubIndex(0),nOptResult(0)
	{
		memset(nParam, 0, sizeof(nParam));
		memset(dbParam, 0, sizeof(dbParam));
	}
}ST_KernelCmd;

typedef struct _ST_SDOCmd
{
	unsigned short usStateMachine;
	unsigned short usReadOrWrite;
	unsigned short nSlaveID;
	unsigned short nIndex;
	unsigned short nSubIndex;
	unsigned short nType;
	unsigned short nError;
	int nPDOValue;

	_ST_SDOCmd() :usStateMachine(0), usReadOrWrite(0), 
		nSlaveID(0), nIndex(0), nSubIndex(0), nPDOValue(0), nType(0), nError(0)
	{
	}
}ST_SDOCmd;

/*************************************************************************
**机器人内核命令区域
**MoveCmdArea=运动命令区域
**MotionCtrlCmdArea=运动控制命令区域
**SetCmdArea=设置命令区域
*************************************************************************/
typedef struct _ST_KernelCmdArea
{
	ST_KernelCmd MoveCmdArea;
	ST_KernelCmd MotionCtrlCmdArea;
	ST_KernelCmd SetCmdArea;
	ST_KernelCmd HighFrequencyCmdArea;
}ST_KernelCmdArea;

/*************************************************************************
**Robot_Status=机器人状态区
**Digital_In=数字输入
**Digital_Out=数字输出
**Analog_In=模拟量输入
**OutSafeSpace_CoordType=超出安全空间的坐标类型
**OutSafeSpace_AxisID=超出安全空间轴ID
**OutSafeSpace_Direction=超出安全空间轴方向，0=负向；1=正向
**Error_AxisID=机器人报错轴ID
**EndatPosMulti=多圈绝对位置
**Actual_Position_ACS=角度坐标实际位置[m,m,m,rad,rad,rad]
**Actual_Position_PCS=空间坐标实际位置[m,m,m,rad,rad,rad]
**Command_Position_ACS=角度坐标命令位置[m,m,m,rad,rad,rad]
**Command_Position_PCS=空间坐标命令位置[m,m,m,rad,rad,rad]
**Actual_Override=实际速度比
**ConveyorValue=传送带编码器的值
**Robot_Type=机器人类型
*************************************************************************/
typedef struct _ST_StatusArea
{	
	int Robot_Status[enRobotStatusCnt];		
	int Digital_In[DigitalINCnt];			
	int Digital_Out[DigitalOutCnt];
	int Analog_In[AnalogINCnt];
	int OutSafeSpace_CoordType;
//	int OutSafeSpace_AxisID;
	int OutSafeSpace_Direction;
	int Error_AxisID;
	int Error_Code;
	int EndatPosMulti[Max_ActualAxisCnt];	
	double Actual_Position_ACS[Max_ActualAxisCnt];
	double Actual_Position_PCS[Max_ActualAxisCnt];
	double Command_Position_ACS[Max_ActualAxisCnt];
	double Command_Position_PCS[Max_ActualAxisCnt];
	double Actual_Override;	
//	char Robot_Type[Max_RobotNameLen];
	double Command_Joint_Velocity[Max_ActualAxisCnt];
	double Command_Joint_Acceleration[Max_ActualAxisCnt];
	double Actual_Motor_Current[Max_ActualAxisCnt];
	double Command_Motor_Current[Max_ActualAxisCnt];
	double Sensed_Joint_Torque[Max_ActualAxisCnt]; 
	double Estimated_Joint_Torque[Max_ActualAxisCnt];
	double Command_PCS_Velocity[Max_ActualAxisCnt];
	double Actual_PCS_Velocity[Max_ActualAxisCnt];
	int ToolCoordinateMotionFlag;
	int CurrentControlMode;	
	int ForceSensorState;
	int CheckRequestSupplyBuffer;	
	int KPASlaveInitState;
	int nTrackingState;//是否关闭跟随,0为彻底关闭
	int ConveyorValueCount;//传送带位置count
	double ConveyorVelocity;//传送带速度 mm/s
	double ConveyorValuemm;//传送带位置mm
	double Acceleration[Max_ActualAxisCnt];
	double Decleration[Max_ActualAxisCnt];
	double dForceVel[Max_ActualAxisCnt];
	double dForceValue[Max_ActualAxisCnt];
	double dPhysicsPower;
	double dMomentum;
	_ST_StatusArea() : OutSafeSpace_CoordType(0), OutSafeSpace_Direction(0), Error_AxisID(0),
		Error_Code(0), Actual_Override(0), ToolCoordinateMotionFlag(0), CurrentControlMode(0),
		ForceSensorState(0), CheckRequestSupplyBuffer(0), KPASlaveInitState(0),
		nTrackingState(0), ConveyorValueCount(0), ConveyorVelocity(0), ConveyorValuemm(0),
		dPhysicsPower(0), dMomentum(0)
	{
		memset(Robot_Status, 0, sizeof(Robot_Status));
		memset(Digital_In, 0, sizeof(Digital_In));
		memset(Digital_Out, 0, sizeof(Digital_Out));
		memset(Analog_In, 0, sizeof(Analog_In));

		memset(EndatPosMulti, 0, sizeof(EndatPosMulti));
		memset(Actual_Position_ACS, 0, sizeof(Actual_Position_ACS));
		memset(Actual_Position_PCS, 0, sizeof(Actual_Position_PCS));
		memset(Command_Position_ACS, 0, sizeof(Command_Position_ACS));
		memset(Command_Position_PCS, 0, sizeof(Command_Position_PCS));

		memset(Command_Joint_Velocity, 0, sizeof(Command_Joint_Velocity));
		memset(Command_Joint_Acceleration, 0, sizeof(Command_Joint_Acceleration));
		memset(Actual_Motor_Current, 0, sizeof(Actual_Motor_Current));
		memset(Command_Motor_Current, 0, sizeof(Command_Motor_Current));
		memset(Sensed_Joint_Torque, 0, sizeof(Sensed_Joint_Torque));
		memset(Estimated_Joint_Torque, 0, sizeof(Estimated_Joint_Torque));
		memset(Command_PCS_Velocity, 0, sizeof(Command_PCS_Velocity));
		memset(Actual_PCS_Velocity, 0, sizeof(Actual_PCS_Velocity));
		memset(Acceleration, 0, sizeof(Acceleration));
		memset(Decleration, 0, sizeof(Decleration));
		memset(dForceVel, 0, sizeof(dForceVel));
		memset(dForceValue, 0, sizeof(dForceValue));
	}
}ST_StatusArea;

/*************************************************************************
**机器人错误区
*************************************************************************/
typedef struct _ST_ErrorArea
{
	short ErrorCode[enErrorCodeCnt];
	short ErrorCode1[enErrorCodeCnt];
	_ST_ErrorArea()
	{
		memset(ErrorCode, 0, sizeof(ErrorCode));
		memset(ErrorCode1, 0, sizeof(ErrorCode1));
	}
}ST_ErrorArea;

/*************************************************************************
**DCS状态区-由DCS共享数据给控制器
*************************************************************************/
typedef struct _ST_DCSStatusArea
{
	double dMotorTemperature[Max_ActualAxisCnt];
	double dReserve1[Max_ActualAxisCnt];
	double dReserve2[Max_ActualAxisCnt];
	double dReserve3[Max_ActualAxisCnt];
	double dReserve4[Max_ActualAxisCnt];
	double dReserve5[Max_ActualAxisCnt];
	_ST_DCSStatusArea()
	{
		memset(dMotorTemperature, 0, sizeof(dMotorTemperature));
		memset(dReserve1, 0, sizeof(dReserve1));
		memset(dReserve2, 0, sizeof(dReserve2));
		memset(dReserve3, 0, sizeof(dReserve3));
		memset(dReserve4, 0, sizeof(dReserve4));
		memset(dReserve5, 0, sizeof(dReserve5));
	}
}ST_DCSStatusArea;

/*************************************************************************
**ECWIN内存状态结构体
**dwTotalPhys=bytes total physical RAM
**dwAvailPhys=bytes available physical RAM
**dwLargestFreeBlock=bytes in largest free block on Win32 heap
**dwTotalVirtual=bytes total virtual address space
**dwAvailVirtual=bytes available virtual address space
**dwMemoryLoad=memory load
*************************************************************************/
typedef struct _ST_MemoryStatus
{
	unsigned long dwTotalPhys;
	unsigned long dwAvailPhys;
	unsigned long dwLargestFreeBlock;
	unsigned long dwTotalVirtual;
	unsigned long dwAvailVirtual;
	unsigned long dwMemoryLoad;
	_ST_MemoryStatus() :dwTotalPhys(0), dwAvailPhys(0),
		dwLargestFreeBlock(0), dwTotalVirtual(0), dwAvailVirtual(0),
		dwMemoryLoad(0)
	{

	}
	_ST_MemoryStatus(const _ST_MemoryStatus& obj)
	{
		*this = obj;
	}
	_ST_MemoryStatus& operator=(const _ST_MemoryStatus& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(_ST_MemoryStatus* pThis, const _ST_MemoryStatus& obj)
	{
		pThis->dwTotalPhys = obj.dwTotalPhys;
		pThis->dwAvailPhys = obj.dwAvailPhys;
		pThis->dwLargestFreeBlock = obj.dwLargestFreeBlock;
		pThis->dwTotalVirtual = obj.dwTotalVirtual;
		pThis->dwAvailVirtual = obj.dwAvailVirtual;
		pThis->dwMemoryLoad = obj.dwMemoryLoad;
	}
}ST_MemoryStatus;

/***********************************************************
**机器人信息结构体
**id=当前机器人ID，从0开始计数，第一台机器人的ID为0
**type=机器人类型
**name=机器人名称
***********************************************************/
typedef struct _ST_RobotsConfig
{
	int Id;
	string Name;
	ENRobotType Type;
	_ST_RobotsConfig() :Id(0), Type(enRobotType_Null)
	{
	}
	_ST_RobotsConfig(const _ST_RobotsConfig& obj)
	{
		*this = obj;
	}
	_ST_RobotsConfig& operator=(const _ST_RobotsConfig& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(_ST_RobotsConfig* pThis, const _ST_RobotsConfig& obj)
	{
		pThis->Id = obj.Id;
		pThis->Type = obj.Type;
		pThis->Name = obj.Name;
	}
}ST_RobotsConfig;
/***********************************************************
**机器人信息结构体
***********************************************************/
typedef struct _UserShm_SlavesInfo
{
	ENSlaveType_V3 SlaveType;
	int nPDOSize;
	_UserShm_SlavesInfo() :nPDOSize(0), SlaveType(enSlaveTypeV3_Null)
	{
	}
}UserShm_SlavesInfo;

typedef struct _UserShm_RobotConfigInfo
{
	int useConveyor;
	ENRobotType Type;
	char Robot_Type[Max_RobotNameLen];
	char szModel[131072];//Mode file content 1M
	unsigned short usSlavesCNT;
	UserShm_SlavesInfo slaves[Max_SlaveCnt];
	int GearRatio[MAX_SLAVE_CNT];
	_UserShm_RobotConfigInfo() :useConveyor(0), Type(enRobotType_Null)
	{
		memset(Robot_Type, 0, sizeof(Robot_Type));
		memset(szModel, 0, sizeof(szModel));
		memset(GearRatio, 0, sizeof(GearRatio));
	}
}UserShm_RobotConfigInfo;
/***********************************************************
**控制器配置结构体
***********************************************************/
typedef struct _UserShm_RobotsConfig
{
	int nInitFlag;//HANS
	int nRTOSWatchDog;
	int nDCSWatchDog;
	int nCycleTime;
	int nRTOSErr;//主站error
	int nRTOSStatus;//控制器状态 EN_SystemState
	char szENIFile[256];
	char szENI[131072];//ENI file content 1M
	unsigned short usRobotCNT;
	UserShm_RobotConfigInfo robots[Max_RobotCnt];
	_UserShm_RobotsConfig() :nInitFlag(0), nRTOSWatchDog(0), nDCSWatchDog(0), nCycleTime(0),
		nRTOSErr(0), nRTOSStatus(0), usRobotCNT(0)
	{
		memset(szENIFile, 0, sizeof(szENIFile));
		memset(szENI, 0, sizeof(szENI));
	}
}UserShm_RobotsConfig;

typedef struct _ST_RobotArea
{
	ST_KernelCmdArea kcArea;
	ST_StatusArea stArea;
	ST_ErrorArea erArea;
	ST_DCSStatusArea dcsArea;
//	int nType;
}ST_RobotArea;

typedef struct _ST_UserShm
{
	UserShm_RobotsConfig robotConfig;
	ST_RobotArea robotsArea[Max_RobotCnt];
}ST_UserShm;
/*************************************************************************
**控制器内核命令结构体
**nRobotsCnt=机器人数量
**nTimeTick=时间应答计数
**cMasterStart=操作主站指令;0=无效指令;1=启动主站;2=关闭主站;3=Initphase;
**cPrintLog=打印日志开关;0=关闭打印日志;1=打开打印日志;
**cMasterState=主站的状态;false=未启动;true=已启动
**stMemoryStatus=内存状态
*************************************************************************/
typedef struct _ST_ControllerCmd
{
	int nControllerMainVer;
	int nControllerSubVer;
	int nControllerMinVer;
	char cMasterState;
	char cSlaveDroped;
	int nHMDriveOperation;
	int nSlaveDropedCount;
	char cSlaveState;
	char cSimulation;
	unsigned int volatile nlock;
	int nFrameResponseErrorCnt;
	int nFrameResponseErrorMaxCnt;
	int nCurFrameResponseErrorCnt;
	ST_KernelCmd RtosCmdArea;
	ST_SDOCmd SDOCmdArea;
	ST_SDOCmd IFSDOCmdArea;
	_ST_ControllerCmd() : nControllerMainVer(0), nControllerSubVer(0), nControllerMinVer(0),
		nHMDriveOperation(0), cSlaveState(0), nlock(0), nFrameResponseErrorCnt(0),
		nFrameResponseErrorMaxCnt(0),cMasterState(enMS_NotStart), cSlaveDroped(0),
		cSimulation(0), nSlaveDropedCount(0), nCurFrameResponseErrorCnt(0)
	{
		//memset(cCtrlerStatus, 0, sizeof(cCtrlerStatus));
	}
}ST_ControllerCmd;

/*************************************************************************
**控制器内核命令结构体
**nRobotsCnt=机器人数量
**nTimeTick=时间应答计数
**cMasterStart=操作主站指令;0=无效指令;1=启动主站;2=关闭主站;3=Initphase;
**cPrintLog=打印日志开关;0=关闭打印日志;1=打开打印日志;
**cMasterState=主站的状态;false=未启动;true=已启动
**stMemoryStatus=内存状态
*************************************************************************/
typedef struct _ST_ScopeInfo
{
	double dbACSCmd[Max_ActualAxisCnt];
	double dbACSAct[Max_ActualAxisCnt];
	double dbPCSCmd[Max_ActualAxisCnt];
	double dbPCSAct[Max_ActualAxisCnt];
	double dbActVel[Max_ActualAxisCnt];
	double dbCmdVel[Max_ActualAxisCnt];
 	double dbSensed_Joint_Torque[Max_ActualAxisCnt];//检测力矩
// 	double Estimated_Joint_Torque[Max_ActualAxisCnt];//理论计算力矩
	double dbActCur[Max_ActualAxisCnt];
//	double dbCmdCur[Max_ActualAxisCnt];
	double dbActPCSVel[Max_ActualAxisCnt];
	double dbCmdPCSVel[Max_ActualAxisCnt];

	double dbCmdCur[Max_ActualAxisCnt];
	double dbFeedforwardVel[Max_ActualAxisCnt];
	double dbFeedforwardCur[Max_ActualAxisCnt];

	double dbFilACSAct[Max_ActualAxisCnt];
	double dbFilACSVel[Max_ActualAxisCnt];
	double dbFilACSAcc[Max_ActualAxisCnt];
	double dbFilJointCur[Max_ActualAxisCnt];
	double dbFilSensedJointTorque[Max_ActualAxisCnt];
	double dbFilEstimatedJointTorque[Max_ActualAxisCnt];
	double dbFilDisturbanceTorques[Max_ActualAxisCnt];
	
	double dbMomentum;
	double dPhysicsPower;//force * velocity
	double dElectricPower;//voltage * current
	_ST_ScopeInfo() :dbMomentum(0.0), dPhysicsPower(0.0), dElectricPower(0.0)
	{
		memset(dbACSCmd, 0, sizeof(dbACSCmd));
		memset(dbACSAct, 0, sizeof(dbACSAct));
		memset(dbPCSCmd, 0, sizeof(dbPCSCmd));
		memset(dbPCSAct, 0, sizeof(dbPCSAct));
		memset(dbActVel, 0, sizeof(dbActVel));
		memset(dbCmdVel, 0, sizeof(dbCmdVel));
		memset(dbSensed_Joint_Torque, 0, sizeof(dbSensed_Joint_Torque));
		memset(dbActCur, 0, sizeof(dbActCur));
		memset(dbActPCSVel, 0, sizeof(dbActPCSVel));
		memset(dbCmdPCSVel, 0, sizeof(dbCmdPCSVel));

		memset(dbCmdCur, 0, sizeof(dbCmdCur));
		memset(dbFeedforwardVel, 0, sizeof(dbFeedforwardVel));
		memset(dbFeedforwardCur, 0, sizeof(dbFeedforwardCur));

		memset(dbFilACSAct, 0, sizeof(dbFilACSAct));
		memset(dbFilACSVel, 0, sizeof(dbFilACSVel));
		memset(dbFilACSAcc, 0, sizeof(dbFilACSAcc));
		memset(dbFilJointCur, 0, sizeof(dbFilJointCur));
		memset(dbFilSensedJointTorque, 0, sizeof(dbFilSensedJointTorque));
		memset(dbFilEstimatedJointTorque, 0, sizeof(dbFilEstimatedJointTorque));
		memset(dbFilDisturbanceTorques, 0, sizeof(dbFilDisturbanceTorques));
	}
}ST_ScopeInfo;

#define SCOPE_CACHEWRITE 100
#define SCOPE_CACHEREAD 200

typedef struct _ST_ScopeCache
{
	int nScopePos;
	ST_ScopeInfo scopeData[SCOPE_CACHEREAD];
}ST_ScopeCache;

typedef struct _ST_RobotScope
{
// 	struct _ST_ScopeCacheWrite
// 	{
// 		int nScopeWrite;
// 		ST_ScopeInfo scopeWrite[SCOPE_CACHEWRITE];
// 	}scopeCacheWrite;
// 	unsigned int volatile m_dwWriteLock;
	unsigned int volatile m_dwScopeLock;
	ST_ScopeCache scopeCache;
}ST_RobotScope;

typedef struct _ST_DebugInfo
{
	ST_MemoryStatus stMemoryStatus;
	int nPDOSize;
	unsigned char m_pbyTxPdo[Max_PDOLen];
	unsigned char m_pbyRxPdo[Max_PDOLen];
	ST_RobotScope robotScope[Max_RobotCnt];
	_ST_DebugInfo() : nPDOSize(0)
	{
		memset(m_pbyTxPdo, 0, sizeof(m_pbyTxPdo));
		memset(m_pbyRxPdo, 0, sizeof(m_pbyRxPdo));
	}

}ST_DebugInfo;

typedef struct _ST_RobotsShm
{
	ST_UserShm UserShm;
	ST_ControllerCmd ControllerCmdShm;
	ST_DebugInfo DebugInfoShm;
}ST_RobotsShm;

/***********************************************************
**JointMotionLimits
***********************************************************/
typedef struct ST_JointMotionLimits
{
	double Speed[Max_ActualAxisCnt];
	double Acc[Max_ActualAxisCnt];
	double Jerk[Max_ActualAxisCnt];
	ST_JointMotionLimits()
	{
		memset(Speed, 0, sizeof(Speed));
		memset(Acc, 0, sizeof(Acc));
		memset(Jerk, 0, sizeof(Jerk));
	}

	ST_JointMotionLimits(const ST_JointMotionLimits& obj)
	{
		*this = obj;
	}

	ST_JointMotionLimits& operator=(const ST_JointMotionLimits& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(ST_JointMotionLimits* pThis, const ST_JointMotionLimits& obj)
	{
		for (int n = 0; n < Max_ActualAxisCnt; ++n)
		{
			pThis->Speed[n] = obj.Speed[n];
			pThis->Acc[n] = obj.Acc[n];
			pThis->Jerk[n] = obj.Jerk[n];
		}
	}
}JointMotionLimits;

/***********************************************************
**MotionLimits
***********************************************************/
typedef struct ST_PcsMotionLimits
{
	double Linear_Speed;
	double Linear_Acc;
	double Linear_Jerk;
	double Angular_Speed;
	double Angular_Acc;
	double Angular_Jerk;
	ST_PcsMotionLimits() :Linear_Speed(0.0), Linear_Acc(0.0), Linear_Jerk(0.0),
		Angular_Speed(0.0), Angular_Acc(0.0), Angular_Jerk(0.0)
	{

	}

	ST_PcsMotionLimits(const ST_PcsMotionLimits& obj)
	{
		*this = obj;
	}

	ST_PcsMotionLimits& operator=(const ST_PcsMotionLimits& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(ST_PcsMotionLimits* pThis, const ST_PcsMotionLimits& obj)
	{
		pThis->Linear_Speed = obj.Linear_Speed;
		pThis->Linear_Acc = obj.Linear_Acc;
		pThis->Linear_Jerk = obj.Linear_Jerk;
		pThis->Angular_Speed = obj.Angular_Speed;
		pThis->Angular_Acc = obj.Angular_Acc;
		pThis->Angular_Jerk = obj.Angular_Jerk;
	}
}PcsMotionLimits;

/***************************************************************
**模型参数结构体
**payload_mass=负载质量，单位：Kg
**payload_centerOfMass=负载质心(x,y,z)，单位：m
**thresholds=阀值，单位：Nm
**assistiveMode_thresholds=零力示教碰撞阈值，单位：Nm
***************************************************************/
typedef struct ST_RobotDynamics
{
	double m_dTorqueConstant[Max_ActualAxisCnt];					//力矩常数
	double m_dMaxEfficiency[Max_ActualAxisCnt];						//最大功率因数
	double m_dDampConstant[Max_ActualAxisCnt];						//最大功率因数
	double m_dMountingRotation;										//基座安装旋转角度
	double m_dMountingTilt;											//基座安装倾斜角度
	double m_dKinematicsParameters[KINEMATICSPARAM_CNT];			//运动学模型参数
	double m_dDynamicsParameters1[DYNAMICSPARAM_CNT];				//动力学模型参数Axis1
	double m_dDynamicsParameters2[DYNAMICSPARAM_CNT];				//动力学模型参数Axis2
	double m_dDynamicsParameters3[DYNAMICSPARAM_CNT];				//动力学模型参数Axis3
	double m_dDynamicsParameters4[DYNAMICSPARAM_CNT];				//动力学模型参数Axis4
	double m_dDynamicsParameters5[DYNAMICSPARAM_CNT];				//动力学模型参数Axis5
	double m_dDynamicsParameters6[DYNAMICSPARAM_CNT];				//动力学模型参数Axis6

	ST_RobotDynamics() :m_dMountingRotation(0.0), m_dMountingTilt(0.0)
	{
		m_dKinematicsParameters[0] = 0.0;
		m_dKinematicsParameters[1] = 0.0;
		m_dKinematicsParameters[2] = 0.0;
		m_dKinematicsParameters[3] = 0.0;

		for (int n = 0; n < Max_ActualAxisCnt; ++n)
		{
			m_dTorqueConstant[n] = 0.0;
			m_dMaxEfficiency[n] = 0.0;
			m_dDampConstant[n] = 0.0;
		}
		for (int n = 0; n < DYNAMICSPARAM_CNT; ++n)
		{
			m_dDynamicsParameters1[n] = 0.0;
			m_dDynamicsParameters2[n] = 0.0;
			m_dDynamicsParameters3[n] = 0.0;
			m_dDynamicsParameters4[n] = 0.0;
			m_dDynamicsParameters5[n] = 0.0;
			m_dDynamicsParameters6[n] = 0.0;
		}

	}

	ST_RobotDynamics(const ST_RobotDynamics& obj)
	{
		*this = obj;
	}

	ST_RobotDynamics& operator=(const ST_RobotDynamics& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(ST_RobotDynamics* pThis, const ST_RobotDynamics& obj)
	{
		pThis->m_dMountingRotation = obj.m_dMountingRotation;
		pThis->m_dMountingTilt = obj.m_dMountingTilt;
		pThis->m_dKinematicsParameters[0] = obj.m_dKinematicsParameters[0];
		pThis->m_dKinematicsParameters[1] = obj.m_dKinematicsParameters[1];
		pThis->m_dKinematicsParameters[2] = obj.m_dKinematicsParameters[2];
		pThis->m_dKinematicsParameters[3] = obj.m_dKinematicsParameters[3];

		for (int n = 0; n < Max_ActualAxisCnt; ++n)
		{
			pThis->m_dTorqueConstant[n] = obj.m_dTorqueConstant[n];
			pThis->m_dMaxEfficiency[n] = obj.m_dMaxEfficiency[n];
			pThis->m_dDampConstant[n] = obj.m_dDampConstant[n];	
		}
		for (int n = 0; n < DYNAMICSPARAM_CNT; ++n)
		{
			pThis->m_dDynamicsParameters1[n] = obj.m_dDynamicsParameters1[n];
			pThis->m_dDynamicsParameters2[n] = obj.m_dDynamicsParameters2[n];
			pThis->m_dDynamicsParameters3[n] = obj.m_dDynamicsParameters3[n];
			pThis->m_dDynamicsParameters4[n] = obj.m_dDynamicsParameters4[n];
			pThis->m_dDynamicsParameters5[n] = obj.m_dDynamicsParameters5[n];
			pThis->m_dDynamicsParameters6[n] = obj.m_dDynamicsParameters6[n];
		}
	}
}RobotDynamicsParam;

// 机器人DH参数
typedef struct ST_RobotDHK
{
	double d1;
	double d4;
	double d6;
	double a2;
	ST_RobotDHK() :
		d1(0),
		d4(0),
		d6(0),
		a2(0)
	{
		;
	}

	ST_RobotDHK(const ST_RobotDHK& obj)
	{
		*this = obj;
	}

	ST_RobotDHK& operator=(const ST_RobotDHK& obj)
	{
		copy(this, obj);
		return *this;
	}
private:
	void copy(ST_RobotDHK* pThis, const ST_RobotDHK& obj)
	{
		pThis->d1 = obj.d1;
		pThis->d4 = obj.d4;
		pThis->d6 = obj.d6;
		pThis->a2 = obj.a2;
	}
}RobotDHK, *PRobotDHK;

typedef struct ST_DragMoveDO
{
	int nPointNum;
	int nIOCnt;
	int nWaitFlag;
	int nWaitTime;
	int nAxisID[Max_DragMoveDOCnt];
	int nBit[Max_DragMoveDOCnt];
	int nState[Max_DragMoveDOCnt];

	ST_DragMoveDO() :nPointNum(0), nIOCnt(0), nWaitFlag(0), nWaitTime(0)
	{
		for (int n = 0; n < Max_DragMoveDOCnt; ++n)
		{
			nAxisID[n] = 0;
			nBit[n] = 0;
			nState[n] = 0;
		}
	}

	ST_DragMoveDO(const ST_DragMoveDO& obj)
	{
		*this = obj;
	}

	ST_DragMoveDO& operator=(const ST_DragMoveDO& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(ST_DragMoveDO* pThis, const ST_DragMoveDO& obj)
	{
		pThis->nPointNum = obj.nPointNum;
		pThis->nIOCnt = obj.nIOCnt;
		pThis->nWaitFlag = obj.nWaitFlag;
		pThis->nWaitTime = obj.nWaitTime;
		for (int n = 0; n < Max_DragMoveDOCnt; ++n)
		{
			pThis->nAxisID[n] = obj.nAxisID[n];
			pThis->nBit[n] = obj.nBit[n];
			pThis->nState[n] = obj.nState[n];
		}
	}
}DragMoveDO;


typedef struct ST_BlendingL
{
	double Axis[Max_ActualAxisCnt];
	double StartAxis[Max_ActualAxisCnt];
	ST_BlendingL()
	{
		for (int n = 0; n < Max_ActualAxisCnt; ++n)
		{
			Axis[n] = 0;
			StartAxis[n] = 0;
		}
	}

	ST_BlendingL(const ST_BlendingL& obj)
	{
		*this = obj;
	}

	ST_BlendingL& operator=(const ST_BlendingL& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(ST_BlendingL* pThis, const ST_BlendingL& obj)
	{
		for (int n = 0; n < Max_ActualAxisCnt; ++n)
		{
			pThis->Axis[n] = obj.Axis[n];
			pThis->StartAxis[n] = obj.StartAxis[n];
		}
	}
}BlendingL;


typedef struct ST_BlendingC
{
	int nCircType;
	int nCircCnt;
	int nFixposure;
	double dStartPoint[Max_ActualAxisCnt];
	double dViaPoint[Max_ActualAxisCnt];
	double dEndPoint[Max_ActualAxisCnt];
	double dCenterPoint[Max_ActualAxisCnt];
	double dDirectionPoint[Max_ActualAxisCnt];
	double arcLength;

	ST_BlendingC() :arcLength(0), nCircType(0), nCircCnt(0), nFixposure(0)
	{
		for (int n = 0; n < Max_ActualAxisCnt; ++n)
		{
			dStartPoint[n] = 0;
			dViaPoint[n] = 0;
			dEndPoint[n] = 0;
			dCenterPoint[n] = 0;
			dDirectionPoint[n] = 0;
		}
	}

	ST_BlendingC(const ST_BlendingC& obj)
	{
		*this = obj;
	}

	ST_BlendingC& operator=(const ST_BlendingC& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(ST_BlendingC* pThis, const ST_BlendingC& obj)
	{
		pThis->arcLength = obj.arcLength;
		pThis->nCircType = obj.nCircType;
		pThis->nCircCnt = obj.nCircCnt;
		pThis->nFixposure = obj.nFixposure;
		for (int n = 0; n < Max_ActualAxisCnt; ++n)
		{
			pThis->dStartPoint[n] = obj.dStartPoint[n];
			pThis->dViaPoint[n] = obj.dViaPoint[n];
			pThis->dEndPoint[n] = obj.dEndPoint[n];
			pThis->dCenterPoint[n] = obj.dCenterPoint[n];
			pThis->dDirectionPoint[n] = obj.dDirectionPoint[n];
		}
	}
}BlendingC;

typedef struct ST_PauseBlendingPoint
{
	int nPointType;
	double dMoveProgress;
	double dPausePoint[Max_ActualAxisCnt];
	BlendingL BlendingLPoint;
	BlendingC BlendingCPoint;

	ST_PauseBlendingPoint() :nPointType(0), dMoveProgress(0.0)
	{
		for (int n = 0; n < Max_ActualAxisCnt; ++n)
		{
			dPausePoint[n] = 0;
		}
	}

	ST_PauseBlendingPoint(const ST_PauseBlendingPoint& obj)
	{
		*this = obj;
	}

	ST_PauseBlendingPoint& operator=(const ST_PauseBlendingPoint& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(ST_PauseBlendingPoint* pThis, const ST_PauseBlendingPoint& obj)
	{
		pThis->nPointType = obj.nPointType;
		pThis->dMoveProgress = obj.dMoveProgress;
		pThis->BlendingLPoint = obj.BlendingLPoint;
		pThis->BlendingCPoint = obj.BlendingCPoint;
		for (int n = 0; n < Max_ActualAxisCnt; ++n)
		{
			pThis->dPausePoint[n] = obj.dPausePoint[n];
		}
	}
}PauseBlendingPoint;


/************************************************************************/
/*Page 8                                                                */
/************************************************************************/
#pragma once
//#include "../HR_ScriptData/HR_ScriptData.h"
#include "hmRb_Def.h"
#include <vector>
#include <string>
#include <map>
using std::string;
using std::map;



//////////////////////////////////////////////////////////////////////////
//IO映射结构体
//////////////////////////////////////////////////////////////////////////
typedef struct _ST_IOMap
{
public:
	int index;
	int axisID;
	int ioBit;
	int reversal;
	int enabled;
	string name;
	_ST_IOMap() :index(0), axisID(0), ioBit(0), reversal(0), enabled(0)
	{
	}

	_ST_IOMap(const _ST_IOMap& ioMap)
	{
		*this = ioMap;
	}

	_ST_IOMap& operator=(const _ST_IOMap& ioMap)
	{
		copy(this, ioMap);
		return *this;
	}
private:
	void copy(_ST_IOMap* pIOMap, const _ST_IOMap& ioMap)
	{
		pIOMap->index = ioMap.index;
		pIOMap->axisID = ioMap.axisID;
		pIOMap->ioBit = ioMap.ioBit;
		pIOMap->reversal = ioMap.reversal;
		pIOMap->enabled = ioMap.enabled;
		pIOMap->name = ioMap.name;
	}
}ST_IOMap;


enum EN_ConfigOutType
{
	EN_ConfigOutType_Enable,
	EN_ConfigOutType_Moving,
	EN_ConfigOutType_Error,
	EN_ConfigOutType_AutoRunningState,
	EN_ConfigOutType_ScriptRunningState,
	EN_ConfigOutType_ScriptHoldingState,
	EN_ConfigOutType_Disable,
	EN_ConfigOutType_Out7,
	EN_ConfigOutType_Cnt,
};

enum EnLEDColour
{
	enBlack = 0,
	enGreen,
	enRed,
	enYellow,
	enBlue,
	enLightBlue,
	enPurple,
	enWhite
};

enum EnConfigOutputBitMask
{
	enEnableMask = 1 << 4,
	enInverseMask = 1 << 5
};

//高电平 = 0；低电平 = 1；上升沿 = 2；下降沿 = 3；
enum ConfigIOMode
{
	EN_High = 0,
	EN_Low,
	EN_RisingEdge,
	EN_FallingEdge
};

enum ReadInputType
{
	enReadSerial = 0,
	enReadEndIO,
	enReadSerialAnalog,
	enReadEndAnalog,
	enReadInternalCoil,
	enReadExpendMB,
};

typedef struct _ST_ConfigInput
{
	int Index;
	int Enabled;
	int Name;
	int nTriggerType;
	int nFunction;


	_ST_ConfigInput() :
		Index(0),
		Enabled(0),
		Name(0),
		nTriggerType(0),
		nFunction(0)
	{
	}

	_ST_ConfigInput(const _ST_ConfigInput& obj)
	{
		*this = obj;
	}

	_ST_ConfigInput& operator=(const _ST_ConfigInput& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(_ST_ConfigInput* pThis, const _ST_ConfigInput& obj)
	{
		pThis->Index = obj.Index;
		pThis->Enabled = obj.Enabled;
		pThis->Name = obj.Name;
		pThis->nTriggerType = obj.nTriggerType;
		pThis->nFunction = obj.nFunction;
	}
}ST_ConfigInput;

typedef struct _ST_ConfigOutput
{
	int Index;
	int Enabled;
	int Name;
	int nFunction;

	_ST_ConfigOutput() :
		Index(0),
		Enabled(0),
		Name(0),
		nFunction(0)
	{

	}

	_ST_ConfigOutput(const _ST_ConfigOutput& obj)
	{
		*this = obj;
	}

	_ST_ConfigOutput& operator=(const _ST_ConfigOutput& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(_ST_ConfigOutput* pThis, const _ST_ConfigOutput& obj)
	{
		pThis->Index = obj.Index;
		pThis->Enabled = obj.Enabled;
		pThis->Name = obj.Name;
		pThis->nFunction = obj.nFunction;
	}
}ST_ConfigOutput;

//////////////////////////////////////////////////////////////////////////
///Modbus映射结构体
//////////////////////////////////////////////////////////////////////////
typedef struct _ST_ModbusIOMap
{
public:
	int Type;
	int Adrress;
	string Name;
	int ResFrequency;
	int ResponseTime;
	int Slave;
	int Value;
	_ST_ModbusIOMap() :Type(0), Adrress(0), Name(""), ResFrequency(0), ResponseTime(0), Slave(0), Value(0)
	{
	}

	_ST_ModbusIOMap(const _ST_ModbusIOMap& ModbusMap)
	{
		*this = ModbusMap;
	}

	_ST_ModbusIOMap& operator=(const _ST_ModbusIOMap& ModbusMap)
	{
		copy(this, ModbusMap);
		return *this;
	}
private:
	void copy(_ST_ModbusIOMap* pModbusMap, const _ST_ModbusIOMap& ModbusMap)
	{
		pModbusMap->Type = ModbusMap.Type;
		pModbusMap->Adrress = ModbusMap.Adrress;
		pModbusMap->Name = ModbusMap.Name;
		pModbusMap->ResFrequency = ModbusMap.ResFrequency;
		pModbusMap->ResponseTime = ModbusMap.ResponseTime;
		pModbusMap->Slave = ModbusMap.Slave;
		pModbusMap->Value = ModbusMap.Value;
	}
}ST_ModbusIOMap;

typedef struct ST_PcsSafeSpaceLimt
{
	double xPositiveLimit;
	double xNegativeLimit;
	double yPositiveLimit;
	double yNegativeLimit;
	double zPositiveLimit;
	double zNegativeLimit;
	double aPositiveLimit;
	double aNegativeLimit;
	double bPositiveLimit;
	double bNegativeLimit;
	double cPositiveLimit;
	double cNegativeLimit;
	ST_PcsSafeSpaceLimt() :xPositiveLimit(0.0), xNegativeLimit(0.0),
		yPositiveLimit(0.0), yNegativeLimit(0.0),
		zPositiveLimit(0.0), zNegativeLimit(0.0),
		aPositiveLimit(0.0), aNegativeLimit(0.0),
		bPositiveLimit(0.0), bNegativeLimit(0.0),
		cPositiveLimit(0.0), cNegativeLimit(0.0)
	{
		;
	}
	ST_PcsSafeSpaceLimt(const ST_PcsSafeSpaceLimt& obj)
	{
		*this = obj;
	}

	ST_PcsSafeSpaceLimt& operator=(const ST_PcsSafeSpaceLimt& obj)
	{
		copy(this, obj);
		return *this;
	}
private:
	void copy(ST_PcsSafeSpaceLimt* pThis, const ST_PcsSafeSpaceLimt& obj)
	{
		pThis->xNegativeLimit = obj.xNegativeLimit;
		pThis->xPositiveLimit = obj.xPositiveLimit;
		pThis->yNegativeLimit = obj.yNegativeLimit;
		pThis->yPositiveLimit = obj.yPositiveLimit;
		pThis->zNegativeLimit = obj.zNegativeLimit;
		pThis->zPositiveLimit = obj.zPositiveLimit;
		pThis->aNegativeLimit = obj.aNegativeLimit;
		pThis->aPositiveLimit = obj.aPositiveLimit;
		pThis->bNegativeLimit = obj.bNegativeLimit;
		pThis->bPositiveLimit = obj.bPositiveLimit;
		pThis->cNegativeLimit = obj.cNegativeLimit;
		pThis->cPositiveLimit = obj.cPositiveLimit;
	}
}PcsSafeSpaceLimit, *PPcsSafeSpaceLimit;

typedef struct ST_AcsSafeSpaceLimt
{
	double j1PositiveLimit;
	double j1NegativeLimit;
	double j2PositiveLimit;
	double j2NegativeLimit;
	double j3PositiveLimit;
	double j3NegativeLimit;
	double j4PositiveLimit;
	double j4NegativeLimit;
	double j5PositiveLimit;
	double j5NegativeLimit;
	double j6PositiveLimit;
	double j6NegativeLimit;
	ST_AcsSafeSpaceLimt() :j1PositiveLimit(0.0), j1NegativeLimit(0.0),
		j2PositiveLimit(0.0), j2NegativeLimit(0.0),
		j3PositiveLimit(0.0), j3NegativeLimit(0.0),
		j4PositiveLimit(0.0), j4NegativeLimit(0.0),
		j5PositiveLimit(0.0), j5NegativeLimit(0.0),
		j6PositiveLimit(0.0), j6NegativeLimit(0.0)
	{
		;
	}
	ST_AcsSafeSpaceLimt(const ST_AcsSafeSpaceLimt& obj)
	{
		*this = obj;
	}

	ST_AcsSafeSpaceLimt& operator=(const ST_AcsSafeSpaceLimt& obj)
	{
		copy(this, obj);
		return *this;
	}
private:
	void copy(ST_AcsSafeSpaceLimt* pThis, const ST_AcsSafeSpaceLimt& obj)
	{
		pThis->j1NegativeLimit = obj.j1NegativeLimit;
		pThis->j1PositiveLimit = obj.j1PositiveLimit;
		pThis->j2NegativeLimit = obj.j2NegativeLimit;
		pThis->j2PositiveLimit = obj.j2PositiveLimit;
		pThis->j3NegativeLimit = obj.j3NegativeLimit;
		pThis->j3PositiveLimit = obj.j3PositiveLimit;
		pThis->j4NegativeLimit = obj.j4NegativeLimit;
		pThis->j4PositiveLimit = obj.j4PositiveLimit;
		pThis->j5NegativeLimit = obj.j5NegativeLimit;
		pThis->j5PositiveLimit = obj.j5PositiveLimit;
		pThis->j6NegativeLimit = obj.j6NegativeLimit;
		pThis->j6PositiveLimit = obj.j6PositiveLimit;
	}
}AcsSafeSpaceLimit, *PAcsSafeSpaceLimit;

// 角度坐标
typedef struct ST_PointCoord
{
	double Axis[Max_ActualAxisCnt];
	ST_PointCoord()
	{
		memset(Axis, 0, sizeof(Axis));
	}

	ST_PointCoord(double Axis1Val, double Axis2Val, double Axis3Val,
		double Axis4Val, double Axis5Val, double Axis6Val)
	{
		Axis[0] = Axis1Val;
		Axis[1] = Axis2Val;
		Axis[2] = Axis3Val;
		Axis[3] = Axis4Val;
		Axis[4] = Axis5Val;
		Axis[5] = Axis6Val;
	}

	ST_PointCoord(const ST_PointCoord& obj)
	{
		*this = obj;
	}

	ST_PointCoord& operator=(const ST_PointCoord& obj)
	{
		copy(this, obj);
		return *this;
	}

	bool operator==(const ST_PointCoord& obj)
	{
		for (int i = 0; i < Max_ActualAxisCnt; ++i)
		{
			if (obj.Axis[i] != obj.Axis[i])
			{
				return false;
			}
		}
		return true;
	}

	int Assign(const double* pdVal, int nSize)
	{
		if ((NULL == pdVal) || (nSize < Max_ActualAxisCnt))
			return -1;
		for (int i = 0; i < Max_ActualAxisCnt; ++i)
		{
			this->Axis[i] = pdVal[i];
		}
		return 0;
	}

	int ToDouble(double* pdVal, int nSize)
	{
		if ((NULL == pdVal) || (nSize < Max_ActualAxisCnt))
			return -1;
		for (int i = 0; i < Max_ActualAxisCnt; ++i)
		{
			pdVal[i] = this->Axis[i];
		}
		return 0;
	}
private:
	void copy(ST_PointCoord* pThis, const ST_PointCoord& obj)
	{
		for (int i = 0; i < Max_ActualAxisCnt; ++i)
		{
			this->Axis[i] = obj.Axis[i];
		}
	}
}PointCoord, *PPointCoord;

//DragMove信息结构体
typedef struct ST_DragMovePoint
{
	int nPointCnt;
	int nPcsPointCnt;
	vector<PointCoord> vecAcsCoord;
	vector<PointCoord> vecPcsCoord;

	vector<int> vecIOCnt;
	vector<int> vecIOIndex;
	vector<int> vecIOState;
	vector<int> vecTimeElapse;
	vector<int> vecIsStop;
	ST_DragMovePoint() :nPointCnt(0), nPcsPointCnt(0)
	{
	}

	ST_DragMovePoint(const ST_DragMovePoint& obj)
	{
		*this = obj;
	}

	ST_DragMovePoint& operator=(const ST_DragMovePoint& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(ST_DragMovePoint* pThis, const ST_DragMovePoint& obj)
	{

		pThis->nPointCnt = obj.nPointCnt;
		pThis->nPcsPointCnt = obj.nPcsPointCnt;
		pThis->vecAcsCoord = obj.vecAcsCoord;
		pThis->vecPcsCoord = obj.vecPcsCoord;
		pThis->vecIOCnt = obj.vecIOCnt;
		pThis->vecIOIndex = obj.vecIOIndex;
		pThis->vecIOState = obj.vecIOState;
		pThis->vecIsStop = obj.vecIsStop;
		pThis->vecTimeElapse = obj.vecTimeElapse;
	}
}DragMovePoint;

enum EN_SeekStatus
{
	en_seekStatusInit = 0,
	en_seekStatusIng ,
	en_seekStatusReset,
	en_seekStatusEnd ,
};

/*码垛重入时，保存的上下文信息*/
typedef struct ST_SeekData
{
	int inCount;  							/* 计数器 0的时候，初始化码垛参数 */
	int seekType;							/* 0,码垛，1 卸跺 */
	int seekIfResult;						/* 0, 开始探寻*/
	int seekUseIfGoOn;						/* 示教器勾选继续码垛条件 */
	int seekIfGoOnResult;					/* 是否正方向继续码垛 */
	double thisStartPoint[6];				/* 当前序列的start位置,对应序列中的锚点位置 */
	vector<vector<double>> thisWayPoints;	/* 当前路点轨迹列表 */
	vector<double> startPoint;				/* 存储起始点位 */	
	int nStartPointMoveing;					/* 1: 正在运动到start点位, 0 运动停止*/
	int nSeekMoveing;						/* 1: 正在执行探寻运动 ，0，运动停止*/
	int nWayPointCnt;						/* 序列路点个数 */



	enum EN_SeekStatus stackStatus;			/* 0,初始状态，1，码垛过程中，2，码垛结束 */
											// 码垛: a) 码到最高点，码垛结束
											//       b) 码垛结束条件为真，码垛结束
													 // 
											
											// 卸跺：a) 卸到最高点，卸跺结束
											//       b) 卸跺结束条件为真，卸跺结束
											//       c) 卸跺探寻到最高点，卸跺结束
	bool nSeekToEnd;                        // 卸跺：a) 卸到最高点，卸跺结束
	/*探寻条件表达式*/
	int m_enIfType;
	bool m_bValue;
	int m_nComPort;				//0~15
	int m_nEtcPort;				//0~15
	int m_nRobotID;
	int m_nConditionType;
	string m_sCondition;
	vector<string> m_vecCondition;
	int m_nCondCount;
	vector<string> m_vecCondition1[5];
	vector<string> m_vecCondSymbol;


	/*Dir条件表达式*/
	int  m_DiruseStopIf;
	int  m_DirenIfType;
	bool m_DirbValue;
	int m_DirnComPort;				//0~15
	int m_DirnEtcPort;				//0~15
	int m_DirnConditionType;
	string m_DirsCondition;
	vector<string> m_DirvecCondition;
	int m_DirnCondCount;
	vector<string> m_DirvecCondition1[5];
	vector<string> m_DirvecCondSymbol;

	//记录是否使用Call
	int m_UseEndFun;
	int m_UseStartFun;
	bool bstackRestart;
	int m_FristSeek;
	int m_FristSeekStatus;
	
}SeekData;



//DragMove信息结构体
typedef struct ST_BlendingPoint
{
	int nMoveType;
	double dRadius;
	int nMoveCFixposure;
	PointCoord p1;
	PointCoord p2;

	ST_BlendingPoint() :nMoveType(0), dRadius(0), nMoveCFixposure(0)
	{
	}

	ST_BlendingPoint(const ST_BlendingPoint& obj)
	{
		*this = obj;
	}

	ST_BlendingPoint& operator=(const ST_BlendingPoint& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(ST_BlendingPoint* pThis, const ST_BlendingPoint& obj)
	{

		pThis->nMoveType = obj.nMoveType;
		pThis->dRadius = obj.dRadius;
		pThis->nMoveCFixposure = obj.nMoveCFixposure;
		pThis->p1 = obj.p1;
		pThis->p2 = obj.p2;
	}
}BlendingPoint;

//Blending模拟AO信息结构体
typedef struct ST_BlendingAO
{
	int nSlot[SERIALANALOGDO_CNT];
	double dValue[SERIALANALOGDO_CNT];
	int nMode[SERIALANALOGDO_CNT];
	PointCoord coord;
	ST_BlendingAO()
	{
		for (int i = 0; i < SERIALANALOGDO_CNT;++i)
		{
			nSlot[i] = 0;
			dValue[i] = 0;
			nMode[i] = 0;
		}
	}

	ST_BlendingAO(const ST_BlendingAO& obj)
	{
		*this = obj;
	}

	ST_BlendingAO& operator=(const ST_BlendingAO& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(ST_BlendingAO* pThis, const ST_BlendingAO& obj)
	{
		for (int i = 0; i < SERIALANALOGDO_CNT; ++i)
		{
			this->nSlot[i] = obj.nSlot[i];
			this->dValue[i] = obj.dValue[i];
			this->nMode[i] = obj.nMode[i];
		}
		this->coord = obj.coord;
	}
}BlendingAO;


//空间坐标
typedef struct ST_SpatialCoord
{
	double x;
	double y;
	double z;
	double a;
	double b;
	double c;
	ST_SpatialCoord() :x(0.0), y(0.0), z(0.0), a(0.0), b(0.0), c(0.0)
	{
		;
	}

	ST_SpatialCoord(double xVal, double yVal, double zVal,
		double aVal, double bVal, double cVal)
	{
		x = xVal;
		y = yVal;
		z = zVal;
		a = aVal;
		b = bVal;
		c = cVal;
	}

	ST_SpatialCoord(const PointCoord& coord)
	{
		x = coord.Axis[0];
		y = coord.Axis[1];
		z = coord.Axis[2];
		a = coord.Axis[3];
		b = coord.Axis[4];
		c = coord.Axis[5];
	}

	ST_SpatialCoord(const ST_SpatialCoord& obj)
	{
		*this = obj;
	}

	ST_SpatialCoord& operator=(const ST_SpatialCoord& obj)
	{
		copy(this, obj);
		return *this;
	}

	// 相等运算符重载
	bool operator==(const ST_SpatialCoord& obj)
	{
		double dxTmp = 0.0, dyTmp = 0.0, dzTmp = 0.0,
			daTmp = 0.0, dbTmp = 0.0, dcTmp = 0.0;
		dxTmp = this->x - obj.x;
		dyTmp = this->y - obj.y;
		dzTmp = this->z - obj.z;
		daTmp = this->a - obj.a;
		dbTmp = this->b - obj.b;
		dcTmp = this->c - obj.c;

		if ((fabs(dxTmp) <= DOUBLE_EPSINON) &&
			(fabs(dyTmp) <= DOUBLE_EPSINON) &&
			(fabs(dzTmp) <= DOUBLE_EPSINON) &&
			(fabs(daTmp) <= DOUBLE_EPSINON) &&
			(fabs(dbTmp) <= DOUBLE_EPSINON) &&
			(fabs(dcTmp) <= DOUBLE_EPSINON))
		{
			return true;
		}

		return false;
	}

	int Assign(const double* pdVal, int nSize)
	{
		if ((NULL == pdVal) || (nSize < Max_ActualAxisCnt))
			return -1;
		this->x = pdVal[0];
		this->y = pdVal[1];
		this->z = pdVal[2];
		this->a = pdVal[3];
		this->b = pdVal[4];
		this->c = pdVal[5];
		return 0;
	}

	int ToDouble(double* pdVal, int nSize)
	{
		if ((NULL == pdVal) || (nSize < Max_ActualAxisCnt))
			return -1;
		pdVal[enPAPIndex_x] = x;
		pdVal[enPAPIndex_y] = y;
		pdVal[enPAPIndex_z] = z;
		pdVal[enPAPIndex_a] = a;
		pdVal[enPAPIndex_b] = b;
		pdVal[enPAPIndex_c] = c;
		return 0;
	}

	string Format() const
	{
		string sFormat;
		sFormat.append(std::to_string(x));
		sFormat.append(",");
		sFormat.append(std::to_string(y));
		sFormat.append(",");
		sFormat.append(std::to_string(z));
		sFormat.append(",");
		sFormat.append(std::to_string(a));
		sFormat.append(",");
		sFormat.append(std::to_string(b));
		sFormat.append(",");
		sFormat.append(std::to_string(c));
		sFormat.append(",");

		return sFormat;
	}
private:
	void copy(ST_SpatialCoord* pThis, const ST_SpatialCoord& obj)
	{
		pThis->x = obj.x;
		pThis->y = obj.y;
		pThis->z = obj.z;
		pThis->a = obj.a;
		pThis->b = obj.b;
		pThis->c = obj.c;
	}
}SpatialCoord, *PSpatialCoord;

// 轴电流
typedef struct ST_Current
{
	double Axis[Max_ActualAxisCnt];
	ST_Current()
	{
		memset(Axis, 0, sizeof(Axis));
	}

	ST_Current(double Axis1Val, double Axis2Val, double Axis3Val,
		double Axis4Val, double Axis5Val, double Axis6Val)
	{
		Axis[0] = Axis1Val;
		Axis[1] = Axis2Val;
		Axis[2] = Axis3Val;
		Axis[3] = Axis4Val;
		Axis[4] = Axis5Val;
		Axis[5] = Axis6Val;
	}

	ST_Current(const ST_Current& obj)
	{
		*this = obj;
	}

	ST_Current& operator=(const ST_Current& obj)
	{
		copy(this, obj);
		return *this;
	}

	int Assign(const double* pdVal, int nSize)
	{
		if ((NULL == pdVal) || (nSize < Max_ActualAxisCnt))
			return -1;
		for (int i = 0; i < Max_ActualAxisCnt; ++i)
		{
			this->Axis[i] = pdVal[i];
		}
		return 0;
	}

	int ToDouble(double* pdVal, int nSize)
	{
		if ((NULL == pdVal) || (nSize < Max_ActualAxisCnt))
			return -1;
		for (int i = 0; i < Max_ActualAxisCnt; ++i)
		{
			pdVal[i] = this->Axis[i];
		}
		return 0;
	}
private:
	void copy(ST_Current* pThis, const ST_Current& obj)
	{
		for (int i = 0; i < Max_ActualAxisCnt; ++i)
		{
			this->Axis[i] = obj.Axis[i];
		}
	}
}AxisCurrent, *PAxisCurrent;

typedef struct _ST_AxisData
{
	double Axis[Max_ActualAxisCnt];
	_ST_AxisData()
	{
		memset(Axis, 0, sizeof(Axis));
	}

	_ST_AxisData(double Axis1Val, double Axis2Val, double Axis3Val,
		double Axis4Val, double Axis5Val, double Axis6Val)
	{
		Axis[0] = Axis1Val; Axis[1] = Axis2Val; Axis[2] = Axis3Val;
		Axis[3] = Axis4Val; Axis[4] = Axis5Val; Axis[5] = Axis6Val;
	}

	_ST_AxisData(const _ST_AxisData& obj)
	{
		*this = obj;
	}

	_ST_AxisData& operator=(const _ST_AxisData& obj)
	{
		copy(this, obj);
		return *this;
	}

	int Assign(const double* pdVal, int nSize)
	{
		if ((NULL == pdVal) || (nSize < Max_ActualAxisCnt))
			return -1;
		for (int i = 0; i < Max_ActualAxisCnt; ++i)
		{
			this->Axis[i] = pdVal[i];
		}
		return 0;
	}

	int ToDouble(double* pdVal, int nSize)
	{
		if ((NULL == pdVal) || (nSize < Max_ActualAxisCnt))
			return -1;
		for (int i = 0; i < Max_ActualAxisCnt; ++i)
		{
			pdVal[i] = this->Axis[i];
		}
		return 0;
	}
private:
	void copy(_ST_AxisData* pThis, const _ST_AxisData& obj)
	{
		for (int i = 0; i < Max_ActualAxisCnt; ++i)
		{
			this->Axis[i] = obj.Axis[i];
		}
	}
}ST_AxisData;


/*********************************************************************
**机器人位置信息结构体
**AcsActualPos=角度坐标实际位置
**PcsActualPos=空间坐标实际位置
*********************************************************************/
typedef struct ST_RobotPosition
{
	PointCoord AcsActualPos;
	PointCoord PcsActualPos;
	PointCoord KineActualPos;

	AxisCurrent RobotAxisCurrent;
	ST_RobotPosition()
	{
	}

	ST_RobotPosition(const ST_RobotPosition& obj)
	{
		*this = obj;
	}

	ST_RobotPosition& operator=(const ST_RobotPosition& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(ST_RobotPosition* pThis, const ST_RobotPosition& obj)
	{
		pThis->AcsActualPos = obj.AcsActualPos;
		pThis->PcsActualPos = obj.PcsActualPos;
		pThis->KineActualPos = obj.KineActualPos;
		pThis->RobotAxisCurrent = obj.RobotAxisCurrent;
	}
}RobotPosition;

typedef struct ST_MultiEndatPos
{
	int multiEndatPos[Max_ActualAxisCnt];
	ST_MultiEndatPos()
	{
		memset(multiEndatPos, 0, sizeof(multiEndatPos));
	}

	ST_MultiEndatPos(const ST_MultiEndatPos& obj)
	{
		*this = obj;
	}

	ST_MultiEndatPos& operator=(const ST_MultiEndatPos& obj)
	{
		copy(this, obj);
		return *this;
	}
private:
	void copy(ST_MultiEndatPos* pThis, const ST_MultiEndatPos& obj)
	{
		for (int i = 0; i < Max_ActualAxisCnt; ++i)
		{
			pThis->multiEndatPos[i] = obj.multiEndatPos[i];
		}
	}
}MultiEndatPos, *PMultiEndatPos;

// 阈值参数(Threshold)
typedef struct ST_Threshold
{
	double J1;
	double J2;
	double J3;
	double J4;
	double J5;
	double J6;
	ST_Threshold() :
		J1(0),
		J2(0),
		J3(0),
		J4(0),
		J5(0),
		J6(0)
	{
		;
	}

	ST_Threshold(const ST_Threshold& obj)
	{
		*this = obj;
	}

	ST_Threshold& operator=(const ST_Threshold& obj)
	{
		copy(this, obj);
		return *this;
	}
private:
	void copy(ST_Threshold* pThis, const ST_Threshold& obj)
	{
		pThis->J1 = obj.J1;
		pThis->J2 = obj.J2;
		pThis->J3 = obj.J3;
		pThis->J4 = obj.J4;
		pThis->J5 = obj.J5;
		pThis->J6 = obj.J6;
	}
}Threshold, *PThreshold;

//安全停车模式(CollideMode)
typedef struct ST_CollideMode
{
	int nLevel;
	int nWaitTime;
	ST_CollideMode() :nLevel(0), nWaitTime(0){}
	ST_CollideMode(const ST_CollideMode& obj)
	{
		*this = obj;
	}
	ST_CollideMode& operator=(const ST_CollideMode& obj)
	{
		copy(this, obj);
		return *this;
	}
private:
	void copy(ST_CollideMode* pThis, const ST_CollideMode& obj)
	{
		pThis->nLevel = obj.nLevel;
		pThis->nWaitTime = obj.nWaitTime;
	}
}CollideMode;
// 负载参数(Payload)
typedef struct ST_Payload
{
	double mass;
	double masscenterX;
	double masscenterY;
	double masscenterZ;
	ST_Payload() :
		mass(0),
		masscenterX(0),
		masscenterY(0),
		masscenterZ(0)
	{
		;
	}

	ST_Payload(const ST_Payload& obj)
	{
		*this = obj;
	}

	ST_Payload& operator=(const ST_Payload& obj)
	{
		copy(this, obj);
		return *this;
	}
private:
	void copy(ST_Payload* pThis, const ST_Payload& obj)
	{
		pThis->mass = obj.mass;
		pThis->masscenterX = obj.masscenterX;
		pThis->masscenterY = obj.masscenterY;
		pThis->masscenterZ = obj.masscenterZ;
	}
}Payload, *PPayload;

// 零力示教参数(Teach)
typedef struct ST_Teach
{
	double J1;
	double J2;
	double J3;
	double J4;
	double J5;
	double J6;
	ST_Teach() :
		J1(0),
		J2(0),
		J3(0),
		J4(0),
		J5(0),
		J6(0)
	{
		;
	}

	ST_Teach(const ST_Teach& obj)
	{
		*this = obj;
	}

	ST_Teach& operator=(const ST_Teach& obj)
	{
		copy(this, obj);
		return *this;
	}
private:
	void copy(ST_Teach* pThis, const ST_Teach& obj)
	{
		pThis->J1 = obj.J1;
		pThis->J2 = obj.J2;
		pThis->J3 = obj.J3;
		pThis->J4 = obj.J4;
		pThis->J5 = obj.J5;
		pThis->J6 = obj.J6;
	}
}Teach, *PTeach;

// PNP
typedef struct ST_PNPInfo
{

}PNPInfo, *PPNPInfo;

typedef struct _ST_ScriptFuncInfo
{
	int nSocket;
	int nEvent;
	int nCallRlt;						// 调用脚本函数结果（0=调用成功；其他=错误码）
	string funcName;					// 函数名称
	string funcResult;					// 函数返回结果	
	string paramList;								// 参数列表
	PFuncRunResultCallBack pRunRltCallBackFunc;		// 函数运行结果回调函数
	_ST_ScriptFuncInfo() :nSocket(0), pRunRltCallBackFunc(0), nEvent(0),
		nCallRlt(0)
	{
	}

	_ST_ScriptFuncInfo(const _ST_ScriptFuncInfo& obj)
	{
		*this = obj;
	}

	_ST_ScriptFuncInfo& operator=(const _ST_ScriptFuncInfo& obj)
	{
		copy(this, obj);
		return *this;
	}

	void Clear()
	{
		funcName.clear();
		funcResult.clear();
		nSocket = 0;
		nEvent = 0;
		nCallRlt = 0;
		paramList.clear();
		pRunRltCallBackFunc = 0;
	}

private:
	void copy(_ST_ScriptFuncInfo* pDest, const _ST_ScriptFuncInfo& obj)
	{
		pDest->nSocket = obj.nSocket;
		pDest->nEvent = obj.nEvent;
		pDest->nCallRlt = obj.nCallRlt;
		pDest->funcName = obj.funcName;
		pDest->funcResult = obj.funcResult;
		pDest->paramList = obj.paramList;
		pDest->pRunRltCallBackFunc = obj.pRunRltCallBackFunc;
	}
}ScriptFuncInfo;

//////////////////////////////////////////////////////////////////////////
//刀具坐标信息
//index=刀具坐标所在的索引
//name=刀具名称
//property=属性
//defaule=是否设为默认刀具
//coord=刀具坐标值，空间坐标
//////////////////////////////////////////////////////////////////////////
typedef struct _ST_CoordInfo
{
	int index;
	char szName[Coord_Name];
	bool default;
	PointCoord coord;
	_ST_CoordInfo() :index(0), default(false)
	{
		memset(szName, 0, sizeof(szName));
	}

	_ST_CoordInfo(const _ST_CoordInfo& obj)
	{
		*this = obj;
	}

	_ST_CoordInfo& operator=(const _ST_CoordInfo& obj)
	{
		Copy(this, obj);
		return *this;
	}
	//bool _ST_CoordInfo
	bool operator==(const _ST_CoordInfo &obj)
	{
		bool isIndexSame = (this->index == obj.index);
		bool isNameSame = (0 == strcmp(this->szName, obj.szName));
		bool isCoordSame = (this->coord == obj.coord);

		return isIndexSame && isNameSame && isCoordSame;
	}

private:
	void Copy(_ST_CoordInfo* pThis, const _ST_CoordInfo& obj)
	{
		pThis->index = obj.index;
		pThis->default = obj.default;
		pThis->coord = obj.coord;
		memcpy(szName, obj.szName, sizeof(szName));
		//		pThis->name = obj.name;
	}
}ST_CoordInfo;

//////////////////////////////////////////////////////////////////////////
//RobotVelocityInfo
//////////////////////////////////////////////////////////////////////////
typedef struct ST_RobotMotionLimits
{
	JointMotionLimits moveJointMotionLimits;
	PcsMotionLimits movePcsMotionLimits;
	JointMotionLimits stopJointMotionLimits;
	PcsMotionLimits stopPcsMotionLimits;
	ST_RobotMotionLimits()
	{

	}

	ST_RobotMotionLimits(const ST_RobotMotionLimits& obj)
	{
		*this = obj;
	}

	ST_RobotMotionLimits& operator=(const ST_RobotMotionLimits& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(ST_RobotMotionLimits* pThis, const ST_RobotMotionLimits& obj)
	{
		pThis->moveJointMotionLimits = obj.moveJointMotionLimits;
		pThis->movePcsMotionLimits = obj.movePcsMotionLimits;
		pThis->stopJointMotionLimits = obj.stopJointMotionLimits;
		pThis->stopPcsMotionLimits = obj.stopPcsMotionLimits;
	}
}RobotMotionLimits;

//////////////////////////////////////////////////////////////////////////
//碰撞检测参数结构体
//payload_mass=负载质量，单位：Kg
//payload_centerOfMass=负载质心(x,y,z)，单位：m
//thresholds=阀值，单位：Nm
//assistiveMode_thresholds=零力示教碰撞阈值，单位：Nm
//////////////////////////////////////////////////////////////////////////
typedef struct ST_CollideStopParam
{
	double payload_mass;
	double payload_centerOfMass[3];
	double thresholds[Max_ActualAxisCnt];
	double assistiveMode_thresholds[Max_ActualAxisCnt];
	double velocity_thresholds[Max_ActualAxisCnt];
	double frictionCompensationFactor[Max_ActualAxisCnt];
	double motorScales_Slave[ElfinSlaveCnt];
	double frictionWithTemperature[Max_ActualAxisCnt];
	ST_CollideStopParam() :payload_mass(0.0)
	{
		payload_centerOfMass[0] = 0.0;
		payload_centerOfMass[1] = 0.0;
		payload_centerOfMass[2] = 0.0;

		for (int n = 0; n < Max_ActualAxisCnt; ++n)
		{
			thresholds[n] = 0.0;
			assistiveMode_thresholds[n] = 0.0;
			velocity_thresholds[n] = 0.0;
			frictionCompensationFactor[n] = 0.0;
			frictionWithTemperature[n] = 0.0;
		}
		for (int n = 0; n < ElfinSlaveCnt; ++n)
		{
			motorScales_Slave[n] = 0.0;
		}
	}

	ST_CollideStopParam(const ST_CollideStopParam& obj)
	{
		*this = obj;
	}

	ST_CollideStopParam& operator=(const ST_CollideStopParam& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(ST_CollideStopParam* pThis, const ST_CollideStopParam& obj)
	{
		pThis->payload_mass = obj.payload_mass;
		pThis->payload_centerOfMass[0] = obj.payload_centerOfMass[0];
		pThis->payload_centerOfMass[1] = obj.payload_centerOfMass[1];
		pThis->payload_centerOfMass[2] = obj.payload_centerOfMass[2];

		for (int n = 0; n < Max_ActualAxisCnt; ++n)
		{
			pThis->thresholds[n] = obj.thresholds[n];
			pThis->assistiveMode_thresholds[n] = obj.assistiveMode_thresholds[n];
			pThis->velocity_thresholds[n] = obj.velocity_thresholds[n];
			pThis->frictionCompensationFactor[n] = obj.frictionCompensationFactor[n];
			pThis->frictionWithTemperature[n] = obj.frictionWithTemperature[n];	
		}
		for (int n = 0; n < ElfinSlaveCnt; ++n)
		{
			pThis->motorScales_Slave[n] = obj.motorScales_Slave[n];
		}
	}
}CollideStopParam;

//////////////////////////////////////////////////////////////////////////
//碰撞检测参数结构体
//payload_mass=负载质量，单位：Kg
//payload_centerOfMass=负载质心(x,y,z)，单位：m
//thresholds=阀值，单位：Nm
//assistiveMode_thresholds=零力示教碰撞阈值，单位：Nm
//////////////////////////////////////////////////////////////////////////
typedef struct ST_PayloadRecognitionParam
{
	double JointPositions[Max_ActualAxisCnt];
	double JointVelocities[Max_ActualAxisCnt];
	double JointAccelerations[Max_ActualAxisCnt];
	double SensedJointTorques[Max_ActualAxisCnt];
	double EstimatedJointTorques[Max_ActualAxisCnt];
	double MotorCurrent[Max_ActualAxisCnt];
	ST_PayloadRecognitionParam()
	{
		for (int n = 0; n < Max_ActualAxisCnt; ++n)
		{
			//thresholds[n] = 0.0;
			JointPositions[n] = 0.0;
			JointVelocities[n] = 0.0;
			JointAccelerations[n] = 0.0;
			SensedJointTorques[n] = 0.0;
			EstimatedJointTorques[n] = 0.0;
			MotorCurrent[n] = 0.0;
		}
	}

	ST_PayloadRecognitionParam(const ST_PayloadRecognitionParam& obj)
	{
		*this = obj;
	}

	ST_PayloadRecognitionParam& operator=(const ST_PayloadRecognitionParam& obj)
	{
		Copy(this, obj);
		return *this;
	}
private:
	void Copy(ST_PayloadRecognitionParam* pThis, const ST_PayloadRecognitionParam& obj)
	{
		for (int n = 0; n < Max_ActualAxisCnt; ++n)
		{
			//pThis->thresholds[n] = obj.thresholds[n];
			pThis->JointPositions[n] = obj.JointPositions[n];
			pThis->JointVelocities[n] = obj.JointVelocities[n];
			pThis->JointAccelerations[n] = obj.JointAccelerations[n];
			pThis->SensedJointTorques[n] = obj.SensedJointTorques[n];
			pThis->EstimatedJointTorques[n] = obj.EstimatedJointTorques[n];
			pThis->MotorCurrent[n] = obj.MotorCurrent[n];
		}
	}
}PayloadRecognitionParam;



typedef struct _ST_RobotRealTimeStatu
{
	MultiEndatPos rbtPosEndat;
	PointCoord rbtPosACS;
	PointCoord rbtPosPCS;
	PointCoord rbtPosTCP;//该位置是后来加的,为了记录装换TCP模式时的PCS位置
	// 实际速度 DOUBLE SIZE=1
	double dOverride;
	//串口IO
	int nSerialInDIO[BoxCommIO_CNT];
	int nSerialOutDIO[BoxCommIO_CNT];
	int nSerialAnalog[SERIALANALOG_CNT];
	//末端IO
	int nInDIO[Max_InIOCnt];
	int nOutDIO[Max_OutIOCnt];
	int nAnalogInState[AnalogINCnt];
	// 获取机器人状态 INT SIZE=8
	int nRobotStatus[enRobotStatusCnt];
	// 获取超出安全空间相关信息 INT SIZE=3
	int nOSS_AxisID;
	int nOSS_Direction;
	int nOSS_CoordType;
	// 获取错误信息 INT SIZE=2
	int nErrCode;
	int nErrAxisID;
	//控制器仿真与打印信息状态
	int nSimulation;
	int nPrintLog;
	int nEmergencyStop;
	_ST_RobotRealTimeStatu() :dOverride(0.0), nOSS_AxisID(0), nOSS_Direction(0), nOSS_CoordType(0),
		nErrCode(0), nErrAxisID(0), nSimulation(0), nPrintLog(0), nEmergencyStop(0)
	{
		memset(nSerialInDIO, 0, BoxCommIO_CNT*sizeof(int));
		memset(nSerialOutDIO, 0, BoxCommIO_CNT*sizeof(int));
		memset(nSerialAnalog, 0, SERIALANALOG_CNT*sizeof(int));
		memset(nInDIO, 0, Max_InIOCnt*sizeof(int));
		memset(nOutDIO, 0, Max_OutIOCnt*sizeof(int));
		memset(nAnalogInState, 0, AnalogINCnt);
		memset(nRobotStatus, 0, enRobotStatusCnt);
	}
}ST_RobotRealTimeStatu;

//RbTL 的ASCII码
#define MAGIC_NUMBER 0x4C544252

enum ScriptThread
{
	enScriptThread_Main = 0,
	enScriptThread_Timer0,
	enScriptThread_Timer1,
	enScriptThread_Timer2,
	enScriptThread_Timer3,
	enScriptThread_Timer4,
	enScriptThread_CNT
};

//DCS状态机
enum enHMRCStatus
{
	enHMRCStatus_UnInitialize = 0,
	enHMRCStatus_Initialize,
	enHMRCStatus_SystemBoardNotConnect,
	enHMRCStatus_SystemBoardNotElectrify,
	enHMRCStatus_RTOSNotStart,
	enHMRCStatus_ControllerNotStart,
	enHMRCStatus_Config,
	enHMRCStatus_Emergency,
	enHMRCStatus_SaftyGuard,
	enHMRCStatus_Error,
	enHMRCStatus_Normal,
	enHMRCStatus_RunScript,
	enHMRCStatus_MasterNotStart,
	enHMRCStatus_TeachMode,
	enHMRCStatus_MasterStarting,
	enHMRCStatus_EmergencyHandling,
	enHMRCStatus_OutofSafeSpace,
	enHMRCStatus_SwitchONBTNError,
	enHMRCStatus_Count
};

enum EN_LogsLevel
{
	enLogsLevel_Unknown = 0,
	enLogsLevel_Normal,
	enLogsLevel_Warning,
	enLogsLevel_Serious,
};
typedef void(OnLogInfoCallback)(int nErr, string strLog);
typedef void(OnScopeCallback)(int nRotID, int nCtrlID,bool& ret, string &strFileName);//示波器回调函数申明
typedef void(OnSetHandInfoNotify)();

#define CMDNAME_KEY "CmdName"
#define CMDID_KEY "CmdID"
#define CMDREQEVENT_KEY "CmdReqEvent"
#define CMDTYPE_KEY "CmdType"

// Actin 交互声明
const int HANSMC_ACTIN = 0xff;
const int Actin_PacketHeadLen = 2;
typedef enum E_ActinCMD
{
	ActinCMD_Read = 1,
}ActinCMD;


//-------------------Tcp Port---------------//
#define Port_Command    1991
#define Port_STO        10003
#define Port_IF			10003
#define Port_RealTime   10004

enum EN_PortType
{
	enPortType_Command = 0,
	enPortType_STO
};

//定义:存储数item和节点函数名字
//typedef QMap<QString, QTreeWidgetItem*>MAP_FuncTreeItem;
typedef map<string, void*>MAP_FuncTreeItem;

//////////////////////////////////////////////////////////////////////////
//CMD_ReqLogin：上层软件(HansMC)发送给DCS的请求登陆消息
//CMD_AckLogin：DCS返回给HansMC的对应请求登陆消息的响应消息
//////////////////////////////////////////////////////////////////////////
#define CMD_ReqLogin "CmdReqLogin"
#define CMD_AckLogin "CmdAckLogin"

//////////////////////////////////////////////////////////////////////////
//CmdReqAvalilableLogDataLen：
//CmdAckAvalilableLogDataLen：
//////////////////////////////////////////////////////////////////////////
#define  CMD_ReqAvalilableLogDataLen  "CmdReqAvalilableLogDataLen"
#define  CMD_AckAvalilableLogDataLen  "CmdAckAvalilableLogDataLen"
//////////////////////////////////////////////////////////////////////////
//CMD_ReqQureyLogData：上层软件(HansMC)发送给DCS的请求查询日志数据内容
//CMD_AckQureyLogData：DCS返回给HansMC的对应请求查询日志数据内容
//////////////////////////////////////////////////////////////////////////
#define  CMD_ReqAvalilableLogData  "CmdReqAvalilableLogData"
#define  CMD_AckAvalilableLogData  "CmdAckAvalilableLogData"


#pragma pack(push,1)
//RbTL协议头部
typedef struct _ST_hmMsgHeader
{
	int dwMagicNum;	//0 magic number, always 0x5242544C
	int dwProcoVec;
	int dwEncrypt;	//EncryptType
	int dwPackLen;//pack len, network byte
	int dwDataLen;//data len, network byte
	char Data[1];
}ST_hmMsgHeader;

typedef struct _ST_RbtRealTimeStatu
{
	MultiEndatPos rbtPosEndat;
	PointCoord rbtPosACS;
	PointCoord rbtPosPCS;
	PointCoord rbtPosPCSForBase;
	PointCoord rbtPosTOOL;
	AxisCurrent motorCurrent;
	double CommandVelocity[Max_ActualAxisCnt];
	double motorTemperature[Max_ActualAxisCnt];
	double motorVoltage[Max_ActualAxisCnt];
//	double motorA[Max_ActualAxisCnt];
	double Acceleration[Max_ActualAxisCnt];
	double Dcceleration[Max_ActualAxisCnt];
	// 	double Joints[Max_ActualAxisCnt];//关节角度
	// 	double Axis[Max_ActualAxisCnt];//空间坐标
	// 实际速度 DOUBLE SIZE=1
	double dOverride;
	//末端IO
	int nInDIO[Max_InIOCnt];
	int nOutDIO[Max_OutIOCnt];
	int nModbusInDIO[Max_InIOCnt];
	double nAnalogInState[AnalogINCnt];
	double nAnalogOutState[AnalogINCnt];
	// 获取机器人状态 INT SIZE=8
	int nRobotStatus[enRobotStatusCnt];
	// 获取超出安全空间相关信息 INT SIZE=3
	int nOSS_AxisID;
	int nOSS_Direction;
	int nOSS_CoordType;
	// 获取错误信息 INT SIZE=2
	int nErrCode;
	int nErrAxisID;
	int nCurToolCoord;
	int nControlMode;
	bool bRbtIDValid;	//代表机器ID是否启用
	int nRbtType;
	int nToolCoordinateMotionFlag;	//
	double dErrData[Max_ActualAxisCnt];
	double dConveyorVal;
	double dConveyorPos;
	_ST_RbtRealTimeStatu() :dOverride(0.0), nOSS_AxisID(0), nOSS_Direction(0), nOSS_CoordType(0),
		nErrCode(0), nErrAxisID(0), nCurToolCoord(0), nControlMode(0), bRbtIDValid(false), nRbtType(0),
		nToolCoordinateMotionFlag(0), dConveyorPos(0)
	{

		memset(nInDIO, 0, Max_InIOCnt*sizeof(int));
		memset(nOutDIO, 0, Max_OutIOCnt*sizeof(int));
		memset(nAnalogInState, 0, AnalogINCnt);
		memset(nRobotStatus, 0, enRobotStatusCnt);
		memset(nModbusInDIO, 0, sizeof(nModbusInDIO));
		memset(nAnalogOutState, 0, sizeof(nAnalogOutState));

		memset(CommandVelocity, 0, sizeof(CommandVelocity));
		memset(motorTemperature, 0, sizeof(motorTemperature));
		memset(motorVoltage, 0, sizeof(motorVoltage));
		memset(Acceleration, 0, sizeof(Acceleration));
		memset(Dcceleration, 0, sizeof(Dcceleration));
		memset(dErrData, 0, sizeof(dErrData));
	}
}ST_RbtRealTimeStatu;
typedef struct _ST_RbtsStatu
{
	int nCurStatus;
	int nCurUDMDefIndex[enScriptThread_CNT];
	int nRbtCNT;	
	int nSimulation;//控制器仿真与打印信息状态
	int nScriptRunState;//0xXY ,X代表是否运行，Y代表是否暂停
	int nEmergencyStop;
	int nRbtForceSensorMode[Max_RobotCnt];
	//串口IO
	int nSerialInDIO[TOTALIO_CNT];
	int nSerialOutDIO[TOTALIO_CNT];
	int nSerialAnalogMode[SERIALANALOGDO_CNT];
	int nRtosStatus;
	int nMasterStatus;
	int nSlaveStatus;
	int nSlaveDropState;
	int nFrameResponseErrorCnt;
//	int nRTOSWatchDog;
	int nRemotePoweroff;
	double dbAnalog_48V_VoltagAct;		//48v_control电压检测，电压值=(0.02719*码值)V
	double dbAnalog_5V_VoltagAct;		//5伏电压检测，电压值=(0.00244*码值)V
	double nSerialAnalogCH[SERIALANALOG_CNT];		//模拟量输入，
	double dbAnalog_24V_VoltagAct;		//24伏电压检测，电压值=（0.01343*码值）V
	double dbAnalog_24V_CurrentAct;	//24伏电流检测，电流值=（0.00197*码值）A
	double dbAnalog_12V_VoltagAct;		//12伏电压检测，电压值=（0.00642*码值）V
	double dbAnalog_12V_CurrentAct;	//12伏电流检测，电流值=（0.00197*码值）A
	double dbAnalog_48V_CurrentAct1;	//48伏电流检测1，电流值=（0.01831*码值-37.5）A
	double dbAnalog_48V_CurrentAct2;	//48伏电流检测2 48伏电流为1和2项之和，电流值=（0.01831*码值-37.5）A
	double dbAnalog_48VOUT_VoltagAct;	//48V_OUT电压，电压值=(0.02563*码值)V
	double dbAnalog_48VIN_VoltagAct;	//48V_IN电压，电压值=(0.02563*码值)V
	double dbSerialAnalogOutput[SERIALANALOG_CNT];
	ST_RbtRealTimeStatu rbtsStatus[Max_RobotCnt];
	_ST_RbtsStatu() :nCurStatus(0), nRbtCNT(0), nSimulation(0), nScriptRunState(0), nEmergencyStop(0),
		nRtosStatus(0), nMasterStatus(0), nSlaveStatus(0), nRemotePoweroff(0),
		dbAnalog_48V_VoltagAct(0), dbAnalog_5V_VoltagAct(0),
		dbAnalog_24V_VoltagAct(0), dbAnalog_24V_CurrentAct(0),
		dbAnalog_12V_VoltagAct(0), dbAnalog_12V_CurrentAct(0),
		dbAnalog_48V_CurrentAct1(0), dbAnalog_48V_CurrentAct2(0),
		dbAnalog_48VOUT_VoltagAct(0), dbAnalog_48VIN_VoltagAct(0),
		nFrameResponseErrorCnt(0), nSlaveDropState(0)
	{
		for (int i = 0; i < enScriptThread_CNT; ++i)
		{
			nCurUDMDefIndex[i] = -1;
		}

		memset(nSerialInDIO, 0, sizeof(int)*TOTALIO_CNT);
		memset(nSerialOutDIO, 0, sizeof(int)*TOTALIO_CNT);
		memset(nSerialAnalogCH, 0, sizeof(nSerialAnalogCH));
		memset(nSerialAnalogMode, 0, sizeof(nSerialAnalogMode));
		memset(dbSerialAnalogOutput, 0, sizeof(dbSerialAnalogOutput));
		memset(nRbtForceSensorMode, 0, sizeof(nRbtForceSensorMode));
	}
}ST_RbtsStatu;

//RbTL协议头部
typedef struct _ST_RealTimeStatuNet
{
	int dwMagicNum;	//0 magic number, always 0x5242544C
	int dwProcoVec;
	int dwEncrypt;	//EncryptType
	int dwPackLen;//pack len, network byte
	int dwDataLen;//data len, network byte
	ST_RbtsStatu rbtStatus;
	_ST_RealTimeStatuNet() : dwMagicNum(0), dwProcoVec(0), dwEncrypt(0),
		dwPackLen(sizeof(_ST_RealTimeStatuNet)), dwDataLen(sizeof(ST_RbtsStatu))
	{}
}ST_RealTimeStatuNet;



typedef struct _ST_RbtRealTime10005Statu
{
	int nCurStatus;
	int nEmergencyStop;
	// 获取机器人状态 INT SIZE=8
	int nRobotStatus_Moving;
	int nRobotStatus_Power;
	// 获取错误信息 INT SIZE=2
	int nErrCode;
	double dConveyorVal;
	double dConveyorPos;
	ST_AxisData rbtPosACS;
	ST_AxisData rbtPosPCS;
	ST_AxisData rbtPosPCSForBase;
	ST_AxisData rbtPosTOOL;
	ST_AxisData motorCurrent;
	ST_AxisData Acceleration;
	_ST_RbtRealTime10005Statu() :nRobotStatus_Moving(0), nRobotStatus_Power(0),
		nErrCode(0), dConveyorVal(0.0), dConveyorPos(0.0)
	{
	}
}ST_RbtRealTime10005Statu;

typedef struct _ST_10005StatuNet
{
	int dwMagicNum;	//0 magic number, always 0x5242544C
	int dwProcoVec1;
	int dwProcoVec2;
	int dwPackLen;//pack len, network byte
	int dwDataLen;//data len, network byte

	ST_RbtRealTime10005Statu rbtStatus;
	_ST_10005StatuNet() : dwMagicNum(0x5242544C), dwProcoVec1(5), dwProcoVec2(509),
		dwPackLen(sizeof(_ST_10005StatuNet)), dwDataLen(sizeof(ST_RbtRealTime10005Statu))
	{}
}_ST_10005StatuNet;



#pragma pack(pop)


typedef enum EncryptType
{
	enNotEncrypt = 0,
	enEncrypt,
	enSetAESKey,
	enAckSetAESKey
};

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define HMRC_ConfigModbus	"HMRC/HR_Modbus"
#define Node_ModRoot		"HR_Modbus"
#define Node_ModDeviceList	"ModbusDeviceList"
#define Node_ModDevice		"ModbusDevice"
#define Node_ModSignal		"ModSignalNode"

#define Attrib_ModIP		"ModIP"
#define Attrib_ModPort		"ModPort"
#define Attrib_ModSlaveID	"ModSlave"

#define Attrib_ModSignalName		"ModSignalName"
#define Attrib_ModSignalType		"ModSignalType"
#define Attrib_ModSignalRegister	"ModSignalRegister"

#define HMRC_AppConfigChildCnt 15
#define HMRC_HardConfigChildCnt 19

#define HMRC_Config "HMRC"
#define HMRC_ROBOT_Pre "HMRC/Robot"
#define ROBOT_Pre "Robot"
#define HMR_Application "Application"

#define  Log "Log"
#define  LogStartTime "LogStartTime"
#define  LogEndTime "LogEndTime"
#define  LogLevel "LogLevel"

#define  LogBackError "sBackLogError"
#define  LogBackLevel "sBackLogLevel"
#define  LogBackInfo "sBackLogInfo"
#define  LogBackTime "sBackLogTime"

#define  HR_SerialPort "HR_SerialPort"

#define Attrib_J1 "J1"
#define Attrib_J2 "J2"
#define Attrib_J3 "J3"
#define Attrib_J4 "J4"
#define Attrib_J5 "J5"
#define Attrib_J6 "J6"

#define Attrib_X "X"
#define Attrib_Y "Y"
#define Attrib_Z "Z"
#define Attrib_A "A"
#define Attrib_B "B"
#define Attrib_C "C"

#define Attrib_X_MoveC "X_MoveC"
#define Attrib_Y_MoveC "Y_MoveC"
#define Attrib_Z_MoveC "Z_MoveC"
#define Attrib_A_MoveC "A_MoveC"
#define Attrib_B_MoveC "B_MoveC"
#define Attrib_C_MoveC "C_MoveC"

#define Attrib_Axis1 "Axis1"
#define Attrib_Axis2 "Axis2"
#define Attrib_Axis3 "Axis3"
#define Attrib_Axis4 "Axis4"
#define Attrib_Axis5 "Axis5"
#define Attrib_Axis6 "Axis6"


#define Attrib_Val1 "Val1"
#define Attrib_Val2 "Val2"
#define Attrib_Val3 "Val3"
#define Attrib_Val4 "Val4"
#define Attrib_Val5 "Val5"
#define Attrib_Val6 "Val6"
#define Attrib_Val7 "Val7"
#define Attrib_Val8 "Val8"
#define Attrib_Val9 "Val9"
#define Attrib_Val10 "Val10"
#define Attrib_Val11 "Val11"
#define Attrib_Val12 "Val12"
#define Attrib_Val13 "Val13"

#define Attrib_Slave1 "Slave1"
#define Attrib_Slave2 "Slave2"
#define Attrib_Slave3 "Slave3"

#define Attrib_a2	 "a2"
#define Attrib_d6	 "d6"
#define Attrib_d4	 "d4"
#define Attrib_d1	 "d1"

#define Attrib_Index "Index"
#define Attrib_Name "Name"
#define Attrib_Alias "Alias"
#define Attrib_Default "Default"
#define Attrib_Enabled "Enabled"
#define Attrib_TriggerType "TriggerType"
#define Attrib_Function "Function"
#define Attrib_Reversal "Reversal"
#define Attrib_IOBit "IOBit"
#define Attrib_AxisID "AxisID"
#define Attrib_Type "Type"

#define Attrib_Level "Level"
#define Attrib_CollideTimeout "CollideTimeout"
#define Attrib_CollideMaxDegree "CollideMaxDegree"

#define Attrib_InposTimeout "InposTimeout"
#define Attrib_InposJoint "InposJoint"
#define Attrib_InposCartesian "InposCartesian"

#define RobotModelType "RobotModelType"
#define TeachModelType "TeachModelType"
#define MachineOrigin "MachineOrigin"
#define MaxActutorCurrents "MaxActutorCurrents"
#define ConveyorInfo_Scale "ConveyorInfo_Scale"
#define ForceControlMode "ForceControlMode"
#define FeedForwardParam "FeedForwardParam"
#define EnableFeedForwardMode "EnableFeedForwardMode"
#define EnableFeedForwardVelocityMode "EnableFeedForwardVelocityMode"
#define CollideStopNotDisable "CollideStopNotDisable"
#define IsAbsoluteEncode "IsAbsoluteEncode"
#define AssistiveCheckTime "AssistiveCheckTime"
#define TorqueCompensationType "TorqueCompensationType"
#define ToolsCoord "ToolsCoord"
#define UCSCoord "UCSCoord"
#define SysPoints "SysPoints"
#define EndOutIO "EndOutIO"
#define EndInIO "EndInIO"
#define ConfigInIO "ConfigInIO"
#define ConfigOutIO "ConfigOutIO"
#define Gravity "Gravity"
#define PayloadParam "Gravity/Payload"
#define PayloadParam_Mass "Mass"
#define PayloadParam_CenterX "CenterX"
#define PayloadParam_CenterY "CenterY"
#define PayloadParam_CenterZ "CenterZ"
#define PowerOffNoStop "PowerOffNoStop"
#define FrameResponseErrorMaxCnt "FrameResponseErrorMaxCnt"
#define ForceSensor "ForceSensor"
#define ForceSensorCalibration "ForceSensorCalibration"
#define ForceSensorControlMode "ForceSensorControlMode"
#define ForceSensorEnableInfo "ForceSensor/EnableInfo"
#define ForceSensorVeldamp "ForceSensor/VelDamp"
#define ForceSensorStopDamp "ForceSensor/StopDamp"
#define ForceSensorAdmitMass "ForceSensor/AdmitMass"
#define ForceSensorForceLimit "ForceSensor/ForceLimit"
#define ForceSensorCycleTime "ForceSensor/CycleTime"
#define ForceSensorTorqueLimit "ForceSensor/TorqueLimit"
#define ForceSensorControlType "ForceSensor/ControlType"
#define ForceSensorPhysicParams "ForceSensor/PhysicParams"
#define ForceSensorSearchSpeed "FTSearchSpeed"
#define ForceSensorRotation "ForceSensor/Rotation"
#define ForceSensorPhysicParamMass "Mass"
#define ForceSensorPhysicParamFrictionForce "FrictionForce"
#define ForceSensorPhysicParamInertia "Inertia"
#define ForceSensorPhysicParamFrictionTorque "FrictionTorque"
#define ForceSensorThreshold "ForceSensor/Threshold"
#define ForceSensorJointSpeedLimitForceThreshold "ForceThreshold"
#define ForceSensorJointSpeedLimitTorqueThreshold "TorqueThreshold"
#define ForceSensorSearchSpeedLinVel "LinVel"
#define ForceSensorSearchSpeedAngVel "AngVel"
#define CheckTemperatureValue "CheckTemperatureValue"
#define Attrib_CheckTemperatureSlave0 "Slave0"
#define Attrib_CheckTemperatureSlave1 "Slave1"
#define Attrib_CheckTemperatureSlave2 "Slave2"
#define Attrib_LinVel "LinVel"
#define Attrib_AngVel "AngVel"


#define Attrib_ForceSensorRotation "Rotation"
#define Attrib_ForceSensorEnable "Enabled"
#define Attrib_ForceSensorPortType   "PortType"
#define Attrib_ForceSensorPortName   "PortName"
#define Attrib_ForceSensorPortIP	 "PortIP"
#define Attrib_ForceSensorPortNumber "PortNum"
#define Attrib_ForceSensorIsUseForceSensor "IsUseForceSensor"
#define Attrib_ForceSensorPositionType "PositionType"
#define Attrib_ForceSensorImplement "Implement"

#define ForceControl "ForceControl"
#define ForceControlFTSensorPos "ForceControl/FTSensorPos"
#define ForceControlMaxPIDParams "ForceControl/MaxPIDParams"
#define ForceControlFTGoalMaxValue "ForceControl/FTGoalMaxValue"
#define ForceControlControlFreedom "ForceControl/ControlFreedom"
#define ForceControlPIDControlParams "ForceControl/PIDControlParams"
#define ForceControlFTControlGoal "ForceControl/FTControlGoal"

#define Attrib_ForceControlfP "fP"
#define Attrib_ForceControlfI "fI"
#define Attrib_ForceControlfD "fD"
#define Attrib_ForceControltP "tP"
#define Attrib_ForceControltI "tI"
#define Attrib_ForceControltD "tD"
#define Attrib_ForceControlForceMaxValue "ForceMaxValue"
#define Attrib_ForceControlTorqueMaxValue "TorqueMaxValue"
#define Attrib_ForceControlRx "Rx"
#define Attrib_ForceControlRy "Ry"
#define Attrib_ForceControlRz "Rz"

#define Attrib_ForceControlGoalForce1 "GoalForce1"
#define Attrib_ForceControlGoalForce2 "GoalForce2"
#define Attrib_ForceControlGoalForce3 "GoalForce3"
#define Attrib_ForceControlGoalTorque1 "GoalTorque1"
#define Attrib_ForceControlGoalTorque2 "GoalTorque2"
#define Attrib_ForceControlGoalTorque3 "GoalTorque3"

#define Attrib_MoveZIntervalDist "MoveZParam/IntervalDist"
#define Attrib_MoveZWidthCompensation "MoveZParam/WidthCompensation"
#define Attrib_DragMoveAlgorithmType "DragMoveParam/AlgorithmType"

#define Attrib_Fx0 "Fx0"
#define Attrib_Fy0 "Fy0"
#define Attrib_Fz0 "Fz0"
#define Attrib_Mx0 "Mx0"
#define Attrib_My0 "My0"
#define Attrib_Mz0 "Mz0"
#define Attrib_G   "G"
#define Attrib_Tx  "x"
#define Attrib_Ty  "y"
#define Attrib_Tz  "z"

#define Attrib_CtrolModeX "X"
#define Attrib_CtrolModeY "Y"
#define Attrib_CtrolModeZ "Z"
#define Attrib_CtrolModeRX "RX"
#define Attrib_CtrolModeRY "RY"
#define Attrib_CtrolModeRZ "RZ"

#define Modbus "Modbus"
//#define Attrib_Type "Type"
#define Attrib_Adrress "Adrress"
//#define Attrib_Name "Name"
#define Attrib_Frequency "Frequency"
#define Attrib_ResponseTime "ResponseTime"
#define Attrib_Slave "Slave"
#define Attrib_Value "Value"

#define Attrib_PointName "PointName"
#define Attrib_PointAlis "PointAlis"
#define Attrib_PointIsTeached "PointIsTeached"

#define Collide_Stop "CollideStop_New"//更改节点名称
#define StopThresholdGear "CollideStop_New/StopThresholds_gear"//更改节点名称
#define StopThresholds "CollideStop/StopThresholds"
#define CollideLevel "CollideStopLevel"
#define Collide_Mode "CollideMode"
#define CollideStopLevel "CollideMode/CollideStopLevel"
#define CollideToFreeDriveParam "CollideToFreeDrive/CollideToFreeDriveParam"

#define BaseMounting "BaseMountingAngle/BaseMounting"
#define BaseMountingAngle "BaseMountingAngle"
#define Attrib_Mountingrotation "Mountingrotation"
#define Attrib_Mountingtilt "Mountingtilt"

#define BlendingPos "BlendingPosError/BlendingPos"
#define BlendingPosError "BlendingPosError"
#define Attrib_AcsPosError "AcsPosError"
#define Attrib_PcsPosError "PcsPosError"

#define AvoidResonantEnable "AvoidResonant"
#define Attrib_AvoidResonantEnable "Enabled"
#define Attrib_AvoidResonantMin "Min"
#define Attrib_AvoidResonantMax "Max"

#define AssistiveMode_Thresholds "CollideStop_AssistiveMode_Thresholds"
#define Velocity_thresholds "CollideStop_Velocity_thresholds"
#define FrictionCompensationFactor "CollideStop_FrictionCompensationFactor"
#define FrictionWithTemperature "CollideStop_FrictionWithTemperature"
#define MotorScales "CollideStop_MotorScales"
#define torqueconstant "MotorInfo_torqueconstant"
#define maxefficiency "MotorInfo_maxefficiency"
#define dampConstant "MotorInfo_dampConstant"
#define mountingrotation "BaseMountingAngle_mountingrotation"
#define mountingtilt "BaseMountingAngle_mountingtilt"
#define KinematicsParameters "KinematicsParameters"
#define dynamicsparameters1 "dynamicsparameters1"
#define dynamicsparameters2 "dynamicsparameters2"
#define dynamicsparameters3 "dynamicsparameters3"
#define dynamicsparameters4 "dynamicsparameters4"
#define dynamicsparameters5 "dynamicsparameters5"
#define dynamicsparameters6 "dynamicsparameters6"
#define ELFIN_SS_ACS_Range_J1Min "ELFIN_SS_ACS/Range_J1Min"
#define ELFIN_SS_ACS_Range_J1Max "ELFIN_SS_ACS/Range_J1Max"
#define ELFIN_SS_ACS_Range_J2Min "ELFIN_SS_ACS/Range_J2Min"
#define ELFIN_SS_ACS_Range_J2Max "ELFIN_SS_ACS/Range_J2Max"
#define ELFIN_SS_ACS_Range_J3Min "ELFIN_SS_ACS/Range_J3Min"
#define ELFIN_SS_ACS_Range_J3Max "ELFIN_SS_ACS/Range_J3Max"
#define ELFIN_SS_ACS_Range_J4Min "ELFIN_SS_ACS/Range_J4Min"
#define ELFIN_SS_ACS_Range_J4Max "ELFIN_SS_ACS/Range_J4Max"
#define ELFIN_SS_ACS_Range_J5Min "ELFIN_SS_ACS/Range_J5Min"
#define ELFIN_SS_ACS_Range_J5Max "ELFIN_SS_ACS/Range_J5Max"
#define ELFIN_SS_ACS_Range_J6Min "ELFIN_SS_ACS/Range_J6Min"
#define ELFIN_SS_ACS_Range_J6Max "ELFIN_SS_ACS/Range_J6Max"
#define SafeSpace_ACS "ELFIN_SS_ACS"
#define ELFIN_SS_ACS_J1Min "ELFIN_SS_ACS/J1_Min"
#define ELFIN_SS_ACS_J1Max "ELFIN_SS_ACS/J1_Max"
#define ELFIN_SS_ACS_J2Min "ELFIN_SS_ACS/J2_Min"
#define ELFIN_SS_ACS_J2Max "ELFIN_SS_ACS/J2_Max"
#define ELFIN_SS_ACS_J3Min "ELFIN_SS_ACS/J3_Min"
#define ELFIN_SS_ACS_J3Max "ELFIN_SS_ACS/J3_Max"
#define ELFIN_SS_ACS_J4Min "ELFIN_SS_ACS/J4_Min"
#define ELFIN_SS_ACS_J4Max "ELFIN_SS_ACS/J4_Max"
#define ELFIN_SS_ACS_J5Min "ELFIN_SS_ACS/J5_Min"
#define ELFIN_SS_ACS_J5Max "ELFIN_SS_ACS/J5_Max"
#define ELFIN_SS_ACS_J6Min "ELFIN_SS_ACS/J6_Min"
#define ELFIN_SS_ACS_J6Max "ELFIN_SS_ACS/J6_Max"


#define ELFIN_SS_PCS_Range_XMin "ELFIN_SS_PCS/Range_XMin"
#define ELFIN_SS_PCS_Range_XMax "ELFIN_SS_PCS/Range_XMax"
#define ELFIN_SS_PCS_Range_YMin "ELFIN_SS_PCS/Range_YMin"
#define ELFIN_SS_PCS_Range_YMax "ELFIN_SS_PCS/Range_YMax"
#define ELFIN_SS_PCS_Range_ZMin "ELFIN_SS_PCS/Range_ZMin"
#define ELFIN_SS_PCS_Range_ZMax "ELFIN_SS_PCS/Range_ZMax"
#define ELFIN_SS_PCS_Range_AMin "ELFIN_SS_PCS/Range_AMin"
#define ELFIN_SS_PCS_Range_AMax "ELFIN_SS_PCS/Range_AMax"
#define ELFIN_SS_PCS_Range_BMin "ELFIN_SS_PCS/Range_BMin"
#define ELFIN_SS_PCS_Range_BMax "ELFIN_SS_PCS/Range_BMax"
#define ELFIN_SS_PCS_Range_CMin "ELFIN_SS_PCS/Range_CMin"
#define ELFIN_SS_PCS_Range_CMax "ELFIN_SS_PCS/Range_CMax"
#define SafeSpace_PCS "ELFIN_SS_PCS"
#define ELFIN_SS_PCS_XMin "ELFIN_SS_PCS/X_Min"
#define ELFIN_SS_PCS_XMax "ELFIN_SS_PCS/X_Max"
#define ELFIN_SS_PCS_YMin "ELFIN_SS_PCS/Y_Min"
#define ELFIN_SS_PCS_YMax "ELFIN_SS_PCS/Y_Max"
#define ELFIN_SS_PCS_ZMin "ELFIN_SS_PCS/Z_Min"
#define ELFIN_SS_PCS_ZMax "ELFIN_SS_PCS/Z_Max"
#define ELFIN_SS_PCS_AMin "ELFIN_SS_PCS/A_Min"
#define ELFIN_SS_PCS_AMax "ELFIN_SS_PCS/A_Max"
#define ELFIN_SS_PCS_BMin "ELFIN_SS_PCS/B_Min"
#define ELFIN_SS_PCS_BMax "ELFIN_SS_PCS/B_Max"
#define ELFIN_SS_PCS_CMin "ELFIN_SS_PCS/C_Min"
#define ELFIN_SS_PCS_CMax "ELFIN_SS_PCS/C_Max"

#define Move_PCSMotionLimits "Move_PCSMotionLimits"
#define Stop_PCSMotionLimits "Stop_PCSMotionLimits"
#define Speed_Linear "Speed_Linear"
#define Acceleration_Linear "Acceleration_Linear"
#define Jerk_Linear "Jerk_Linear"
#define Speed_Angular "Speed_Angular"
#define Acceleration_Angular "Acceleration_Angular"
#define Jerk_Angular "Jerk_Angular"

#define Move_JointMotionLimits "Move_JointMotionLimits"
#define Stop_JointMotionLimits "Stop_JointMotionLimits"
#define Speed_ACS "Speed_J"
#define Acceleration_ACS "Acceleration_J"
#define Jerk_ACS "Jerk_J"
#define Speed_J1 "Speed_J1"
#define Acceleration_J1 "Acceleration_J1"
#define Jerk_J1 "Jerk_J1"
#define Speed_J2 "Speed_J2"
#define Acceleration_J2 "Acceleration_J2"
#define Jerk_J2 "Jerk_J2"
#define Speed_J3 "Speed_J3"
#define Acceleration_J3 "Acceleration_J3"
#define Jerk_J3 "Jerk_J3"
#define Speed_J4 "Speed_J4"
#define Acceleration_J4 "Acceleration_J4"
#define Jerk_J4 "Jerk_J4"
#define Speed_J5 "Speed_J5"
#define Acceleration_J5 "Acceleration_J5"
#define Jerk_J5 "Jerk_J5"
#define Speed_J6 "Speed_J6"
#define Acceleration_J6 "Acceleration_J6"
#define Jerk_J6 "Jerk_J6"
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define Protocol_CmdName "hmRoot/hmData/CmdName"
#define Protocol_RbtID "hmRoot/hmData/Protocol_RbtID"
#define Protocol_CmdRespone "hmRoot/hmData/CmdRespone"
#define Protocol_CmdType "hmRoot/hmData/CmdType"
#define Protocol_ParamKey "hmRoot/hmData/paramKey"

//////////////////////////////////////////////////////////////////////////
//IF指令
//////////////////////////////////////////////////////////////////////////

#define OPEmpty "<Empty>"
#define OPTOK "OK"
#define OPTFAIL "Fail"
#define IFSplit ","
#define IFEnd ",;"
#define FAIL_RESULT_FORMAT "%s,Fail,%d,;" 

#define Protocol_ReadMachineOrigin "ReadMachineOrigin"
#define Protocol_ReadMultiEndatPos "ReadMultiEndatPos"

#define Protocol_SetSimulation "SetSimulation"
#define Protocol_SetHMDriveOperation "SetHMDriveOperation"
#define Protocol_SetDragMoveCalType "SetDragMoveCalType"
#define Protocol_SetLEDColour "SetLEDColour"

#define Protocol_Blackout		"BlackOut"
#define Protocol_StartMaster		"StartMaster"
#define Protocol_CloseMaster		"CloseMaster"
#define Protocol_Electrify		"Electrify"
#define Protocol_GetVersion		"GetVersion"
#define Protocol_ShutDown		"ShutDown"
#define Protocol_ElectrifyNoHoming		"ElectrifyNoHoming"

#define Protocol_ElfinBrakeOnly	"ElfinBrakeOnly"
#define Protocol_ElfinHomeStep2	"ElfinHomeStep2"
#define Protocol_ElfinRelay		"ElfinRelay"
#define Protocol_EncoderSetHomeEndatPos "EncoderSetHomeEndatPos"
#define Protocol_ForceSensorCalibration "ForceSensorCalibration"
//#define Protocol_StartAssistiveMode			"StartAssistiveMode"
//#define Protocol_CloseAssistiveMode			"CloseAssistiveMode"
#define Protocol_SetFeedforwardVelocityMode	"SetFeedforwardVelocityMode" 
//#define Protocol_SetCollisionStopMode			"SetCollisionStopMode"
#define Protocol_SetCollisionMomentum			"SetCollisionMomentum"

#define Protocol_MoveV					"MoveV"
#define Protocol_MoveBP					"MoveBP"
#define Protocol_MoveCC					"MoveCC"
#define Protocol_MoveCL					"MoveCL"
#define Protocol_MoveCCL					"MoveCCL"
#define Protocol_MoveLCL					"MoveLCL"
#define Protocol_MoveVex					"MoveVex"
#define Protocol_MoveGate				"MoveGate"

#define Protocol_MoveBEX					"MoveBEX"
#define Protocol_CalcRelBEX					"CalcRelBEX"

#define Protocol_Blending				"Blending"
#define Protocol_SetBlendingStatus		"SetBlendingStatus"
#define Protocol_PushBlendingMoveL		"PushBlendingMoveL"
#define Protocol_PushBlendingMoveC		"PushBlendingMoveC"
//#define Protocol_RobotIQ					"RobotIQ"
#define Protocol_RobotIQReset			"RobotIQReset"
#define Protocol_RobotiqStatus			"RobotiqStatus"

#define Protocol_SetOutIOConfig			"SetOutIOConfig"
#define Protocol_SetInIOConfig			"SetInIOConfig"
#define Protocol_SetKineCoordInfo			"SetKineCoordInfo"
#define Protocol_SetUserCoordInfo			"SetUserCoordInfo"
#define Protocol_SetSystemPointInfo		"SetSystemPointInfo"
//#define Protocol_SetPcsSafeSpaceLimit				"SetPcsSafeSpaceLimit"
//#define Protocol_SetAcsSafeSpaceLimit				"SetAcsSafeSpaceLimit"

//#define Protocol_SetToolCoordinateMotion			"SetToolCoordinateMotion"

#define Protocol_SetDragMoveParameters			"SetDragMoveParameters"
#define Protocol_SetDragMoveStatus				"SetDragMoveStatus"
#define Protocol_SetDragMovePoints				"SetDragMovePoints"
#define Protocol_SetConveyorScale				"SetConveyorScale"
#define Protocol_SetHomePosition					"SetHomePosition"
#define Protocol_SetKinematicCoordinate			"SetKinematicCoordinate"
#define Protocol_SetTCP							"SetTCP"
//#define Protocol_SetOverride						"SetOverride"
#define Protocol_SetOutIOState					"SetOutIOState"
#define Protocol_SetControlDOState				"SetControlDOState"
#define Protocol_SetPcsPosition					"SetPcsPosition"
#define Protocol_SetPrintSDKLog					"SetPrintRtos"
#define Protocol_SetSerialDO						"SetSerialDO"
#define Protocol_SetConfigDO						"SetConfigDO"
#define Protocol_SetSerialAnalogOutput			"SetSerialAnalogOutput"
#define Protocol_SetSerialAnalogMode				"SetSerialAnalogMode"
#define Protocol_SetTrackingSwitch				"SetTrackingSwitch"
#define Protocol_SetUserCoordinate				"SetUserCoordinate"
#define Protocol_SetUCS							"SetUCS"
#define Protocol_SetCollideStopThresholdGear		"SetSafeStopGear"	//add by tml 2018-11-15

#define Protocol_SetReduce				"SetReduce"

#define Protocol_SetCollideStopLevel				"SetSafeStopLevel"	
#define Protocol_SetAssistiveModeThresholds		"SetZeroForce"
#define Protocol_SetCollideStopPayload			"SetPayload"
#define Protocol_SetSpeedUp						"SetSpeeProtocol_SetPayloaddUp"
#define Protocol_SetSpeedDown					"SetSpeedDown"
//#define Protocol_SetMoveJointMotionLimits		"SetMoveJointMotionLimits"
//#define Protocol_SetMovePcsMotionLimits			"SetMovePcsMotionLimits"
//#define Protocol_SetStopJointMotionLimits		"SetStopJointMotionLimits"
//#define Protocol_SetStopPcsMotionLimits			"SetStopPcsMotionLimits"
#define Protocol_SetPayloadRecognitionStart		"SetPayloadRecognitionStart"
//#define Protocol_SetBaseMountingAngle			"SetBaseMountingAngle"
#define Protocol_StartUDMTimer					"StartUDMTimer"
#define Protocol_CloseUDMTimer					"CloseUDMTimer"
#define Protocol_SetStartTimer					"SetStartTimer"
#define Protocol_SetCloseTimer					"SetCloseTimer"

#define Protocol_ReadCurTCP						"ReadCurTCP"
#define Protocol_ReadCurUCS						"ReadCurUCS"
#define Protocol_ReadCurPayload					"ReadCurPayload"

#define Protocol_ReadAcsActualPos			"ReadAcsActualPos"
#define Protocol_ReadPcsActualPos			"ReadPcsActualPos"
#define Protocol_ReadPcs2UCSTCP				"ReadPcs2UCSTCP"

#define Protocol_ReadRobotPosInfo			"ReadRobotPosInfo"
#define Protocol_ReadOverride				"ReadOverride"
#define Protocol_ReadRobotState				"ReadRobotState"
#define Protocol_ReadMoveState				"ReadMoveState"
#define Protocol_ReadInIOState				"ReadInIOState"
#define Protocol_ReadControlInIOState		"ReadControlInIOState"
#define Protocol_ReadOutIOState				"ReadOutIOState"
#define Protocol_ReadMultiInIOState			"ReadMultiInIOState"
#define Protocol_ReadMultiOutIOState			"ReadMultiOutIOState"
#define Protocol_ReadAnalogInput				"ReadAnalogInput"
#define Protocol_ReadMultiSerialDI			"ReadMultiSerialDI"
#define Protocol_ReadSerialAllDI				"ReadSerialAllDI"
#define Protocol_ReadSerialAllDO				"ReadSerialAllDO"
#define Protocol_ReadSerialDI				"ReadSerialDI"
#define Protocol_ReadConfigDI				"ReadConfigDI"
#define Protocol_ReadSerialDO				"ReadSerialDO"
#define Protocol_ReadConfigDO				"ReadConfigDO"
#define Protocol_ReadSerialAnalog			"ReadSerialAnalog"
#define Protocol_ReadUDMCurMD5				"ReadUDMCurMD5"
#define Protocol_ReadScriptHold				"ReadScriptHold"
#define Protocol_GetInverseKin				"GetInverseKin"
#define Protocol_PCS2ACS					"PCS2ACS"
#define Protocol_GetPoseInterpolate			"GetPoseInterpolate"
#define Protocol_GetMatrix2PCS				"GetMatrix2PCS"
#define Protocol_GetPCS2Matrix				"GetPCS2Matrix"
#define Protocol_PCS2ACS					"PCS2ACS"
#define Protocol_PCSWithTCPUCS2ACS			"PCSWithTCPUCS2ACS"
#define Protocol_GetForwardKin				"GetForwardKin"
#define Protocol_SetRobotiq					"SetRobotiq"

#define Protocol_ReadConveyorValue			"ReadConveyorValue"
//#define Protocol_ReadConveyorPos			"ReadConveyorPos"
#define Protocol_ReadErrorInfo				"ReadErrorInfo"
#define Protocol_ReadMultiEndatPos			"ReadMultiEndatPos"
#define Protocol_ReadSafeSpaceLimit			"ReadSafeSpaceLimit"
#define Protocol_ReadRobotParamInfo			"ReadRobotParamInfo"
#define Protocol_ReadPayloadRecognition		"ReadPayloadRecognition"
#define Protocol_ReadRobotInfoParam			"ReadRobotInfoParam"

#define Protocol_RunScriptFunc				"RunScriptFunc"
#define Protocol_HoldScriptFunc			    "HoldScriptFunc"
#define Protocol_ContinusScriptFunc		    "ContinusScriptFunc"

#define Protocol_ReadScriptMainFirstPoint	"ReadScriptMainFirstPoint"

#define Protocol_StepRunScriptFunc		    "StepRunScriptFunc"
#define Protocol_StepNextScript			    "StepNextScript"
#define Protocol_StopRunUDM					"StopRunUDM"

#define Protocol_SetRunningMode   			"SetRunningMode"
#define Protocol_ChangeDCSStatu   			"ChangeDCSStatu"

#define Protocol_ReadCurrentScript			"ReadCurrentScript"
#define Protocol_ReadSystemPointInfo		"ReadSystemPointInfo"
#define Protocol_UpdateSystemPointInfo		"UpdateSystemPointInfo"



//////////////////////////////////////////////////////////////////////////
//Tree Script
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
#define Tree_PreScriptHead "ScriptFile"  //第一级
#define Tree_ScriptHead "ScriptFile/Instruct/FuncInfo_" //第二级
#define Tree_InstructHead "ScriptFile/Instruct"
#define Tree_VariantHead "ScriptFile/Global_Variant" //第二级
#define Tree_PointListHead "ScriptFile/Global_PointList" //第二级
#define Tree_LocalVariantHead "ScriptFile/Local_Variant" //第二级
#define Tree_DragMoveHead "ScriptFile/Global_TrajectList" //第二级

#define Tree_ScriptName "ScriptName"  //第一级

#define Tree_RegsiterHead "ScriptFile/Regsiter" //第二级
#define Tree_FuncIndex "FuncInfo_"  //指令索引  FuncInfo_0,FuncInfo_1,FuncInfo_2
#define Tree_VarIndex  "Var_"       //变量索引  Var_0,Var_1,Var_2
#define Tree_PointIndex "Point_"	//变量索引 Point_Name

#define Tree_RegsiterEAX "ScriptFile/Regsiter/EAX"
#define Tree_RegsiterEBX "ScriptFile/Regsiter/EBX"
#define Tree_RegsiterECX "ScriptFile/Regsiter/ECX"

#define Tree_VarMarkName  "Global"  
#define Tree_VarListName  "Var"  
//-------------指令枚举-------------//
enum EN_ScripCmd
{
	enScriptCmd_Start = 0,
	//////////////////////////////////////////////////////////////////////////
	enScriptCmd_Function = 0,
	enScriptCmd_Script,
	enScriptCmd_Timer,
	// 	enScriptCmd_MoveL,
	// 	enScriptCmd_MoveJ,
	enScriptCmd_Motion,

	enScriptCmd_SetIO,
	enScriptCmd_Wait,
	enScriptCmd_Loop,
	enScriptCmd_Call,
	enScriptCmd_If,
	enScriptCmd_ElseIf,
	enScriptCmd_Else = 10,
	enScriptCmd_SetRobot,
	enScriptCmd_Communicate,
	enScriptCmd_Send,
	enScriptCmd_Receive,
	enScriptCmd_Return,
	enScriptCmd_SetValue,
	//enScriptCmd_SetOverride,
	enScriptCmd_ModbusWrite,
	enScriptCmd_ModbusRead,
	enScriptCmd_ReadActPos,//获取机器人的实际位置
	enScriptCmd_DragMove = 20,
	enScriptCmd_ReadSerial,
	enScriptCmd_StringAdd,
	enScriptCmd_Blending,
	enScriptCmd_SetRobotiq,
	enScriptCmd_MoveZ,
	enScriptCmd_Log,
	enScriptCmd_Way,
	enScriptCmd_WayPoint,
	enScriptCmd_Seek,
	enScriptCmd_Stack = 30,
	enScriptCmd_StackDir,
	enScriptCmd_StackSeq,
	enScriptCmd_Pallet,
	enScriptCmd_PalletPatternSel,
	enScriptCmd_PalletPattern,
	enScriptCmd_SetMoreValue,
	enScriptCmd_Break,
	//////////////////////////////////////////////////////////////////////////
	enScriptCmd_Root,

	enScriptCmd_Empty,
	enScriptCmd_Jump,
	enScriptCmd_pushs,
	enScriptCmd_pops,
	enScriptCmd_WaitSetRbt,
	enScriptCmd_WaitMotionDone,
	enScriptCmd_WaitSeekDone,
	enScriptCmd_JudgeSeekStart,
	enScriptCmd_JudgeStackEnd,
	enScriptCmd_JudgeStackEndJumpTo,
	enScriptCmd_WaitRobotiq,
	enScriptCmd_ReadOverride,
	enScriptCmd_ReadConveyorValue,
	
	enScriptCmdCnt,
};

//////////////////////////////////////////////////////////////////////////
//SetValue指令的类型枚举
//enVarDoType_Const=选择常量
//enVarDoType_Var=选择变量
//////////////////////////////////////////////////////////////////////////
enum EN_VarDoType
{
	enVarDoType_Const = 0,
	enVarDoType_Var,
	enVarDoType_Local
};
//////////////////////////////////////////////////////////////////////////
//Modbus数据类型
//enModDataType_CoilStatus线圈寄存器状态
//enModDataType_InputStatus输入寄存器状态
//enModDataType_HoldingReg保持寄存器
//enModDataType_InputReg输入寄存器
//////////////////////////////////////////////////////////////////////////
enum EN_ModbusDataType
{
	enModDataType_CoilStatus = 0,
	enModDataType_InputStatus,
	enModDataType_HoldingReg,
	enModDataType_InputReg,
	enModDataType_Cnt
};

enum EN_BoardType
{
	enSerial_Disable = 0,
	enSerial_HM,
	enSerial_Anit,
	enSerial_HMV3,
	enSerial_HRV4
};

//-------------指令宏定义------------//
#define TCmd_Root "Script Root"

#define TCmd_Function "Function"
#define TCmd_Script   "Script"
//#define TCmd_MoveL    "MoveL"
//#define TCmd_MoveJ    "MoveJ"
#define TCmd_Motion	  "Move"//将运动指令集合

#define TCmd_Wait     "Wait"
#define TCmd_SetIO    "SetIO"
#define TCmd_SetRbt   "SetRobot"
#define TCmd_Call     "Call"
#define TCmd_Loop     "Loop"
#define TCmd_If       "If"
#define TCmd_ElseIf   "ElseIf"
#define TCmd_Else     "Else"
#define TCmd_Break     "Break"
#define TCmd_Return   "Return"
#define TCmd_Timer    "Timer"
#define TCmd_Communicate   "Communicate"
#define TCmd_Send   "Send"
#define TCmd_Receive   "Receive"
#define TCmd_SetValue   "SetValue"
#define TCmd_SetMoreValue   "SetMoreValue"

#define TCmd_SetSpeed   "SetSpeed"
#define TCmd_ReadSpeed   "ReadSpeed"
#define TCmd_ModbusWrite   "ModbusWrite"
#define TCmd_ModbusRead   "ModbusRead"
#define TCmd_ReadActPos   "ReadPos"
#define TCmd_DragMove	"DragMove"
#define TCmd_ReadSerial "ReadInput"
#define TCmd_StringAdd "StringAdd"
#define TCmd_Blending "Blending"
#define TCmd_RobotIQ	"SetRobotiq"

#define TCmd_MoveZ	"MoveZ"
#define TCmd_Log   			"Log"
#define TCmd_Way   			"Way"
#define TCmd_WayPoint   	"WayPoint"
#define TCmd_Seek   	"Seek"
#define TCmd_Stack   	"Stack"
#define TCmd_Direction   	"Direction"
// #define TCmd_Stack   	"Stack"
// #define TCmd_Seek   	"Seek"
// #define TCmd_Stack   	"Stack"

#define TCmd_Pallet		"Pallet"
#define TCmd_PalletSel	"PalletSel"
#define TCmd_PalletPattern "Pattern"

#define TCmd_ReadConveyorValue "ReadConveyorValue"

#define TCmd_ConnectModbusTCP   "ConnectModbusTCP"

#define TCmd_WaitSetRbt "WaitSetRbt"
#define TCmd_WaitMotionDone "WaitMotionDone"
#define TCmd_WaitSeekDone "WaitSeekDone"
#define TCmd_JudgeSeekStart "JudgeSeekStart"
#define TCmd_JudgeStackEnd "JudgeStackEnd"
#define TCmd_JudgeStackEndJumpTo "JudgeStackEndJumpTo"
#define TCmd_WaitRobotIQ	"WaitSetRobotiq"

#define TCmd_GetPoseAddition	"GetPoseAddition"


//-----------------------------------//
#define VarName_Int "int"
#define VarName_Double "double"
#define VarName_String "string"
#define VarName_Socket "socket"

//-------------------------------------//
#define Attrib_IsCmdSave "nIsCmdSave"
#define Attrib_sTip "sTip"

#define Attrib_VarIndex "Index"
#define Attrib_VarInstructDisable "InstructDisable"
#define Attrib_VarName  "sName"
#define Attrib_ChannelIndex "ChannelIndex"
#define Attrib_ReadType "ReadType"
#define Attrib_SerialMask "sSerialMask"
#define Attrib_SerialRbtID "nRobotID"
#define Attrib_VarType  "sType"
//#define Attrib_VarSize  "sSize"
#define Attrib_VarSave2File  "sSave"
#define Attrib_VarArraySize  "sAyyaySize"


#define Attrib_VarValue "sValue"
#define Attrib_VarMark  "sMark"

#define Attrib_ConditionCount  "nCondCount"//判断条件的个数
#define Attrib_ConditionIndex  "nCondIndex_"//判断条件的标号
#define Attrib_CondLogicIndex  "nLogicIndex_"//判断条件的标号

//--------------commandDef------------//
#define Attrib_Func          "InstructName"//指令名字
#define Attrib_InstructType  "InstructType"//指令类型

#define Attrib_ContenType "ContenType"
#define Attrib_VariableType "VariableType"
#define Attrib_VariableName "VariableName"
//-----------------------------------//
#define Attrib_FuncType       "enType"
#define Attrib_FuncName       "sName"
#define Attrib_FuncReturn     "sReturn"
#define Attrib_FuncParamters  "sParamters"
#define Attrib_FuncContent	  "sContent"

#define Attrib_CmpVarType      "sCmpVarType"

#define Attrib_LoopType      "enType"
#define Attrib_LoopForever   "bLoopForever"
#define Attrib_LoopCount     "nCount"
#define Attrib_LoopCondition "sCondition"

#define Attrib_IfType      "enType"
#define Attrib_IfRobotID   "nRobotID"
#define Attrib_IfValue     "bValue"
#define Attrib_IfComPort   "nComPort"
#define Attrib_IfEtcPort   "nEtcPort"
#define Attrib_IfCondition "sCondition"

#define Attrib_ElseCondition "sCondition"

#define Attrib_StringAddResult "sResult"
#define Attrib_StringAddParam1 "sParam1"
#define Attrib_StringAddParam2 "sParam2"

//---------------------Motion------------//
#define Attrib_MoveJAlias		"MoveJ"
#define Attrib_MoveLAlias		"MoveL"
#define Attrib_MovePAlias		"MoveP"
#define Attrib_MoveCAlias		"MoveC"
#define Attrib_MoveBAlias		"MoveB"
#define Attrib_MoveRelJAlias	"MoveRelJ"
#define Attrib_MoveRelLAlias	"MoveRelL"
#define Attrib_MoveZAlias		"MoveZ"
#define Attrib_MoveLCLAlias		"MoveLCL"

//#define Attrib_MotionAlias    "alias"
#define Attrib_MotionRobotID  "RobotID"
#define Attrib_MotionType     "MotionType" //0-MoveJ,1-MoveL...
#define Attrib_MoveCType	"MoveCType"
#define Attrib_MoveCNum		"MoveCNum"
#define Attrib_MoveCFixedPosure		"Attrib_MoveCFixedPosure"
#define Attrib_FromPointList  "FromPointList"
#define Attrib_PointName	  "PointName"

#define Attrib_SetRobotiqSpeed "Speed"
#define Attrib_SetRobotiqForce "Force"
#define Attrib_SetRobotiqPosition "Position"
#define Attrib_SetRobotiqAction "Action"

#define Attrib_StartPointX "StartPointX"
#define Attrib_StartPointY "StartPointY"
#define Attrib_StartPointZ "StartPointZ"
#define Attrib_StartPointA "StartPointA"
#define Attrib_StartPointB "StartPointB"
#define Attrib_StartPointC "StartPointC"

#define Attrib_EndPointX "EndPointX"
#define Attrib_EndPointY "EndPointY"
#define Attrib_EndPointZ "EndPointZ"
#define Attrib_EndPointA "EndPointA"
#define Attrib_EndPointB "EndPointB"
#define Attrib_EndPointC "EndPointC"

#define Attrib_PlanePointX "PlanePointX"
#define Attrib_PlanePointY "PlanePointY"
#define Attrib_PlanePointZ "PlanePointZ"
#define Attrib_PlanePointA "PlanePointA"
#define Attrib_PlanePointB "PlanePointB"
#define Attrib_PlanePointC "PlanePointC"

#define Attrib_MoveZEnableZDensity "EnableZDensity"
#define Attrib_MoveZEnablePlanePoint "EnablePlanePoint"
#define Attrib_MoveZEnableWaittime "EnableWaitime"
#define Attrib_MoveZWidth "ZWidth"
#define Attrib_MoveZDensity "ZDensity"
#define Attrib_MoveZSpeed "ZSpeed"
#define Attrib_MoveZXPositiveWaitTime "ZXPositiveWaitTime"
#define Attrib_MoveZXNegativeWaitTime "ZXNegativeWaitTime"

// #define Attrib_MoveGateP2Y "P2Y"
// #define Attrib_MoveGateP2Z "P2Z"
// #define Attrib_MoveGateP2RX "P2RX"
// #define Attrib_MoveGateP2RY "P2RY"
// #define Attrib_MoveGateP2RZ "P2RZ"
// 
// #define Attrib_MoveGateP3X "P3X"
// #define Attrib_MoveGateP3Y "P3Y"
// #define Attrib_MoveGateP3Z "P3Z"
// #define Attrib_MoveGateP3RX "P3RX"
// #define Attrib_MoveGateP3RY "P3RY"
// #define Attrib_MoveGateP3RZ "P3RZ"
// 
// #define Attrib_MoveGateR1 "R1"
// #define Attrib_MoveGateR2 "R2"
// #define Attrib_MoveGateInputType "InputType"
// #define Attrib_MoveGatePoint1 "Point_1"
// #define Attrib_MoveGatePoint2 "Point_2"
// #define Attrib_MoveGatePoint3 "Point_3"
// 
#define Attrib_ZWidth "ZWidth"
#define Attrib_ZDensity "ZDensity"
#define Attrib_UseZDensity "UseZDensity"
#define Attrib_ZBaseUserCoord "ZBaseUserCoord"

#define Attrib_PosType     "nPosType" 

//#define Attrib_PointType      "PointType"//选择点位的来源 ，0-从点位列表获取，1-由示教获得

#define Attrib_Var1          "Var1"
#define Attrib_Var2          "Var2"
#define Attrib_Var3          "Var3"
#define Attrib_Var4          "Var4"
#define Attrib_Var5          "Var5"
#define Attrib_Var6          "Var6"

#define BletPointY         		"BletPoint"
#define BletOffset         		"BletOffset"
#define IsBletPoint				"IsBletPoint"

//#define Attrib_PointName
#define Attrib_PosSource "PosSource"
#define Attrib_BletPos "sBletPos"
#define Attrib_Time "sTime"

#define Attrib_MoveCTargetXVarName "MoveCTargetXVarName"
#define Attrib_MoveCTargetYVarName "MoveCTargetYVarName"
#define Attrib_MoveCTargetZVarName "MoveCTargetZVarName"
#define Attrib_MoveCTargetRXVarName "MoveCTargetRXVarName"
#define Attrib_MoveCTargetRYVarName "MoveCTargetRYVarName"
#define Attrib_MoveCTargetRZVarName "MoveCTargetRZVarName"

#define Attrib_MoveCPassXVarName "MoveCPassXVarName"
#define Attrib_MoveCPassYVarName "MoveCPassYVarName"
#define Attrib_MoveCPassZVarName "MoveCPassZVarName"
#define Attrib_MoveCPassRXVarName "MoveCPassRXVarName"
#define Attrib_MoveCPassRYVarName "MoveCPassRYVarName"
#define Attrib_MoveCPassRZVarName "MoveCPassRZVarName"
#define Attrib_MoveLCLPos "MoveLCLPos"

#define Attrib_MoveRelAbsolute     "nAbsolute" //1-Absolute,0-Increase
#define Attrib_MoveRelDistance     "nDistance"

//-------------------------------------------------//
#define Attrib_WaitType	     "enType"
#define Attrib_WaitRobotID   "nRobotID"
#define Attrib_WaitValue	 "bValue"
#define Attrib_WaitComPort   "nComPort"
#define Attrib_WaitEtcPort   "nEtcPort"
#define Attrib_WaitlnTime    "ulnTime"
#define Attrib_WaitCondition "sCondition"
#define Attrib_InternalCoilIndex "InternalCoilIndex"
#define Attrib_ExpendMBName "ExpendMBName"

#define Attrib_SetIOType	  "enType"
#define Attrib_SetIORobotID   "nRobotID"
#define Attrib_SetIOValue	  "bValue"
#define Attrib_SetIOComPort   "nComPort"
#define Attrib_SetIOEtcPort   "nEtcPort"
#define Attrib_SetIOCondition "sCondition"
#define Attrib_SetIOAnalogIndex "AnalogIndex"
#define Attrib_SetIOAnalogValue "AnalogValue"
#define Attrib_SetIOAnalogPattern "AnalogPattern"
#define Attrib_SetIOAnalogValueType "AnalogValueType"
#define Attrib_SetIOAnalogVariantName "AnalogVariantName"
#define Attrib_SetIOFlashingtime "Flashingtime"

#define Attrib_SetRbtType      "enType"
#define Attrib_SetRbtCoordName  "sName"
#define Attrib_SetRbtRobotID   "nRobotID"

#define Attrib_CallName   "sName"
#define Attrib_CallPassPara   "sPassPara"
#define Attrib_CallType   "nType"
#define Attrib_ReturnValue "sValue"
#define Attrib_TimerInterValue "sValue"
#define Attrib_ParamTypeList "paramTypeList"


#define Attrib_CommunicateType "sCommunicateType"
#define Attrib_CommunicateIP "sIP"
#define Attrib_CommunicatePort "nPort"
#define Attrib_SocketName     "sSocketName"
#define Attrib_SendStringtype   "nStringtype"
#define Attrib_ReceiveStringtype   "nStringtype"
#define Attrib_SpiltValue   "sSpiltValue"
#define Attrib_SendValue       "sSendValue"
#define Attrib_SendDataType "nDataType"
#define Attrib_SendDataCount "nDataCnt"
#define Attrib_SendStartText "sStartText"
#define Attrib_SendEndText "sEndText"
#define Attrib_ReceiveValue  "sReceiveValue"

#define Attrib_SetValueDoType  "sDoType"
#define Attrib_SetValueSymbolType  "sSymbolType"
#define Attrib_SetValueVarType  "sVarType"
#define Attrib_SetValueFactorString1  "sFactorString1"
#define Attrib_SetValueFactorString2  "sFactorString2"

#define Attrib_SetOverride  "sSetOverride"
#define Atttib_SetPayload "dSetPayload"
#define Atttib_SetPayloadRX "dSetPayloadRx"
#define Atttib_SetPayloadRY "dSetPayloadRy"
#define Atttib_SetPayloadRZ "dSetPayloadRz"
#define Attrib_OverrideRobotID  "RobotID"
#define Attrib_SetFollow "bSetFollow"
#define Attrib_InputType "InputType"
#define Attrib_SpeedVarName "SpeedVarName"
#define Attrib_PayloadVarName "PayloadVarName"
#define Attrib_CXVarName "CXVarName"
#define Attrib_CYVarName "CYVarName"
#define Attrib_CZVarName "CZVarName"
#define Attrib_FTCHECKED 	"FTChecked"
#define Attrib_FTVALUES 	"FTValues"
#define Attrib_FTGAINS 		"gains"
#define Attrib_FTCOORDNAME  "FTCoordName"
#define Attrib_FTSTART 		"FTSart"
#define Attrib_REDUCE "bSetReduce"

#define Attrib_ModAddr  "nAddress"
#define Attrib_ModLength "nLength"
#define Attrib_ModSlave  "nSlave"
#define Attrib_ModDataType  "nDataType"
#define Attrib_ModData  "ModData_"//数据个数MAX_MODBUSDATALENGTH，表示为ModData_0，ModData_1...

#define Attrib_DragMoveTeachMode "TeachMode"
#define Attrib_DragMoveMoveMode "MoveMode"
#define Attrib_DragMovePointCnt "PointCnt"
#define Attrib_DragMovePointCntPcs "PointCnt_FilterPcs"
#define Attrib_DragMoveFilterFactor "FilterFactor"
#define Attrib_Tolerance		"Tolerance"
#define Attrib_Distance			"Distance"
#define Attrib_DragMoveToleranceAcsPointCount "ToleranceAcsPointCount"
#define Attrib_DragMoveArrayAcs "PointAcs"
#define Attrib_DragMoveArrayPcs "PointPcs"
#define Attrib_DragMoveArrayFilterPcs "FilterPointPcs"
#define Attrib_DragMoveArrayFilterAcs "FilterPointAcs"
#define Attrib_DragMoveInterval "Interval"
#define Attrib_DragMoveVersion "Version"
#define Attrib_DragMoveListTrackName "Name"
#define Attrib_DragMoveTrackName "TrackName"
#define Attrib_DragMoveTrackType "TrackType"
#define Arrtib_DragMoveIOCnt "IOCnt"
#define Arrtib_DragMoveIOIndex "IOIndex"
#define Attrib_DragMoveIOState "IOState"
#define Attrib_DragMoveTimeElapse "PointInterval"
#define Attrib_DragMoveIsStop	"WaitFlag"

#define Attrib_BlendingTrajectoryCnt			"TrajectoryCnt"
#define Attrib_BlendingMoveMode					"MoveMode"
#define Attrib_BlendingRadius					"Radius"
#define Attrib_BlendingTeach					"Teach"
#define Attrib_BlendingFromPointList			"FromPointList"
#define Attrib_BlendingVarList					"VarList"
#define Attrib_BlendingTypeList					"TypeList"
#define Attrib_BlendingPointValList				"PointValList"
#define Attrib_BlendingPointName				"PointName"
#define Attrib_BlendingMoveCType				"MoveCType"
#define Attrib_BlendingMoveCNum					"MoveCNum"
#define Attrib_BlendingAttrib_MoveCFixedPosure	"MoveCFixedPosure"
#define Attrib_BlendingMoveCTeach				"MoveCTeach"
#define Attrib_BlendingTargetPointValList		"TargetPointValList"
#define Attrib_BlendingPassPointValList			"PassPointValList"
#define Attrib_BlendingTargetPointVarList		"TargetPointVarList"
#define Attrib_BlendingPassPointVarList			"PassPointVarList"
#define Attrib_BlendingAOSlot					"Slot"
#define Attrib_BlendingAOValue					"Value"
#define Attrib_BlendingAOMode					"Mode"
#define Attrib_BlendingAOVarList				"VarList"

#define Attrib_WayMoveType						"nMoveType"
#define Attrib_WaySpeed						"nSpeed"
#define Attrib_WayASpeed						"nASpeed"
#define Attrib_WayTCPName						"sTcpName"
#define Attrib_WayUCSName						"sFeatueName"
#define Attrib_WayUseJoint						"nUseJoint"
#define Attrib_WayShowName						"ShowName"

#define Attrib_PatterWhenStart					"PatterWhenStart"
#define Attrib_PatterWhenafter					"PatterWhenafter"
#define Attrib_PatternType						"PatternType"

#define Attrib_PointOneTwocnt					"PointOneTwocnt"
#define Attrib_PointOneThreecnt					"PointOneThreecnt"
#define Attrib_PointOneFivecnt					"PointOneFivecnt"

#define Attrib_AnchorPointName					"AnchorPointName"

//////////////////////////////////////////////////////////////////////////
//比较符号
//////////////////////////////////////////////////////////////////////////
enum EN_CmpSymbolType
{
	enCmpSymbolType_Equal = 0,
	enCmpSymbolType_NotEqual,
	enCmpSymbolType_GreatrerThan,
	enCmpSymbolType_LessThan,
	enCmpSymbolType_GreatrerThanEqual,
	enCmpSymbolType_LessThanEqual,
	enCmpSymbolType_cnt
};

enum EN_CmpSetValueType
{
	enCmpSymbolType_Set = 0,
	enCmpSymbolType_Add,
	enCmpSymbolType_Sub,
	enCmpSymbolType_Muli,
	enCmpSymbolType_divi,

};


#define CmpSb_Equal					 "=="     
#define CmpSb_NotEqual				 "!="     
#define CmpSb_GreatrerThan   "GreaterThan"//  ">"  //用单词代替符号是因为"<",">"在xml文件显示为乱码
#define CmpSb_LessThan       "LessThan" // "<"
#define CmpSb_GreatrerThanEqual      ">="
#define CmpSb_LessThanEqual          "<="
 



//////////////////////////////////////////////////////////////////////////
//Modebus连接类型
//////////////////////////////////////////////////////////////////////////
enum ENModebusType
{
	enModebusType_Null = 0,
	enModebusType_RTU,
	enModebusType_TCP
};
//////////////////////////////////////////////////////////////////////////
//分隔符符号
//逗号,，空格，冒号:，斜杠/，竖杠|，下划线_，分号；
//////////////////////////////////////////////////////////////////////////
#define Spilt_ConstHead          "Const_"

#define Spilt_ConstMark         "Const"
#define Spilt_VarMark           "Var"
#define Spilt_InputMark         "Input"



#define SpiltSmybol_Douhao        "<,>"
#define SpiltSmybol_Kongge        "< >"
#define SpiltSmybol_Maohao        "<:>"
#define SpiltSmybol_Xiegang       "</>"
#define SpiltSmybol_Shugang       "<|>"
//#define SpiltSmybol_Xiahuaxian    "<_>"
#define SpiltSmybol_Fenhao        "<;>"



/************************************************************************/
/*Page 9                                                                */
/************************************************************************/
#ifndef _RBERRORCODEDEFINE_H_
#define _RBERRORCODEDEFINE_H_

namespace NS_RbErrCodeDef
{
/***************************************************************
**控制器系统状态
***************************************************************/
	/* Protocol exceptions */
	enum {
		MBError_ILLEGAL_FUNCTION = 3001,
		MBError_ILLEGAL_DATA_ADDRESS,
		MBError_ILLEGAL_DATA_VALUE,
		MBError_SLAVE_OR_SERVER_FAILURE,
		MBError_ACKNOWLEDGE,
		MBError_SLAVE_OR_SERVER_BUSY,
		MBError_NEGATIVE_ACKNOWLEDGE,
		MBError_MEMORY_PARITY,
		MBError_NOT_DEFINED,
		MBError_GATEWAY_PATH,
		MBError_GATEWAY_TARGET,
		MBError_MAX
	};

enum EN_SystemState
{
	enSysState_Disabled = 1,
	enSysState_Standby,
	enSysState_Motion,
	enSysState_Stopping,
	enSysState_Disabling,
	enSysState_Enabling,
	enSysState_Teaching,
	enSysState_ErrorStop,
	enSysState_Error,
	enSysState_OutSafeSpace,
	enSysState_ProtectiveStop,
	enSysState_Braking,
	enSysState_HomeStep2ing,
	enSysState_Realying,
	enSysState_InitSlaves,
	enSysState_InitSlaves_Second,
	enSysState_InitResetSlaves,
	enSysState_EnergidSDKError,
	enSysState_Teach,
	enSysState_Feedforwarding,
	enSysState_Feedforward,
	enSysState_Simulation,
	enSysState_PreSimulation,
	enSysState_LeaveSimulation,
	enSysState_SelfCollisionStopping,
	enSysState_AutoPopping,
	enSysState_JudgeHomeStill,
	enSysState_HMDriveOperation,
	enSysState_ForceSensor,
	enSysState_ServoJ,
	enSysState_WaitKPASaveInit,
	enSysState_ForceControl,
	enSysState_Count,
};

/*************************************************************************
**大族机器人返回错误码定义
**REC_Successed=正常
**REC_RTOSNotInit=控制器未启动
**REC_MasterNotStarted=主站未启动
**REC_SlaveDropped=机械原点未设置
**REC_SafeLock=从站掉线
**REC_EmergencyStop=物理急停
**REC_UnPower=机器人未使能
**REC_SlaveError=从站报错
**REC_OutSafeSpace=机器人超出安全空间
**REC_OnMoving=机器人运动中
**REC_CmdInvalid=命令无效
**REC_ParamError=参数错误
**REC_FormatError=格式错误
**REC_WaitCmdExecute=等待命令执行
**REC_IONotExist=IO不存在
**REC_RobotNotExist=机器人不存在
**REC_NetworkTimeout=网络超时
**REC_ConnectedFailed=连接失败
**REC_CtrlerNotStarted=控制器未启动
**REC_HomePosNotSet=机械原点未设置
**REC_LastCmdNotFinished=上一个同样的命令未完成
**REC_SocketInValid=Socket无效，可能网络未连接
**REC_WaitTimeout=控制器处理命令超时
*************************************************************************/
	enum ReturnDCSStateCode
	{
		REC_Successed = 0,
		REC_RTOSNotInit = 1001,
		
		REC_MasterNotStarted,
		REC_SlaveDropped,
		REC_SafeLock,
		REC_EmergencyStop,
		REC_UnServoOn,
		REC_SlaveError,
		REC_OutSafeSpace,
		REC_OnMoving,
		REC_CmdInvalid,
		REC_ParamError,
		REC_FormatError,
		REC_WaitCmdExecute,
		REC_IONotExist,
		REC_RobotNotExist,
		REC_SocketInValid,
		REC_NetworkTimeout,
		REC_ConnectedFailed,
		REC_SerialFailed,
		REC_HomePosNotSet,
		REC_LastCmdNotFinished,
		REC_SerialDISizeIsZero,
		REC_SerialDOSizeIsZero,
		REC_WaitTimeout,
		REC_OnFault,
		REC_OnStopping,
		REC_OnDisabling,
		REC_OnEnabling,
		REC_OptionNotEnabled,
		REC_StartMasterTimeout,
		REC_RobotNotElectrify,
		REC_SerialNotEnabled,
		REC_SimulationStateCmdInvalid,
		REC_RTOSLibNotFound,
		REC_DCSThreadCrash,
		REC_OptionForbidden,
		REC_EmergencyStopStatuStill,
		REC_SafeStopStatuStill,
		REC_RunScript,
		REC_XmlFileError,
		REC_SystemBoardNotConnect,
		REC_ControllerNotStart,
		REC_ControllerStatuError,
		REC_RobotInTeachMode,
		REC_RobotAlreadyElectrify,
		REC_ConnectModbusTCPFailed,
		REC_MasterStarted,
		REC_OverRobotPayload,
		REC_DCSStatusError,
		REC_OutOfJointLimit,
		REC_HMDriveOperating,
		REC_MasterStartErr,
		REC_InitSlaveError,
		REC_ModebusRTUDisConnect,
		REC_ModebusRTUBusy,
		REC_BlendingNotStart,
		REC_BlendingNotEnd,
		REC_StackIsEnd, //码垛结束
		REC_StackReStart,//码垛重新开始
		REC_EmergencyHandling,
		REC_StartPosError,
		REC_ModbusSignalError,
		REC_SeekMustTurnToStackOrDeStack,
		REC_PointListError,
		REC_RobotModeFileExist,
		REC_PalletPatternError,
		REC_EndIODisconnect,
		REC_ForceSensorDisconneted,
		REC_GetInverseErr,
		/* 以下为调用脚本函数返回错误 */
		REC_LibLoadFail = 2000,				// Library 加载失败
		REC_ScriptEmpty,					// 脚本为空
		REC_CompileError,					// 编译错误
		REC_ReloadScriptError,				// ReloadScriptError
		REC_FuncNotExist,					// 函数不存在
		REC_ReturnTypeError,				// 函数返回类型错误
		REC_MissSignal1,					// Miss Signal
		REC_MissSignal2,					// Miss Signal
		REC_ParamTypeError,					// 参数类型错误
		REC_UnIncludeHeadFile,				// 没有包含头文件
		REC_ReturnError = 2010,					// 没有返回值
		REC_ParamCountError,				// 参数类型错误
		REC_UDMRegStackErr,
		REC_LockUDM,
		REC_NotInRunScriptStatus,
		REC_SerialNotConnect,
		REC_ControllerNotStarted,
		REC_SocketNetConnected,
		REC_FuncNamehaveSpace,					// 函数名称不能有空格
		REC_WaitRecvice,
		REC_BrokenStop = 2020,
		REC_SocketError,
		REC_ScriptPauseStatus,
		REC_TimerRunningError,
		REC_EnableSwitchONErr,
		REC_ParamNotExist,						// 参数不存在
		REC_ArrayIndexBound,					// 数组下标越界
		REC_ArrayIndexNotExist,					// 数组下标不存在

		REC_Robotiq_NotConfig,
		REC_Robotiq_Err,
		REC_Robotiq_Err05,
		REC_Robotiq_Err07,
		REC_Robotiq_Err08,
		REC_Robotiq_Err0A,
		REC_Robotiq_Err0B,
		REC_Robotiq_Err0C,
		REC_Robotiq_Err0D,
		REC_Robotiq_Err0E,
		REC_Robotiq_Err0F,
	

		REC_MODBUS_EMBXILFUN = MBError_ILLEGAL_FUNCTION,
		REC_MODBUS_EMBXILADD,
		REC_MODBUS_EMBXILVAL,
		REC_MODBUS_EMBXSFAIL,
		REC_MODBUS_EMBXACK,
		REC_MODBUS_EMBXSBUSY,
		REC_MODBUS_EMBXNACK,
		REC_MODBUS_EMBXMEMPAR,
		REC_MODBUS_EMBXGPATH,
		REC_MODBUS_EMBXGTAR/* = 112345678 + 11*/,
		REC_MODBUS_EMBBADCRC,
		REC_MODBUS_EMBBADDATA,
		REC_MODBUS_EMBBADEXC,
		REC_MODBUS_EMBUNKEXC,
		REC_MODBUS_EMBMDATA,
		REC_MODBUS_EMBBADSLAVE,
		
	};

/*************************************************************************
**控制器错误码定义
**CEC_Normal=正常
**CEC_ShortCircuitError=短路错误
**CEC_OverVoltageError=过压错误
**CEC_UnderVoltageError=欠压错误
**CEC_VelocityOverError=速度过超错误
**CEC_ExecuteError=执行时错误
**CEC_OverCurrentError=RMS过流错误
**CEC_EncoderError=编码器错误
**CEC_PositionFollowError=位置跟随错误错误
**CEC_VelocityFollowError=速度跟随错误
**CEC_NegativeLimitError=负限位错误
**CEC_PositiveLimitError=正限位错误
**CEC_ServerOverHeating=伺服过温错误
**CEC_MaxCurrentError=峰值电流错误
**CEC_ScramError=硬件刹车错误
**CEC_UDMError=UDM错误
**CEC_ServerParameterError=伺服参数错误
**CEC_CtrlerNotStart=控制器未启动
**CEC_MasterNotStart=主站未启动
**CEC_HomePosNotSet=机械原点未设置
**CEC_SlaveDropped=从站掉线
**CEC_SafeLock=控制器处于安全光幕状态
**CEC_EmergencyStop=控制器处于物理急停状态
*************************************************************************/
	enum EN_CtrlerErrCode
	{
		CEC_Normal = 0,
		CEC_ShortCircuitError = 10000,
		CEC_OverVoltageError,
		CEC_UnderVoltageError,
		CEC_VelocityOverError,
		CEC_ExecuteError,
		CEC_RMSOverCurrentError,
		CEC_EncoderError,
		CEC_PositionFollowError,
		CEC_VelocityFollowError,
		CEC_NegativeLimitError,
		CEC_PositiveLimitError,
		CEC_ServerOverHeating,
		CEC_MaxCurrentError,
		CEC_EmergencyStopError,
		CEC_UDMError,
		CEC_ServerParameterError,
		
		CEC_CtrlerNotStart = 20000,
		CEC_MasterNotStart,
		CEC_SlaveDropped,
		CEC_SafeStop,
		CEC_PhysicalStop,
		CEC_OutSafeSpace,
		CEC_EnableTimeOut,
		CEC_RobotNotElectrify,
		CEC_MasterStartErr,

		CEC_CollisionShutdown = 30000,
		CEC_CollisionSelfError,
		CEC_JointLimitError,
		CEC_SingularityError,
		CEC_GeneralStoppingCriterion,
		CEC_CalculateError,
		CEC_UDMStatuError,
		CEC_InitSlaveError,
		CEC_HomeStep2Error,
		CEC_OutOfLimitError,
		CEC_OutOfAxisCurrent,
		CEC_PayloadOrMountingParamError,
		CEC_OverMotorTemperatureLimit,
		CEC_MasterInitError,
		CEC_LoadLicenseError,
		CEC_SlaveCntError,
		CEC_ENIFileError,
		CEC_BlendingCalculateError,
		CEC_AxisPositionFollowError,
		CEC_MotionMonitorUnknown,
		CEC_MotionMonitorNeedClearError,
		CEC_MotionMonitorOverJointVelocityError,
		CEC_MotionMonitorOverLinearVelocityError,
		CEC_MotionMonitorOverMomentumLimit,
		CEC_MotionMonitorOverPowerLimit,
		CtrlerErrCodeCount,		// 用于统计控制器错误码数量，放在最后.
	};


/*************************************************************************
**SCARA错误码定义
*************************************************************************/
	enum EN_ScaraErrCode
	{
		SEC_Normal = 0,
		SEC_ShortCircuitError = 1 << 0,
		SEC_UnderVoltageError = 1 << 1,
		SEC_OverVoltageError = 1 << 2,
		SEC_VelocityOverError = 1 << 3,
		SEC_ExecuteError = 1 << 4,
		SEC_RMSOverCurrentError = 1 << 5,
		SEC_EncoderError = 1 << 6,
		SEC_PositionFollowError = 1 << 7,

		SEC_VelocityFollowError = 1 << 8,
		SEC_NegativeLimitError = 1 << 9,
		SEC_PositiveLimitError = 1 << 10,
		SEC_ServerOverHeating = 1 << 11,
		SEC_MaxCurrentError = 1 << 12,
		SEC_EmergencyStopError = 1 << 13,
		SEC_UDMError = 1 << 14,
		SEC_ServerParameterError = 1 << 15,
	};


/*************************************************************************
**Elfin错误码定义
*************************************************************************/
	enum EN_ElfinErrCode
	{
		EEC_Normal = 0,
		EEC_VelocityFollowError = 1 << 0,
		EEC_NegativeLimitError = 1 << 1,
		EEC_PositiveLimitError = 1 << 2,
		EEC_ServerOverHeating = 1 << 3,
		EEC_MaxCurrentError = 1 << 4,
		EEC_EmergencyStopError = 1 << 5,
		EEC_UDMError = 1 << 6,
		EEC_ServerParameterError = 1 << 7,

		EEC_ShortCircuitError = 1 << 8,
		EEC_UnderVoltageError = 1 << 9,
		EEC_OverVoltageError = 1 << 10,
		EEC_VelocityOverError = 1 << 11,
		EEC_ExecuteError = 1 << 12,
		EEC_RMSOverCurrentError = 1 << 13,
		EEC_EncoderError = 1 << 14,
		EEC_PositionFollowError = 1 << 15,
	};
};
#endif
