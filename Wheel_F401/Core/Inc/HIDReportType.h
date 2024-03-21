/*
 * HIDReportType.h
 *
 *  Created on: Sep 24, 2023
 *      Author: Vielder
 */

#ifndef INC_HIDREPORTTYPE_H_
#define INC_HIDREPORTTYPE_H_

#include <sys/_stdint.h>

// Maximum number of parallel effects in memory
#define MAX_EFFECTS     20
#define SIZE_EFFECT     sizeof(TEffectState)
#define MEMORY_SIZE     (uint16_t)(MAX_EFFECTS*SIZE_EFFECT)
#define TO_LT_END_16(x)    ((x<<8)&0xFF00)|((x>>8)&0x00FF)

#define FFB_ID_OFFSET 0x00

// HID Descriptor definitions - FFB Report IDs
#define HID_ID_STATE	0x02	// Usage PID State report

#define HID_ID_EFFREP	0x01	// Usage Set Effect Report
#define HID_ID_ENVREP	0x02	// Usage Set Envelope Report
#define HID_ID_CONDREP	0x03	// Usage Set Condition Report
#define HID_ID_PRIDREP	0x04	// Usage Set Periodic Report
#define HID_ID_CONSTREP	0x05	// Usage Set Constant Force Report
#define HID_ID_RAMPREP	0x06	// Usage Set Ramp Force Report
#define HID_ID_CSTMREP	0x07	// Usage Custom Force Data Report
#define HID_ID_SMPLREP	0x08	// Usage Download Force Sample
#define HID_ID_EFOPREP	0x0A	// Usage Effect Operation Report
#define HID_ID_BLKFRREP	0x0B	// Usage PID Block Free Report
#define HID_ID_CTRLREP	0x0C	// Usage PID Device Control
#define HID_ID_GAINREP	0x0D	// Usage Device Gain Report
#define HID_ID_SETCREP	0x0E	// Usage Set Custom Force Report
// Features
#define HID_ID_NEWEFREP	0x11	// Usage Create New Effect Report
#define HID_ID_BLKLDREP	0x12	// Usage Block Load Report
#define HID_ID_POOLREP	0x13	// Usage PID Pool Report

#define HID_ACTUATOR_POWER 		0x08
#define HID_SAFETY_SWITCH 		0x04
#define HID_ENABLE_ACTUATORS 	0x02
#define HID_EFFECT_PAUSE		0x01
#define HID_ENABLE_ACTUATORS_MASK 0xFD
#define HID_EFFECT_PLAYING 		0x10

// ---- effect

#define USB_DURATION_INFINITE		0x7FFF

#define FFB_EFFECT_NONE					0x00
#define FFB_EFFECT_CONSTANT	  	0x01
#define FFB_EFFECT_RAMP					0x02
#define FFB_EFFECT_SQUARE 			0x03
#define FFB_EFFECT_SINE 				0x04
#define FFB_EFFECT_TRIANGLE			0x05
#define FFB_EFFECT_SAWTOOTHDOWN	0x06
#define FFB_EFFECT_SAWTOOTHUP		0x07
#define FFB_EFFECT_SPRING				0x08
#define FFB_EFFECT_DAMPER				0x09
#define FFB_EFFECT_INERTIA			0x0A
#define FFB_EFFECT_FRICTION			0x0B
#define FFB_EFFECT_CUSTOM				0x0C

// Bit-masks for effect states
#define MEFFECTSTATE_FREE				0x00
#define MEFFECTSTATE_ALLOCATED	0x01
#define MEFFECTSTATE_PLAYING		0x02

#define X_AXIS_ENABLE           0x01
#define Y_AXIS_ENABLE        		0x02
#define DIRECTION_ENABLE        0x04

//these were needed for testing
#define INERTIA_FORCE 					0xFF
#define FRICTION_FORCE					0xFF
#define INERTIA_DEADBAND				0x30
#define FRICTION_DEADBAND				0x30

#include <stdint.h>

// ---- Input
typedef struct { //WheelReport
	uint8_t buttons;
	int16_t xAxis;
	int16_t yAxis;
	int16_t zAxis;
	int16_t rxAxis;
	int16_t ryAxis;
	int16_t rzAxis;
} USB_FFBReport_WheelReport_Input_Data_t;

#ifdef __cplusplus

// ---- Output

typedef struct { // FFB: Set Effect Output Report
	uint8_t reportId = 1;	// =1
	uint8_t effectBlockIndex;	// 1..40
	uint8_t effectType;	// 1..12 (effect usages: 26,27,30,31,32,33,34,40,41,42,43,28)
	uint16_t duration; // 0..32767 ms
	uint16_t triggerRepeatInterval; // 0..32767 ms
	uint16_t samplePeriod;	// 0..32767 ms
	uint8_t gain;	// 0..255	 (physical 0..10000)
	uint8_t triggerButton;	// button ID (0..8)
	uint8_t enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
	uint8_t directionX;	// angle (0=0 .. 255=360deg)
	uint8_t directionY;	// angle (0=0 .. 255=360deg)
	//	uint16_t	startDelay;	// 0..32767 ms
} __attribute__((packed)) USB_FFBReport_SetEffect_Output_Data_t;

typedef struct { // FFB: Set Envelope Output Report
	uint8_t reportId = 2;	// =2
	uint8_t effectBlockIndex;	// 1..40
	uint16_t attackLevel;
	uint16_t fadeLevel;
	uint16_t attackTime;	// ms
	uint16_t fadeTime;	// ms
} __attribute__((packed)) USB_FFBReport_SetEnvelope_Output_Data_t;

typedef struct { // FFB: Set Condition Output Report
	uint8_t reportId = 3;	// =3
	uint8_t effectBlockIndex;	// 1..40
	uint8_t parameterBlockOffset;	// bits: 0..3=parameterBlockOffset, 4..5=instance1, 6..7=instance2
	int16_t cpOffset;	// 0..255
	int16_t positiveCoefficient;	// -128..127
	int16_t negativeCoefficient;	// -128..127
	uint16_t positiveSaturation;	// -	128..127
	uint16_t negativeSaturation;	// -128..127
	uint16_t deadBand;	// 0..255
} __attribute__((packed)) USB_FFBReport_SetCondition_Output_Data_t;

typedef struct { // FFB: Set Periodic Output Report
	uint8_t reportId = 4;	// =4
	uint8_t effectBlockIndex;	// 1..40
	uint16_t magnitude;
	int16_t offset;
	uint16_t phase;	// 0..255 (=0..359, exp-2)
	uint16_t period;	// 0..32767 ms
} __attribute__((packed)) USB_FFBReport_SetPeriodic_Output_Data_t;

typedef struct { // FFB: Set ConstantForce Output Report
	uint8_t reportId = 5;	// =5
	uint8_t effectBlockIndex;	// 1..40
	int16_t magnitude;	// -255..255
} __attribute__((packed)) USB_FFBReport_SetConstantForce_Output_Data_t;

typedef struct { // FFB: Set RampForce Output Report
	uint8_t reportId = 6;	// =6
	uint8_t effectBlockIndex;	// 1..40
	int16_t startMagnitude;
	int16_t endMagnitude;
} __attribute__((packed)) USB_FFBReport_SetRampForce_Output_Data_t;

typedef struct { // FFB: Set CustomForceData Output Report
	uint8_t reportId = 7;	// =7
	uint8_t effectBlockIndex;	// 1..40
	uint16_t dataOffset;
	int8_t data[12];
} __attribute__((packed)) USB_FFBReport_SetCustomForceData_Output_Data_t;

typedef struct { // FFB: Set DownloadForceSample Output Report
	uint8_t reportId = 8;	// =8
	int8_t x;
	int8_t y;
} __attribute__((packed)) USB_FFBReport_SetDownloadForceSample_Output_Data_t;

typedef struct { // FFB: Set EffectOperation Output Report
	uint8_t reportId = 10;
	uint8_t effectBlockIndex;	// 1..40
	uint8_t operation; // 1=Start, 2=StartSolo, 3=Stop
	uint8_t loopCount;
} __attribute__((packed)) USB_FFBReport_EffectOperation_Output_Data_t;

typedef struct { // FFB: Block Free Output Report
  uint8_t	reportId;	// =11
	uint8_t effectBlockIndex;	// 1..40
} __attribute__((packed)) USB_FFBReport_BlockFree_Output_Data_t;

typedef struct { // FFB: Device Control Output Report
	uint8_t reportId = 12;
	uint8_t control; // 1=Enable Actuators, 2=Disable Actuators, 4=Stop All Effects, 8=Reset, 16=Pause, 32=Continue
} __attribute__((packed)) USB_FFBReport_DeviceControl_Output_Data_t;

typedef struct { // FFB: Set Custom Force Output Report
  uint8_t		reportId;	// =14
	uint8_t effectBlockIndex;	// 1..40
	uint8_t sampleCount;
	uint16_t samplePeriod;	// 0..32767 ms
} USB_FFBReport_SetCustomForce_Output_Data_t;

// ---- Features

typedef struct { // FFB: PID Pool Feature Report
	uint8_t reportId;	// =3
	uint16_t ramPoolSize;	// ?
	uint8_t maxSimultaneousEffects;	// ?? 40?
	uint8_t memoryManagement;	// Bits: 0=DeviceManagedPool, 1=SharedParameterBlocks
} USB_FFBReport_PIDPool_Feature_Data_t;

typedef struct { // FFB: PID Block Load Feature Report
	uint8_t reportId;	// =2
	uint8_t effectBlockIndex;	// 1..40
	uint8_t loadStatus;	// 1=Success,2=Full,3=Error
	uint16_t ramPoolAvailable;	// =0 or 0xFFFF?
} USB_FFBReport_PIDBlockLoad_Feature_Data_t;

typedef struct {
	uint8_t reportId = 1;
	uint8_t effectBlockIndex = 0;	// 1..max_effects
	uint8_t effectType = 0;
	uint16_t duration = 0; // 0..32767 ms
	uint16_t triggerRepeatInterval = 0; // 0..32767 ms
	uint16_t samplePeriod = 0;	// 0..32767 ms
	uint8_t gain = 255;	// 0..255 scaler
	uint8_t triggerButton = 0;	// button ID. unused
	uint8_t enableAxis = 0; // bits: 0=X, 1=Y, 2=DirectionEnable
	uint8_t directionX = 0;	// angle (0=0 .. 255=360deg)
	uint8_t directionY = 0;	// angle (0=0 .. 255=360deg)
//	uint16_t	typeSpecificBlockOffsetX = 0; // Needed?
//	uint16_t	typeSpecificBlockOffsetY = 0;
//	uint16_t	startDelay;	// 0..32767 ms
} __attribute__((packed)) FFB_SetEffect_t;

typedef struct {
	uint8_t reportId;
	uint8_t effectBlockIndex;	// 1..max_effects
	uint8_t parameterBlockOffset;	// bits: 0..3=parameterBlockOffset, 4..5=instance1, 6..7=instance2
	int16_t cpOffset;	// Center
	int16_t positiveCoefficient; // Scaler for positive range
	int16_t negativeCoefficient;
	uint16_t positiveSaturation;	// Clipping point for positive range
	uint16_t negativeSaturation;
	uint16_t deadBand;
} __attribute__((packed)) FFB_SetCondition_Data_t;

typedef struct {
	uint8_t reportId = HID_ID_BLKLDREP;
	uint8_t effectBlockIndex;	// 1..max_effects
	uint8_t loadStatus;	// 1=Success,2=Full,3=Error
	uint16_t ramPoolAvailable;
} __attribute__((packed)) FFB_BlockLoad_Feature_Data_t;

typedef struct {
	uint8_t reportId = HID_ID_POOLREP;
	uint16_t ramPoolSize = MAX_EFFECTS;
	uint8_t maxSimultaneousEffects = MAX_EFFECTS;
	uint8_t memoryManagement = 1;	// 0=DeviceManagedPool, 1=SharedParameterBlocks
} __attribute__((packed)) FFB_PIDPool_Feature_Data_t;

typedef struct {
	uint8_t reportId;
	uint8_t effectBlockIndex;
	uint16_t magnitude;
	int16_t offset;
	uint16_t phase;	// degrees
	uint16_t period;	// 0..32767 ms
} __attribute__((packed)) FFB_SetPeriodic_Data_t;

typedef struct {
	uint8_t reportId;
	uint8_t effectBlockIndex;	// 1..max_effects
	int16_t magnitude;	// High res intensity
} __attribute__((packed)) FFB_SetConstantForce_Data_t;

typedef struct { // FFB: DeviceGain Output Report
	uint8_t reportId = 13;	// =13
	uint8_t gain;
} USB_FFBReport_DeviceGain_Output_Data_Map_t;

// Internal struct for storing effects
typedef struct {
	volatile uint8_t state = 0;
	uint8_t type = FFB_EFFECT_NONE; // Type
	uint8_t gain = 255;	// Scaler. often unused
	int16_t positiveCoefficient = 0;
	int16_t negativeCoefficient = 0;
	uint16_t positiveSaturation = 0;
	uint16_t negativeSaturation = 0;
	int16_t magnitude = 0;	// High res intensity of effect
	int16_t phase = 0;
	int16_t offset = 0;	// Center point
	int32_t last_value = 0;
	uint16_t counter = 0;	// Elapsed time in ms
	uint16_t period = 0;
	uint16_t duration = 0, fadeTime = 0, attackTime = 0;	// Duration in ms
	uint16_t samplePeriod = 0;
	uint8_t axis = 0;	// Active axis
	uint16_t deadBand = 0;
} FFB_Effect;

typedef struct {
	volatile uint8_t state = 0;  // see constants <MEffectState_*>
	uint8_t effectType = FFB_EFFECT_NONE; //
	int16_t offset = 0;
	uint8_t gain = 0;
	int16_t attackLevel = 0, fadeLevel = 0;
	int16_t magnitude = 0;
	uint8_t enableAxis = 0; // bits: 0=X, 1=Y, 2=DirectionEnable
	uint8_t directionX = 0; // angle (0=0 .. 255=360deg)
	uint8_t directionY = 0; // angle (0=0 .. 255=360deg)
	int16_t cpOffset = 0; // -128..127
	int16_t positiveCoefficient = 0; // -128..127
	int16_t negativeCoefficient = 0; // -128..127
	uint16_t positiveSaturation = 0;  // -128..127
	uint16_t negativeSaturation = 0;  // -128..127
	uint16_t deadBand = 0;  // 0..255
	int32_t last_value = 0;
	uint16_t phase = 0;  // 0..255 (=0..359, exp-2)
	int16_t startMagnitude = 0;
	int16_t endMagnitude = 0;
	uint8_t axis = 0;	// Active axis
	uint16_t period = 0; // 0..32767 ms
	uint16_t duration = 0, fadeTime = 0, attackTime = 0, counter = 0;
	uint64_t startTime = 0;
} TEffectState;

typedef struct { //PID State
	uint8_t reportId = HID_ID_STATE;
	uint8_t status;	// Bits: 0=Device Paused,1=Actuators Enabled,2=Safety Switch,3=Actuator Override Switch,4=Actuator Power
	uint8_t effectBlockIndex;	// Bit7=Effect Playing, Bit0..7=EffectId (1..40)
} USB_FFBReport_PIDStatus_Input_Data_t;

#endif //c++

typedef struct {
	volatile uint8_t state;  // see constants <MEffectState_*>
	uint8_t effectType; //
	int16_t offset;
	uint8_t gain;
	int16_t attackLevel, fadeLevel;
	int16_t magnitude;
	uint8_t enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
	uint8_t directionX; // angle (0=0 .. 255=360deg)
	uint8_t directionY; // angle (0=0 .. 255=360deg)
	int16_t cpOffset; // -128..127
	int16_t positiveCoefficient; // -128..127
	int16_t negativeCoefficient; // -128..127
	uint16_t positiveSaturation;  // -128..127
	uint16_t negativeSaturation;  // -128..127
	uint16_t deadBand;  // 0..255
	int32_t last_value;
	uint16_t phase;  // 0..255 (=0..359, exp-2)
	int16_t startMagnitude;
	int16_t endMagnitude;
	uint8_t axis;	// Active axis
	uint16_t period; // 0..32767 ms
	uint16_t duration, fadeTime, attackTime, counter;
	uint64_t startTime;
} TEffectState_c;

typedef struct {
	uint8_t reportId;
	uint8_t effectType;	// Effect type ID
	uint16_t byteCount;	// Size of custom effects
} FFB_CreateNewEffect_Feature_Data_t;

#endif /* INC_HIDREPORTTYPE_H_ */
