/*
 * FfbReportHandler.cpp
 *
 *  Created on: Sep 24, 2023
 *      Author: Vielder
 */

#define NOMINMAX
#include "FfbReportHandler.h"
#include "FfbWheel.h"
#include "math.h"
#include "helpers.h"
#include "usbd_customhid.h"


FfbReportHandler::FfbReportHandler() {
	nextEID = 1;
	devicePaused = 0;
	usedMemory = 0;
	FreeAllEffects();
	memset((void*) &pidBlockLoad, 0, sizeof(pidBlockLoad));
	pidBlockLoad.ramPoolAvailable = MEMORY_SIZE;
}

FfbReportHandler::~FfbReportHandler() {
	FreeAllEffects();
}

uint8_t FfbReportHandler::GetNextFreeEffect(uint8_t effectType) {
	for (uint8_t i = 0; i < MAX_EFFECTS; i++) {
		if (gEffectStates[i].effectType == FFB_EFFECT_NONE || gEffectStates[i].effectType == effectType) {
			return (i + 1);
		}
	}
	return 0;
}

void FfbReportHandler::StopAllEffects(void) {
	for (uint8_t id = 0; id <= MAX_EFFECTS; id++)
		StopEffect(id);
}

void FfbReportHandler::StartEffect(uint8_t id) {
	if (id > MAX_EFFECTS)
		return;
	gEffectStates[id].state = MEFFECTSTATE_PLAYING;
	gEffectStates[id].counter = 0;
	gEffectStates[id].startTime = HAL_GetTick();
}

void FfbReportHandler::StopEffect(uint8_t id) {
	if (id > MAX_EFFECTS)
		return;
	gEffectStates[id].state = MEFFECTSTATE_ALLOCATED;
	pidBlockLoad.ramPoolAvailable += SIZE_EFFECT;

}

void FfbReportHandler::FreeEffect(uint8_t id) {
	if (id > MAX_EFFECTS)
		return;
	gEffectStates[id].state = 0;
	if (id < nextEID)
		nextEID = id;
}

void FfbReportHandler::FreeAllEffects(void) {
	nextEID = 1;
	memset((void*) &gEffectStates, 0, sizeof(gEffectStates));
	pidBlockLoad.ramPoolAvailable = MEMORY_SIZE;
}

void FfbReportHandler::FfbHandle_EffectOperation(
		USB_FFBReport_EffectOperation_Output_Data_t *data) {
	if (data->operation == 1) { // Start
		if (data->loopCount > 0)
			gEffectStates[data->effectBlockIndex-1].duration *= data->loopCount;
		if (data->loopCount == 0xFF)
			gEffectStates[data->effectBlockIndex-1].duration = USB_DURATION_INFINITE;
		StartEffect(data->effectBlockIndex-1);
	} else if (data->operation == 2) { // StartSolo

		// Stop all first
		StopAllEffects();

		// Then start the given effect
		StartEffect(data->effectBlockIndex-1);

	} else if (data->operation == 3) { // Stop

		StopEffect(data->effectBlockIndex-1);
	} else {
	}
}

void FfbReportHandler::FfbHandle_BlockFree(
		USB_FFBReport_BlockFree_Output_Data_t *data) {
	uint8_t eid = data->effectBlockIndex-1;

	if (eid == 0xFF) { // all effects
		FreeAllEffects();
	} else {
		FreeEffect(eid);
	}
}

void FfbReportHandler::FfbHandle_DeviceControl(
		USB_FFBReport_DeviceControl_Output_Data_t *data) {

	uint8_t control = data->control;

	if (control & 0x01) { // 1=Enable Actuators
		pidState.status |= 2;
		ffb_active = true;
	} else if (control & 0x02) { // 2=Disable Actuators
		pidState.status &= ~(0x02);
		ffb_active = false;
	} else if (control & 0x04) { // 4=Stop
		pidState.status &= ~(0x02);
		ffb_active = false;
	} else if (control & 0x03) { // 3=Stop All Effects
		StopAllEffects();
		ffb_active = false;
	} else if (control & 0x08) { //  4=Reset
		FreeAllEffects();
	} else if (control & 0x10) { // 5=Pause
		pidState.status &= ~(0x02);
		ffb_active = false;
	} else if (control & 0x20) { // 6=Continue
		pidState.status |= 2;
		ffb_active = false;
	}
}

void FfbReportHandler::FfbHandle_DeviceGain(
		USB_FFBReport_DeviceGain_Output_Data_Map_t *data) {
	deviceGain.gain = data->gain;
}

void FfbReportHandler::FfbHandle_SetCustomForce(
		USB_FFBReport_SetCustomForce_Output_Data_t *data) {
}

void FfbReportHandler::FfbHandle_SetCustomForceData(
		USB_FFBReport_SetCustomForceData_Output_Data_t *data) {
}

void FfbReportHandler::FfbHandle_SetDownloadForceSample(
		USB_FFBReport_SetDownloadForceSample_Output_Data_t *data) {
}

void FfbReportHandler::FfbHandle_SetEffect(
		USB_FFBReport_SetEffect_Output_Data_t *data) {
	uint8_t index = data->effectBlockIndex;
	if(index > MAX_EFFECTS || index == 0){
			return;
	}

	volatile TEffectState *effect = &gEffectStates[index-1];

	effect->duration = data->duration;
	effect->directionX = data->directionX;
	effect->directionY = data->directionY;
	effect->effectType = data->effectType;
	effect->gain = data->gain;
	effect->duration = data->duration;
	effect->period = data->samplePeriod;
	if (data->enableAxis & 0x4) {
		// All axes
		effect->axis = 0x7;
	} else {
		effect->axis = data->enableAxis;
	}
	if (effect->effectType != data->effectType) {
		effect->counter = 0;
		effect->last_value = 0;
	}
}

void FfbReportHandler::SetEnvelope(
		USB_FFBReport_SetEnvelope_Output_Data_t *data,
		volatile TEffectState *effect) {
	effect->attackLevel = data->attackLevel;
	effect->fadeLevel = data->fadeLevel;
	effect->attackTime = data->attackTime;
	effect->fadeTime = data->fadeTime;
}

void FfbReportHandler::SetCondition(
		USB_FFBReport_SetCondition_Output_Data_t *data,
		volatile TEffectState *effect) {
	if(data->parameterBlockOffset != 0) //TODO if more axes are needed. Only X Axis is implemented now for the wheel.
			return;
	effect->cpOffset = data->cpOffset;
	effect->positiveCoefficient = data->positiveCoefficient;
	effect->negativeCoefficient = data->negativeCoefficient;
	effect->positiveSaturation = data->positiveSaturation;
	effect->negativeSaturation = data->negativeSaturation;
	effect->deadBand = data->deadBand;
}

void FfbReportHandler::SetPeriodic(
		USB_FFBReport_SetPeriodic_Output_Data_t *data,
		volatile TEffectState *effect) {
	effect->magnitude = data->magnitude;
	effect->offset = data->offset;
	effect->phase = data->phase;
	effect->period = data->period;
}

void FfbReportHandler::SetConstantForce(
		USB_FFBReport_SetConstantForce_Output_Data_t *data,
		volatile TEffectState *effect) {
	effect->magnitude = data->magnitude;
}

void FfbReportHandler::SetRampForce(
		USB_FFBReport_SetRampForce_Output_Data_t *data,
		volatile TEffectState *effect) {
	effect->startMagnitude = data->startMagnitude;
	effect->endMagnitude = data->endMagnitude;
}

void FfbReportHandler::FfbOnCreateNewEffect(
		USB_FFBReport_CreateNewEffect_Feature_Data_t *inData) {

	uint8_t index = GetNextFreeEffect(inData->effectType); // next effect
	if (index == 0) {
		pidBlockLoad.loadStatus = 2;
		return;
	}
	//printf("Creating Effect: %d at %d\n",effect->effectType,index);

	volatile TEffectState *effect = &gEffectStates[index - 1];

	memset((void*) effect, 0, sizeof(TEffectState));
	effect->state = MEFFECTSTATE_ALLOCATED;
	effect->effectType = inData->effectType;

	pidBlockLoad.reportId = HID_ID_BLKLDREP;
	usedMemory += SIZE_EFFECT;
	pidState.effectBlockIndex = index;
	pidBlockLoad.effectBlockIndex = index;
	pidBlockLoad.ramPoolAvailable = MEMORY_SIZE - usedMemory;
	pidBlockLoad.loadStatus = 1;
}

uint8_t* FfbReportHandler::FfbOnPIDPool() {
//	FreeAllEffects();

	pidPoolReport.reportId = HID_ID_POOLREP;
	pidPoolReport.ramPoolSize = MEMORY_SIZE;
	pidPoolReport.maxSimultaneousEffects = MAX_EFFECTS;
	pidPoolReport.memoryManagement = 3;
	return (uint8_t*) &pidPoolReport;
}

uint8_t* FfbReportHandler::FfbOnPIDBlockLoad() {
	return (uint8_t*) &pidBlockLoad;
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&pidBlockLoad, sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t));
}

uint8_t* FfbReportHandler::FfbOnPIDStatus() {
	return (uint8_t*) &pidState;
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&pidState, sizeof(USB_FFBReport_PIDStatus_Input_Data_t));
}

/*
 * Sends a status report for a specific effect
 */
void FfbReportHandler::sendStatusReport(uint8_t effect){
	pidState.effectBlockIndex = effect;
	pidState.status = HID_ACTUATOR_POWER;
	if(ffb_active){
		pidState.status |= HID_ENABLE_ACTUATORS;
		pidState.status |= HID_EFFECT_PLAYING;
	}else{
		pidState.status |= HID_EFFECT_PAUSE;
	}
	if(effect > 0 && gEffectStates[effect-1].state == 1)
		pidState.status |= HID_EFFECT_PLAYING;
	//printf("Status: %d\n",reportFFBStatus.status);
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&pidState, sizeof(USB_FFBReport_PIDStatus_Input_Data_t));
}

void FfbReportHandler::FfbOnUsbData(uint8_t event_idx, uint8_t *data,
		uint16_t len) {

	uint8_t effectId = data[0]-1; // effectBlockIndex-1
	switch (event_idx)    // reportID
	{
		case HID_ID_EFFREP:
			FfbHandle_SetEffect((USB_FFBReport_SetEffect_Output_Data_t*) data);
			break;
		case HID_ID_ENVREP:
			SetEnvelope((USB_FFBReport_SetEnvelope_Output_Data_t*) data,
					&gEffectStates[effectId]);
			break;
		case HID_ID_CONDREP:
			SetCondition((USB_FFBReport_SetCondition_Output_Data_t*) data,
					&gEffectStates[effectId]);
			break;
		case HID_ID_PRIDREP:
			SetPeriodic((USB_FFBReport_SetPeriodic_Output_Data_t*) data,
					&gEffectStates[effectId]);
			break;
		case HID_ID_CONSTREP:
			SetConstantForce((USB_FFBReport_SetConstantForce_Output_Data_t*) data,
					&gEffectStates[effectId]);
			break;
		case HID_ID_RAMPREP:
			SetRampForce((USB_FFBReport_SetRampForce_Output_Data_t*) data,
					&gEffectStates[effectId]);
			break;
		case HID_ID_CSTMREP:
			FfbHandle_SetCustomForceData(
					(USB_FFBReport_SetCustomForceData_Output_Data_t*) data);
			break;
		case HID_ID_SMPLREP:
			FfbHandle_SetDownloadForceSample(
					(USB_FFBReport_SetDownloadForceSample_Output_Data_t*) data);
			break;
		case 9:
			break;
		case HID_ID_EFOPREP:
			FfbHandle_EffectOperation(
					(USB_FFBReport_EffectOperation_Output_Data_t*) data);
			break;
		case HID_ID_BLKFRREP:
			FfbHandle_BlockFree((USB_FFBReport_BlockFree_Output_Data_t*) data);
			break;
		case HID_ID_CTRLREP:
			FfbHandle_DeviceControl(
					(USB_FFBReport_DeviceControl_Output_Data_t*) data);
			sendStatusReport(effectId);
			break;
		case HID_ID_GAINREP:
			FfbHandle_DeviceGain((USB_FFBReport_DeviceGain_Output_Data_Map_t*) data);
			break;
		case HID_ID_SETCREP:
			FfbHandle_SetCustomForce(
					(USB_FFBReport_SetCustomForce_Output_Data_t*) data);
			break;
		case HID_ID_NEWEFREP:
			FfbOnCreateNewEffect(
					(USB_FFBReport_CreateNewEffect_Feature_Data_t*) data);
			break;
		default:
			break;
	}

}
;

int32_t FfbReportHandler::calculateEffects(int32_t pos, uint8_t axis = 1) {
//	if(!ffb_active){
//		// Center when FFB is turned of with a spring effect
//		if(idlecenter){
//			int16_t idlespringclip = clip<int32_t,int32_t>((int32_t)idlespringstregth*50,0,10000);
//			float idlespringscale = 0.5f + ((float)idlespringstregth * 0.01f);
//			return clip<int32_t,int32_t>((int32_t)(-pos*idlespringscale),-idlespringclip,idlespringclip);
//		}else{
//			return 0;
//		}
//	}

	int32_t result_torque = 0;

	for (uint8_t i = 0; i < MAX_EFFECTS; i++) {
		volatile TEffectState *effect = &gEffectStates[i];
		// Filter out inactive effects
		if (effect->state != 2)
			continue;

		switch (effect->effectType) {
			case FFB_EFFECT_CONSTANT: {	// Constant force is just the force
				int32_t f = ((int32_t) effect->magnitude * (int32_t) (1 + effect->gain))
						/ 256;
				// Optional filtering to reduce spikes
//			if(cfFilter_f < calcfrequency/2){
//				f = effect->filter->process(f);
//			}
				result_torque -= f;
				break;
			}
			case FFB_EFFECT_SPRING: {
				float scale = 0.0004f; // Tune for desired strength

				int32_t force = 0;

				if (abs(pos - effect->offset) > effect->deadBand) {
					if (pos < effect->offset) { // Deadband side
						force = clip<int32_t, int32_t>(
								(effect->negativeCoefficient * scale
										* (pos - (effect->offset - effect->deadBand))),
								-effect->negativeSaturation, effect->positiveSaturation);
					} else {
						force = clip<int32_t, int32_t>(
								(effect->positiveCoefficient * scale
										* (pos - (effect->offset + effect->deadBand))),
								-effect->negativeSaturation, effect->positiveSaturation);
					}
				}

				result_torque -= force;
				break;
			}

			case FFB_EFFECT_SQUARE: {

				int32_t force =
						((effect->counter + effect->phase) % ((uint32_t) effect->period + 2))
								< (uint32_t) (effect->period + 2) / 2 ?
								-effect->magnitude : effect->magnitude;
				force += effect->offset;
				result_torque -= force;
				break;
			}
			case FFB_EFFECT_SINE: {
				uint16_t t = effect->counter;
				float freq = 1.0f
						/ (float) (max<uint16_t>(uint16_t(effect->period), 2));
				float phase = (float) effect->phase / (float) 35999; //degrees
				float sine = sinf(2.0 * (float) M_PI * (t * freq + phase))
						* effect->magnitude;
				int32_t force = (int32_t) (effect->offset + sine);

				result_torque -= force;
				break;
			}
			case FFB_EFFECT_INERTIA: {
				// Acceleration
				break;
			}
			case FFB_EFFECT_FRICTION:
			case FFB_EFFECT_DAMPER: {

				int32_t force = 0;

				if (effect->counter == 0) {
					effect->last_value = pos;
					break;
				}
//				int32_t speed = pos - effect->last_value;
				effect->last_value = pos;

//				float val = effect->filter->process(speed) * 0.035f; // TODO tune friction

				// Only active outside deadband. Process filter always!
				if (abs(pos - effect->offset) < effect->deadBand) {
					break;
				}
				// Calculate force
				force = clip<int32_t, int32_t>(
						(int32_t) ((effect->positiveCoefficient)),
						-effect->negativeSaturation, effect->positiveSaturation);
				force = (frictionscale * force) / 1919;
				result_torque -= force;
				break;
			}
			default:
				// Unsupported effect
				break;
		}

		if (effect->counter++ > effect->duration) {
			effect->state = 0;
		}

	}
	result_torque = (result_torque * (gain + 1)); // Apply global gain

	return clip(result_torque,-1919,1919);
}
;
