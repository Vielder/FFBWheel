/*
 * FfbReportHandler.cpp
 *
 *  Created on: Sep 24, 2023
 *      Author: Vielder
 */

#include "FfbReportHandler.h"



FfbReportHandler::FfbReportHandler() {
  nextEID = 1;
  devicePaused = 0;
  usedMemory = 0;
  FreeAllEffects();
  memset((void*)&pidBlockLoad, 0, sizeof(pidBlockLoad));
  pidBlockLoad.ramPoolAvailable = MEMORY_SIZE;
}

FfbReportHandler::~FfbReportHandler() {
  FreeAllEffects();
}


uint8_t FfbReportHandler::GetNextFreeEffect(uint8_t effectType)
{
	for(uint8_t i=0;i<MAX_EFFECTS;i++){
			if(gEffectStates[i].effectType == 0){
				return(i+1);
			}
		}
		return 0;
}

void FfbReportHandler::StopAllEffects(void)
{
  for (uint8_t id = 0; id <= MAX_EFFECTS; id++)
    StopEffect(id);
}

void FfbReportHandler::StartEffect(uint8_t id)
{
  if (id > MAX_EFFECTS)
    return;
  gEffectStates[id].state = MEFFECTSTATE_PLAYING;
  gEffectStates[id].elapsedTime = 0;
  gEffectStates[id].startTime = HAL_GetTick();
}

void FfbReportHandler::StopEffect(uint8_t id)
{
  if (id > MAX_EFFECTS)
    return;
  gEffectStates[id].state &= ~MEFFECTSTATE_PLAYING;
  pidBlockLoad.ramPoolAvailable += SIZE_EFFECT;

}

void FfbReportHandler::FreeEffect(uint8_t id)
{
  if (id > MAX_EFFECTS)
    return;
  gEffectStates[id].state = 0;
  if (id < nextEID)
    nextEID = id;
}

void FfbReportHandler::FreeAllEffects(void)
{
  nextEID = 1;
  memset((void*)&gEffectStates, 0, sizeof(gEffectStates));
  pidBlockLoad.ramPoolAvailable = MEMORY_SIZE;
}

void FfbReportHandler::FfbHandle_EffectOperation(USB_FFBReport_EffectOperation_Output_Data_t* data)
{
  if (data->operation == 1)
  { // Start
    if (data->loopCount > 0) gEffectStates[data->effectBlockIndex].duration *= data->loopCount;
    if (data->loopCount == 0xFF) gEffectStates[data->effectBlockIndex].duration = USB_DURATION_INFINITE;
    StartEffect(data->effectBlockIndex);
  }
  else if (data->operation == 2)
  { // StartSolo

    // Stop all first
    StopAllEffects();

    // Then start the given effect
    StartEffect(data->effectBlockIndex);

  }
  else if (data->operation == 3)
  { // Stop

    StopEffect(data->effectBlockIndex);
  }
  else
  {
  }
}

void FfbReportHandler::FfbHandle_BlockFree(USB_FFBReport_BlockFree_Output_Data_t* data)
{
  uint8_t eid = data->effectBlockIndex;

  if (eid == 0xFF)
  { // all effects
    FreeAllEffects();
  }
  else
  {
    FreeEffect(eid);
  }
}

void FfbReportHandler::FfbHandle_DeviceControl(USB_FFBReport_DeviceControl_Output_Data_t* data)
{

  uint8_t control = data->control;

  if (control == 0x01)
  { // 1=Enable Actuators
    pidState.status |= 2;
  }
  else if (control == 0x02)
  { // 2=Disable Actuators
    pidState.status &= ~(0x02);
  }
  else if (control == 0x03)
  { // 3=Stop All Effects
    StopAllEffects();
  }
  else if (control == 0x04)
  { //  4=Reset
    FreeAllEffects();
  }
  else if (control == 0x05)
  { // 5=Pause
    devicePaused = 1;
  }
  else if (control == 0x06)
  { // 6=Continue
    devicePaused = 0;
  }
  else if (control & (0xFF - 0x3F))
  {
  }
}

void FfbReportHandler::FfbHandle_DeviceGain(USB_FFBReport_DeviceGain_Output_Data_Map_t* data)
{
  deviceGain.gain = data->gain;
}


void FfbReportHandler::FfbHandle_SetCustomForce(USB_FFBReport_SetCustomForce_Output_Data_t* data)
{
}


void FfbReportHandler::FfbHandle_SetCustomForceData(USB_FFBReport_SetCustomForceData_Output_Data_t* data)
{
}

void FfbReportHandler::FfbHandle_SetDownloadForceSample(USB_FFBReport_SetDownloadForceSample_Output_Data_t* data)
{
}

void FfbReportHandler::FfbHandle_SetEffect(USB_FFBReport_SetEffect_Output_Data_t* data)
{
  volatile TEffectState* effect = &gEffectStates[data->effectBlockIndex];

  effect->duration = data->duration;
  effect->directionX = data->directionX;
  effect->directionY = data->directionY;
  effect->effectType = data->effectType;
  effect->gain = data->gain;
  effect->enableAxis = data->enableAxis;
  //  effect->triggerRepeatInterval;
  //  effect->samplePeriod;   // 0..32767 ms
  //  effect->triggerButton;
}

void FfbReportHandler::SetEnvelope(USB_FFBReport_SetEnvelope_Output_Data_t* data, volatile TEffectState* effect)
{
  effect->attackLevel = data->attackLevel;
  effect->fadeLevel = data->fadeLevel;
  effect->attackTime = data->attackTime;
  effect->fadeTime = data->fadeTime;
}

void FfbReportHandler::SetCondition(USB_FFBReport_SetCondition_Output_Data_t* data, volatile TEffectState* effect)
{
  effect->cpOffset = data->cpOffset;
  effect->positiveCoefficient = data->positiveCoefficient;
  effect->negativeCoefficient = data->negativeCoefficient;
  effect->positiveSaturation = data->positiveSaturation;
  effect->negativeSaturation = data->negativeSaturation;
  effect->deadBand = data->deadBand;
}

void FfbReportHandler::SetPeriodic(USB_FFBReport_SetPeriodic_Output_Data_t* data, volatile TEffectState* effect)
{
  effect->magnitude = data->magnitude;
  effect->offset = data->offset;
  effect->phase = data->phase;
  effect->period = data->period;
}

void FfbReportHandler::SetConstantForce(USB_FFBReport_SetConstantForce_Output_Data_t* data, volatile TEffectState* effect)
{
  effect->magnitude = data->magnitude;
}

void FfbReportHandler::SetRampForce(USB_FFBReport_SetRampForce_Output_Data_t* data, volatile TEffectState* effect)
{
  effect->startMagnitude = data->startMagnitude;
  effect->endMagnitude = data->endMagnitude;
}

void FfbReportHandler::FfbOnCreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData)
{
  pidBlockLoad.reportId = HID_ID_BLKLDREP;

  uint8_t index = GetNextFreeEffect(inData->effectType); // next effect
	if(index == 0){
		pidBlockLoad.loadStatus = 2;
		return;
	}

	volatile TEffectState* effect = &gEffectStates[index-1];

	memset((void*)effect, 0, sizeof(TEffectState));
	effect->state = MEFFECTSTATE_ALLOCATED;
	effect->effectType = inData->effectType;
	usedMemory += SIZE_EFFECT;
	pidBlockLoad.effectBlockIndex = index;
	pidBlockLoad.ramPoolAvailable = MEMORY_SIZE - usedMemory;
	pidBlockLoad.loadStatus = 1;
}

uint8_t* FfbReportHandler::FfbOnPIDPool()
{
//	FreeAllEffects();

  pidPoolReport.reportId = HID_ID_POOLREP;
  pidPoolReport.ramPoolSize = MEMORY_SIZE;
  pidPoolReport.maxSimultaneousEffects = MAX_EFFECTS;
  pidPoolReport.memoryManagement = 3;
  return (uint8_t*)&pidPoolReport;
}

uint8_t* FfbReportHandler::FfbOnPIDBlockLoad()
{
  return (uint8_t*)&pidBlockLoad;
}

uint8_t* FfbReportHandler::FfbOnPIDStatus()
{
  return (uint8_t*)&pidState;
}

void FfbReportHandler::FfbOnUsbData(uint8_t event_idx ,uint8_t* data, uint16_t len)
{

  uint8_t effectId = data[0]; // effectBlockIndex is always the second byte.
  switch (event_idx)    // reportID
  {
    case HID_ID_EFFREP:
      FfbHandle_SetEffect((USB_FFBReport_SetEffect_Output_Data_t*)data);
      break;
    case HID_ID_ENVREP:
      SetEnvelope((USB_FFBReport_SetEnvelope_Output_Data_t*)data, &gEffectStates[effectId]);
      break;
    case HID_ID_CONDREP:
      SetCondition((USB_FFBReport_SetCondition_Output_Data_t*)data, &gEffectStates[effectId]);
      break;
    case HID_ID_PRIDREP:
      SetPeriodic((USB_FFBReport_SetPeriodic_Output_Data_t*)data, &gEffectStates[effectId]);
      break;
    case HID_ID_CONSTREP:
      SetConstantForce((USB_FFBReport_SetConstantForce_Output_Data_t*)data, &gEffectStates[effectId]);
      break;
    case HID_ID_RAMPREP:
      SetRampForce((USB_FFBReport_SetRampForce_Output_Data_t*)data, &gEffectStates[effectId]);
      break;
    case HID_ID_CSTMREP:
      FfbHandle_SetCustomForceData((USB_FFBReport_SetCustomForceData_Output_Data_t*)data);
      break;
    case HID_ID_SMPLREP:
      FfbHandle_SetDownloadForceSample((USB_FFBReport_SetDownloadForceSample_Output_Data_t*)data);
      break;
    case 9:
      break;
    case HID_ID_EFOPREP:
      FfbHandle_EffectOperation((USB_FFBReport_EffectOperation_Output_Data_t*)data);
      break;
    case HID_ID_BLKFRREP:
      FfbHandle_BlockFree((USB_FFBReport_BlockFree_Output_Data_t*)data);
      break;
    case HID_ID_CTRLREP:
      FfbHandle_DeviceControl((USB_FFBReport_DeviceControl_Output_Data_t*)data);
      break;
    case HID_ID_GAINREP:
      FfbHandle_DeviceGain((USB_FFBReport_DeviceGain_Output_Data_Map_t*)data);
      break;
    case HID_ID_SETCREP:
      FfbHandle_SetCustomForce((USB_FFBReport_SetCustomForce_Output_Data_t*)data);
      break;
    case HID_ID_NEWEFREP:
    	FfbOnCreateNewEffect((USB_FFBReport_CreateNewEffect_Feature_Data_t*)data);
			break;
    default:
      break;
  }
};
