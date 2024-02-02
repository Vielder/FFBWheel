/*
 * FfbReportHandler.h
 *
 *  Created on: Sep 24, 2023
 *      Author: Vielder
 */

#ifndef INC_FFBREPORTHANDLER_H_
#define INC_FFBREPORTHANDLER_H_

#ifdef __cplusplus

#include "HIDReportType.h"
#include "main.h"
#include <chrono>
#include <stdint.h>
#include <cstring>
#include "stm32f4xx_hal.h"

class FfbReportHandler {
public:
	FfbReportHandler();
	~FfbReportHandler();
	// Effect management
	volatile uint8_t nextEID;  // FFP effect indexes starts from 1
	volatile TEffectState gEffectStates[MAX_EFFECTS];
	volatile uint8_t devicePaused;
	volatile uint16_t usedMemory = 0;
	uint8_t frictionscale = 127;
	uint8_t gain = 1;
	bool ffb_active = false;

	//variables for storing previous values
	volatile int32_t inertiaT = 0;
	volatile int16_t oldSpeed = 0;
	volatile int16_t oldAxisPosition = 0;
	volatile USB_FFBReport_PIDStatus_Input_Data_t pidState = { 2, 30, 0 };
	volatile USB_FFBReport_PIDBlockLoad_Feature_Data_t pidBlockLoad;
	volatile USB_FFBReport_PIDPool_Feature_Data_t pidPoolReport;
	volatile USB_FFBReport_DeviceGain_Output_Data_Map_t deviceGain;

	//ffb state structures
	uint8_t GetNextFreeEffect(uint8_t effectType);
	void StartEffect(uint8_t id);
	void StopEffect(uint8_t id);
	void StopAllEffects(void);
	void FreeEffect(uint8_t id);
	void FreeAllEffects(void);

	//handle output report
	void FfbHandle_EffectOperation(USB_FFBReport_EffectOperation_Output_Data_t *data);
	void FfbHandle_BlockFree(USB_FFBReport_BlockFree_Output_Data_t *data);
	void FfbHandle_DeviceControl(USB_FFBReport_DeviceControl_Output_Data_t *data);
	void FfbHandle_DeviceGain(USB_FFBReport_DeviceGain_Output_Data_Map_t *data);
	void FfbHandle_SetCustomForceData(USB_FFBReport_SetCustomForceData_Output_Data_t *data);
	void FfbHandle_SetDownloadForceSample(USB_FFBReport_SetDownloadForceSample_Output_Data_t *data);
	void FfbHandle_SetCustomForce(USB_FFBReport_SetCustomForce_Output_Data_t *data);
	void FfbHandle_SetEffect(FFB_SetEffect_t *data);
	void SetEnvelope(USB_FFBReport_SetEnvelope_Output_Data_t *data, volatile TEffectState *effect);
	void SetCondition(FFB_SetCondition_Data_t *data, volatile TEffectState *effect);
	void SetPeriodic(FFB_SetPeriodic_Data_t *data, volatile TEffectState *effect);
	void SetConstantForce(FFB_SetConstantForce_Data_t *data, volatile TEffectState *effect);
	void SetRampForce(USB_FFBReport_SetRampForce_Output_Data_t *data, volatile TEffectState *effect);

	void sendStatusReport(uint8_t effect);

	// Handle incoming data from USB
	void FfbOnCreateNewEffect(FFB_CreateNewEffect_Feature_Data_t *inData);
	void FfbOnUsbData(uint8_t event_idx, uint8_t *data, uint16_t len);
	uint8_t* FfbOnPIDPool();
	uint8_t* FfbOnPIDBlockLoad();
	uint8_t* FfbOnPIDStatus();

	TEffectState* getEffectData(uint8_t id);
	int32_t calculateEffects(int32_t pos, uint8_t axis);

};

extern FfbReportHandler ffbReportHandler;

#endif

#endif /* INC_FFBREPORTHANDLER_H_ */
