/*
 * FfbReportHandlerWrapper.h
 *
 *  Created on: Sep 29, 2023
 *      Author: Vielder
 */

#ifndef INC_FFBREPORTHANDLERWRAPPER_H_
#define INC_FFBREPORTHANDLERWRAPPER_H_

#include "HIDReportType.h"

#ifdef __cplusplus
extern "C" {
#endif

	void* createFfbReportHandler();
	uint8_t* callFfbOnPIDBlockLoad(void *handler);
	uint8_t* callFfbOnPIDPool(void *handler);
	void callFfbOnCreateNewEffect(void *handler, FFB_CreateNewEffect_Feature_Data_t *inData);
	void callFfbOnUsbData(void *handler, uint8_t event_idx, uint8_t *data, uint16_t len);
	int32_t callCalculateEffects(void *handler, int32_t pos, uint8_t axis);
	TEffectState_c* callGetEffectData(void *handler, uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* INC_FFBREPORTHANDLERWRAPPER_H_ */
