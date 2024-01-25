/*
 * FfbReportHandlerWrapper.cpp
 *
 *  Created on: Sep 29, 2023
 *      Author: Vielder
 */

#include "FfbReportHandlerWrapper.h"
#include "FfbReportHandler.h"

#ifdef __cplusplus
extern "C" {
#endif

void* createFfbReportHandler() {
    return new FfbReportHandler();
}

extern "C" uint8_t* callFfbOnPIDBlockLoad(void* handler) {
	return static_cast<FfbReportHandler*>(handler)->FfbOnPIDBlockLoad();
}

extern "C" uint8_t* callFfbOnPIDPool(void* handler) {
	return static_cast<FfbReportHandler*>(handler)->FfbOnPIDPool();
}

extern "C" void callFfbOnCreateNewEffect(void* handler,
		FFB_CreateNewEffect_Feature_Data_t *inData) {
	static_cast<FfbReportHandler*>(handler)->FfbOnCreateNewEffect(inData);
}

extern "C" void callFfbOnUsbData(void* handler, uint8_t event_idx, uint8_t* data, uint16_t len) {
	static_cast<FfbReportHandler*>(handler)->FfbOnUsbData(event_idx, data, len);
}

extern "C" int32_t callCalculateEffects(void *handler, int32_t pos, uint8_t axis) {
	return static_cast<FfbReportHandler*>(handler)->calculateEffects(pos, axis);
}

#ifdef __cplusplus
}
#endif

