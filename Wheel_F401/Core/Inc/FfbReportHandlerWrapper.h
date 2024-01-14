/*
 * FfbReportHandlerWrapper.h
 *
 *  Created on: Sep 29, 2023
 *      Author: Vielder
 */

#ifndef INC_FFBREPORTHANDLERWRAPPER_H_
#define INC_FFBREPORTHANDLERWRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "HIDReportType.h"

void* createFfbReportHandler();
uint8_t* callFfbOnPIDBlockLoad(void *handler);
uint8_t* callFfbOnPIDPool(void *handler);
void callFfbOnCreateNewEffect(void *handler,
		USB_FFBReport_CreateNewEffect_Feature_Data_t *inData);
void callFfbOnUsbData(void *handler, uint8_t event_idx, uint8_t *data,
		uint16_t len);
int32_t callCalculateEffects(void *handler, int32_t pos, uint8_t axis);

#ifdef __cplusplus
}
#endif

#endif /* INC_FFBREPORTHANDLERWRAPPER_H_ */
