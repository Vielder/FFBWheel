/*
 * FfbWheel.h
 *
 *  Created on: Sep 27, 2023
 *      Author: Vielder
 */

#ifndef INC_FFBWHEEL_H_
#define INC_FFBWHEEL_H_

#include "usb_device.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"
#include <stdint.h>

typedef struct {
	uint8_t buttons = 0x00;
	uint16_t xAxis;
	uint16_t yAxis;
	uint16_t zAxis;
	uint16_t rxAxis;
	uint16_t ryAxis;
	uint16_t rzAxis;

} WheelReport;

class Wheel_ {
private:
	WheelReport _wheelReport;

public:
	Wheel_(void);
	void write(void);
	void press(uint8_t b);
	void release(uint8_t b);
	void releaseAll(void);
	void buttons(uint8_t b);
	void xAxis(uint16_t a);
	void yAxis(uint16_t a);
	void zAxis(uint16_t a);
	void rxAxis(uint16_t a);
	uint8_t AvailableReport();
	void RecvFfbReport();

};
extern Wheel_ Wheel;

extern USBD_HandleTypeDef hUsbDeviceFS;

#endif /* INC_FFBWHEEL_H_ */
