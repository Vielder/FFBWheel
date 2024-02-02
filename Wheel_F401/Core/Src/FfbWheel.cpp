#include "FfbWheel.h"
Wheel_::Wheel_(void) {

}

void Wheel_::write(void) {
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &_wheelReport.buttons,
			sizeof(_wheelReport.buttons));
}

void Wheel_::press(uint8_t b) {
	_wheelReport.buttons |= (uint8_t) 1 << (b - 1);
}

void Wheel_::release(uint8_t b) {
	_wheelReport.buttons &= ~((uint8_t) 1 << (b - 1));
}

void Wheel_::releaseAll(void) {
//  memset(&_wheelReport, 0x00, sizeof(_wheelReport));
}

void Wheel_::buttons(uint8_t b) {
	_wheelReport.buttons = b;
}

void Wheel_::xAxis(uint16_t a) {
	_wheelReport.xAxis = a;
}

void Wheel_::yAxis(uint16_t a) {
	_wheelReport.yAxis = a;
}

void Wheel_::zAxis(uint16_t a) {
	_wheelReport.zAxis = a;
}
void Wheel_::rxAxis(uint16_t a) {
	_wheelReport.rxAxis = a;
}
