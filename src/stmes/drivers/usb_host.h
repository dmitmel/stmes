#pragma once

#include <stm32f4xx_hal.h>
#include <usbh_core.h>

#ifdef __cplusplus
extern "C" {
#endif

enum UsbHostState {
  USB_HOST_IDLE = 0,
  USB_HOST_START,
  USB_HOST_READY,
  USB_HOST_DISCONNECT,
};

extern HCD_HandleTypeDef hhcd_USB_OTG_FS;
extern USBH_HandleTypeDef hUsbHostFS;
extern struct Notification usb_notification;
extern struct Channel usb_keyboard_events;

void MX_USB_HOST_Init(void);
void MX_USB_HOST_Process(void);

#ifdef __cplusplus
}
#endif
