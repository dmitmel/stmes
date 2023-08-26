#include "stmes/drivers/usb_host.h"
#include "stmes/interrupts.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/sync.h"
#include "stmes/kernel/task.h"
#include <usbh_core.h>
#include <usbh_hid.h>

HCD_HandleTypeDef hhcd_USB_OTG_FS;
USBH_HandleTypeDef hUsbHostFS;
enum UsbHostState usb_host_state = USB_HOST_IDLE;
struct Notification usb_notification;
struct Channel usb_keyboard_events;

static __attribute__((constructor)) void init_usb_channels(void) {
  task_notify_init(&usb_notification);
  channel_init(&usb_keyboard_events);
}

void OTG_FS_IRQHandler(void) {
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
  if (task_notify(&usb_notification)) {
    task_yield_from_isr();
  }
}

void USBH_HID_EventCallback(USBH_HandleTypeDef* phost) {
  if (USBH_HID_GetDeviceType(phost) == HID_KEYBOARD) {
    HID_KEYBD_Info_TypeDef* keyboard_info = USBH_HID_GetKeybdInfo(phost);
    channel_send(&usb_keyboard_events, keyboard_info, sizeof(*keyboard_info));
  }
}

static USBH_StatusTypeDef USBH_Get_USB_Status(HAL_StatusTypeDef status) {
  switch (status) {
    case HAL_OK: return USBH_OK;
    case HAL_ERROR: return USBH_FAIL;
    case HAL_BUSY: return USBH_BUSY;
    case HAL_TIMEOUT: return USBH_FAIL;
    default: return USBH_FAIL;
  }
}

void HAL_HCD_MspInit(HCD_HandleTypeDef* hcdHandle) {
  if (hcdHandle->Instance == USB_OTG_FS) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init = {
      .Pin = GPIO_PIN_11 | GPIO_PIN_12,
      .Mode = GPIO_MODE_AF_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
      .Alternate = GPIO_AF10_OTG_FS,
    };
    HAL_GPIO_Init(GPIOA, &gpio_init);
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  }
}

void HAL_HCD_MspDeInit(HCD_HandleTypeDef* hcdHandle) {
  if (hcdHandle->Instance == USB_OTG_FS) {
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
  }
}

void HAL_HCD_SOF_Callback(HCD_HandleTypeDef* hhcd) {
  USBH_LL_IncTimer(hhcd->pData);
}

void HAL_HCD_Connect_Callback(HCD_HandleTypeDef* hhcd) {
  USBH_LL_Connect(hhcd->pData);
}

void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef* hhcd) {
  USBH_LL_Disconnect(hhcd->pData);
}

void HAL_HCD_HC_NotifyURBChange_Callback(
  HCD_HandleTypeDef* hhcd, u8 chnum, HCD_URBStateTypeDef urb_state
) {
#if USBH_USE_OS == 1
  USBH_LL_NotifyURBChange(hhcd->pData);
#else
  UNUSED(hhcd), UNUSED(chnum), UNUSED(urb_state);
#endif
}

void HAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef* hhcd) {
  USBH_LL_PortEnabled(hhcd->pData);
}

void HAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef* hhcd) {
  USBH_LL_PortDisabled(hhcd->pData);
}

USBH_StatusTypeDef USBH_LL_Init(USBH_HandleTypeDef* phost) {
  if (phost->id == HOST_FS) {
    hhcd_USB_OTG_FS.pData = phost;
    phost->pData = &hhcd_USB_OTG_FS;
    hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
    hhcd_USB_OTG_FS.Init = (HCD_InitTypeDef){
      .Host_channels = 8,
      .speed = HCD_SPEED_LOW,
      .dma_enable = DISABLE,
      .phy_itface = HCD_PHY_EMBEDDED,
      .Sof_enable = DISABLE,
    };
    if (HAL_HCD_Init(&hhcd_USB_OTG_FS) != HAL_OK) {
      CRASH("USB ERROR");
    }
    USBH_LL_SetTimer(phost, HAL_HCD_GetCurrentFrame(&hhcd_USB_OTG_FS));
  }
  return USBH_OK;
}

USBH_StatusTypeDef USBH_LL_DeInit(USBH_HandleTypeDef* phost) {
  return USBH_Get_USB_Status(HAL_HCD_DeInit(phost->pData));
}

USBH_StatusTypeDef USBH_LL_Start(USBH_HandleTypeDef* phost) {
  return USBH_Get_USB_Status(HAL_HCD_Start(phost->pData));
}

USBH_StatusTypeDef USBH_LL_Stop(USBH_HandleTypeDef* phost) {
  return USBH_Get_USB_Status(HAL_HCD_Stop(phost->pData));
}

USBH_SpeedTypeDef USBH_LL_GetSpeed(USBH_HandleTypeDef* phost) {
  switch (HAL_HCD_GetCurrentSpeed(phost->pData)) {
    case 0: return USBH_SPEED_HIGH;
    case 1: return USBH_SPEED_FULL;
    case 2: return USBH_SPEED_LOW;
    default: return USBH_SPEED_FULL;
  }
}

USBH_StatusTypeDef USBH_LL_ResetPort(USBH_HandleTypeDef* phost) {
  return USBH_Get_USB_Status(HAL_HCD_ResetPort(phost->pData));
}

u32 USBH_LL_GetLastXferSize(USBH_HandleTypeDef* phost, u8 pipe) {
  return HAL_HCD_HC_GetXferCount(phost->pData, pipe);
}

USBH_StatusTypeDef USBH_LL_OpenPipe(
  USBH_HandleTypeDef* phost, u8 pipe_num, u8 epnum, u8 dev_address, u8 speed, u8 ep_type, u16 mps
) {
  return USBH_Get_USB_Status(
    HAL_HCD_HC_Init(phost->pData, pipe_num, epnum, dev_address, speed, ep_type, mps)
  );
}

USBH_StatusTypeDef USBH_LL_ClosePipe(USBH_HandleTypeDef* phost, u8 pipe) {
  return USBH_Get_USB_Status(HAL_HCD_HC_Halt(phost->pData, pipe));
}

USBH_StatusTypeDef USBH_LL_SubmitURB(
  USBH_HandleTypeDef* phost,
  u8 pipe,
  u8 direction,
  u8 ep_type,
  u8 token,
  u8* pbuff,
  u16 length,
  u8 do_ping
) {
  return USBH_Get_USB_Status(
    HAL_HCD_HC_SubmitRequest(phost->pData, pipe, direction, ep_type, token, pbuff, length, do_ping)
  );
}

USBH_URBStateTypeDef USBH_LL_GetURBState(USBH_HandleTypeDef* phost, u8 pipe) {
  return (USBH_URBStateTypeDef)HAL_HCD_HC_GetURBState(phost->pData, pipe);
}

USBH_StatusTypeDef USBH_LL_DriverVBUS(USBH_HandleTypeDef* phost, u8 state) {
  if (phost->id == HOST_FS) {
    if (state == 0) {
      /* ToDo: Add IOE driver control */
    } else {
      /* ToDo: Add IOE driver control */
    }
  }
  task_sleep(200);
  return USBH_OK;
}

USBH_StatusTypeDef USBH_LL_SetToggle(USBH_HandleTypeDef* phost, u8 pipe, u8 toggle) {
  HCD_HandleTypeDef* pHandle = phost->pData;
  HCD_HCTypeDef* hc = &pHandle->hc[pipe];
  if (hc->ep_is_in) {
    hc->toggle_in = toggle;
  } else {
    hc->toggle_out = toggle;
  }
  return USBH_OK;
}

u8 USBH_LL_GetToggle(USBH_HandleTypeDef* phost, u8 pipe) {
  HCD_HandleTypeDef* pHandle = phost->pData;
  HCD_HCTypeDef* hc = &pHandle->hc[pipe];
  if (hc->ep_is_in) {
    return hc->toggle_in;
  } else {
    return hc->toggle_out;
  }
}

void USBH_Delay(u32 delay) {
  task_sleep(delay);
}

static void USBH_UserProcess(USBH_HandleTypeDef* phost, u8 id) {
  UNUSED(phost);
  switch (id) {
    case HOST_USER_SELECT_CONFIGURATION: break;
    case HOST_USER_DISCONNECTION: usb_host_state = USB_HOST_DISCONNECT; break;
    case HOST_USER_CLASS_ACTIVE: usb_host_state = USB_HOST_READY; break;
    case HOST_USER_CONNECTION: usb_host_state = USB_HOST_START; break;
    default: break;
  }
}

void MX_USB_HOST_Init(void) {
  if (USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS) != USBH_OK) {
    CRASH("USB Error");
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_HID_CLASS) != USBH_OK) {
    CRASH("USB Error");
  }
  if (USBH_Start(&hUsbHostFS) != USBH_OK) {
    CRASH("USB Error");
  }
}

void MX_USB_HOST_Process(void) {
  USBH_Process(&hUsbHostFS);
}
