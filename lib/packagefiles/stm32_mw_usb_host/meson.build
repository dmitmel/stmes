project('stm32_mw_usb_host', 'c', version: '3.3.4')

stm32_usb_host_core_dep = declare_dependency(
  sources: files(
    'Core/Src/usbh_core.c',
    'Core/Src/usbh_ctlreq.c',
    'Core/Src/usbh_ioreq.c',
    'Core/Src/usbh_pipes.c',
  ),
  include_directories: include_directories('Core/Inc'),
)

stm32_usb_host_hid_class_dep = declare_dependency(
  sources: files(
    'Class/HID/Src/usbh_hid.c',
    'Class/HID/Src/usbh_hid_keybd.c',
    'Class/HID/Src/usbh_hid_mouse.c',
    'Class/HID/Src/usbh_hid_parser.c',
  ),
  include_directories: include_directories('Class/HID/Inc'),
)
