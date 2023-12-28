project('stm32f4xx_hal_driver', 'c', version: '1.8.1')

inc = include_directories('Inc')
all_files = files('Src/stm32f4xx_hal.c')

foreach component : [
  'adc', 'adc_ex', 'can', 'cec', 'cortex', 'crc', 'cryp', 'cryp_ex', 'dac',
  'dac_ex', 'dcmi', 'dcmi_ex', 'dfsdm', 'dma', 'dma2d', 'dma_ex', 'dsi', 'eth',
  'exti', 'flash', 'flash_ex', 'flash_ramfunc', 'fmpi2c', 'fmpi2c_ex',
  'fmpsmbus', 'fmpsmbus_ex', 'gpio', 'hash', 'hash_ex', 'hcd', 'i2c', 'i2c_ex',
  'i2s', 'i2s_ex', 'irda', 'iwdg', 'lptim', 'ltdc', 'ltdc_ex', 'mmc', 'nand',
  'nor', 'pccard', 'pcd', 'pcd_ex', 'pwr', 'pwr_ex', 'qspi', 'rcc', 'rcc_ex',
  'rng', 'rtc', 'rtc_ex', 'sai', 'sai_ex', 'sd', 'sdram', 'smartcard', 'smbus',
  'spdifrx', 'spi', 'sram', 'tim', 'tim_ex', 'uart', 'usart', 'wwdg',
]
  all_files += files('Src/stm32f4xx_hal_' + component + '.c')
endforeach

foreach component : [
  'adc', 'crc', 'dac', 'dma', 'dma2d', 'exti', 'fmc', 'fmpi2c', 'fsmc', 'gpio',
  'i2c', 'lptim', 'pwr', 'rcc', 'rng', 'rtc', 'sdmmc', 'spi', 'tim', 'usart',
  'usb', 'utils',
]
  all_files += files('Src/stm32f4xx_ll_' + component + '.c')
endforeach

stm32f4xx_hal_driver_dep = declare_dependency(sources: all_files, include_directories: inc)
