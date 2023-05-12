# The (homemade) STM Entertainment System

Some useful links:

- <https://stm32-base.org/boards/STM32F411CEU6-WeAct-Black-Pill-V2.0> - the board I am using
- <https://www.st.com/resource/en/datasheet/stm32f411ce.pdf> - STM32F411CE datasheet
- <https://www.st.com/resource/en/reference_manual/rm0383-stm32f411xce-advanced-armbased-32bit-mcus-stmicroelectronics.pdf> - STM32F411xx reference manual
- <https://www.st.com/resource/en/errata_sheet/es0287-stm32f411xc-and-stm32f411xe-device-limitations-stmicroelectronics.pdf> - STM32F411xx errata
- <https://cheatography.com/syshella/cheat-sheets/arm-assembly/> - ARM assembly cheat sheet
- <https://www.st.com/resource/en/programming_manual/pm0214-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf> - Cortex-M4 programming manual from STM
- <http://tinyvga.com/vga-timing> - VGA timings and clock frequencies
  - <http://tinyvga.com/vga-timing/640x480@60Hz>
  - <http://tinyvga.com/vga-timing/800x600@60Hz>
- <https://lateblt.tripod.com/bit74.txt> - some really useful info on the practical implementation of VGA, plus the complete pinout
- <https://www.youtube.com/watch?v=l7rce6IQDWs> - "the world's worst video card", the video is very informative and describes the interface and the implementation in great detail
  - <https://www.youtube.com/watch?v=uqY3FMuMuRo> - part 2
  - <https://www.youtube.com/watch?v=2iURr3NBprc> - part 3
  - <https://www.youtube.com/watch?v=BUTHtNrpwiI> - part 4
- <https://github.com/abelykh0/VGA-demo-on-bluepill> - an implementation of VGA on another STM32 controller (in particular, the idea of repurposing PWM for sync pulse generation was stolen from here)
- <http://cliffle.com/p/m4vga/> - this entire series of posts, the author uses STM32F407
  - <http://cliffle.com/blog/introducing-glitch/> - a demo of what it can do (very impressive!)
  - <http://cliffle.com/blog/pushing-pixels/> - explains the usage of DMA on STM32F407 and its different RAM banks
  - <http://cliffle.com/blog/glitch-in-the-matrix/> - introduces various hacks to reduce bus matrix contention
  - <http://cliffle.com/blog/racing-the-beam/> - describes the challenges of real-time rasterization and some assembly-level hacks
- <https://www.youtube.com/watch?v=8z-pJSTi7ME> - usage of DMA for fast GPIO output
  - <https://github.com/mnemocron/STM32_PatternDriver> - the code from the video
- <http://www.efton.sk/STM32/gotcha/g30.html> - why only DMA2 (and not DMA1) can drive GPIO
- <https://stackoverflow.com/questions/46613053/pwm-dma-to-a-whole-gpio/46619315> - more info on the usage of GPIO with DMA and timers
