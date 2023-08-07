// Patching of functions within the STM32 HAL works essentially by declaring
// them weakly linked and providing our own non-weak implementations. To do
// this I abuse the fact that (at least under GCC) a function is marked as a
// weak symbol if any of its declarations (or the definition) have
// `__attribute__((weak))`, or there is a `#pragma weak` preceding the
// definition, which is what I use purely because that latter method requires
// less typing (since the declarations with all the parameters don't need to be
// duplicated here). To reliably insert the pragmas I also exploit the fact
// that the stm32f4xx_hal_conf.h file is indirectly referenced by every C file
// in the HAL, which then includes this file before any of the HAL headers. The
// source files which contain the patched implementations should define an
// appropriate symbol *before including any headers* to turn off the weak
// declarations of the patched functions, so that the linker is certain about
// what to link.
//
// It honestly feels like this should be illegal.

#ifndef HAL_PATCHES_IMPLEMENT_SDMMC
#pragma weak SDMMC_CmdBlockLength
#pragma weak SDMMC_CmdReadSingleBlock
#pragma weak SDMMC_CmdReadMultiBlock
#pragma weak SDMMC_CmdWriteSingleBlock
#pragma weak SDMMC_CmdWriteMultiBlock
#pragma weak SDMMC_CmdEraseStartAdd
#pragma weak SDMMC_CmdSDEraseStartAdd
#pragma weak SDMMC_CmdEraseEndAdd
#pragma weak SDMMC_CmdSDEraseEndAdd
#pragma weak SDMMC_CmdErase
#pragma weak SDMMC_CmdStopTransfer
#pragma weak SDMMC_CmdSelDesel
#pragma weak SDMMC_CmdGoIdleState
#pragma weak SDMMC_CmdOperCond
#pragma weak SDMMC_CmdAppCommand
#pragma weak SDMMC_CmdAppOperCommand
#pragma weak SDMMC_CmdBusWidth
#pragma weak SDMMC_CmdSendSCR
#pragma weak SDMMC_CmdSendCID
#pragma weak SDMMC_CmdSendCSD
#pragma weak SDMMC_CmdSendEXTCSD
#pragma weak SDMMC_CmdSetRelAdd
#pragma weak SDMMC_CmdSendStatus
#pragma weak SDMMC_CmdStatusRegister
#pragma weak SDMMC_CmdOpCondition
#pragma weak SDMMC_CmdSwitch
#endif
