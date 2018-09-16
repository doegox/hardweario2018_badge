#ifndef _FATSTUFF_H_
#define _FATSTUFF_H_

/* Includes: */
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "Descriptors.h"
#include "SCSI.h"
#include "DataflashManager.h"
#include "ff.h"
#include <LUFA/LUFA/Drivers/USB/USB.h>
#include <LUFA/LUFA/Platform/Platform.h>

/** Indicates if the disk is write protected or not. */
#define DISK_READ_ONLY false

#ifdef __cplusplus
extern "C"{
#endif
/* Function Prototypes: */
void SetupHardware(void);
FRESULT TestFat(void);
FRESULT ReadFile(char *FileName, char *LineBuffer, uint16_t BytesRead, uint16_t* pBytesRead);
FRESULT WriteFile(char *FileName, char *LineBuffer, uint16_t BytesWritten, uint16_t* pBytesWritten);

void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);

bool CALLBACK_MS_Device_SCSICommandReceived(USB_ClassInfo_MS_Device_t* const MSInterfaceInfo);
#ifdef __cplusplus
} // extern "C"
#endif
/* Data Types: */
extern USB_ClassInfo_MS_Device_t Disk_MS_Interface;
#endif

