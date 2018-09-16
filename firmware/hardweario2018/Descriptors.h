#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

/* Includes: */
#include "LUFAConfig.h"
#include <avr/pgmspace.h>
#include <LUFA/LUFA/Drivers/USB/USB.h>
#include "FatStuff.h"

/* Macros: */
#define MASS_STORAGE_IN_EPADDR         (ENDPOINT_DIR_IN  | 3)
#define MASS_STORAGE_OUT_EPADDR        (ENDPOINT_DIR_OUT | 4)
#define MASS_STORAGE_IO_EPSIZE         64

/* Type Defines: */
typedef struct
{
    USB_Descriptor_Configuration_Header_t Config;
    // Mass Storage Interface
    USB_Descriptor_Interface_t            MS_Interface;
    USB_Descriptor_Endpoint_t             MS_DataInEndpoint;
    USB_Descriptor_Endpoint_t             MS_DataOutEndpoint;
} USB_Descriptor_Configuration_t;

enum InterfaceDescriptors_t
{
    INTERFACE_ID_MassStorage = 0,
};

enum StringDescriptors_t
{
    STRING_ID_Language     = 0,
    STRING_ID_Manufacturer = 1,
    STRING_ID_Product      = 2,
};

/* Function Prototypes: */
uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint16_t wIndex,
                                    const void** const DescriptorAddress)
                                    ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(3);

#endif
