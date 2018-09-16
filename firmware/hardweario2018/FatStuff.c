#include "FatStuff.h"

USB_ClassInfo_MS_Device_t Disk_MS_Interface =
    {
        .Config =
            {
                .InterfaceNumber           = INTERFACE_ID_MassStorage,
                .DataINEndpoint            =
                    {
                        .Address           = MASS_STORAGE_IN_EPADDR,
                        .Size              = MASS_STORAGE_IO_EPSIZE,
                        .Banks             = 1,
                    },
                .DataOUTEndpoint           =
                    {
                        .Address           = MASS_STORAGE_OUT_EPADDR,
                        .Size              = MASS_STORAGE_IO_EPSIZE,
                        .Banks             = 1,
                    },
                .TotalLUNs                 = 1,
            },
    };

static FATFS DiskFATState;

FRESULT TestFat(void)
{
    FRESULT res;
    FIL File;
    res=f_mount(0, &DiskFATState);
    if (res!=FR_OK) return res;
    res=f_open(&File, "owner.txt", FA_OPEN_EXISTING);
    if ((res!=FR_OK) && (res!=FR_NO_FILE)) return res;
    f_close(&File);
    f_mount(0, NULL);
    return FR_OK;
}

FRESULT ReadFile(char *FileName, char *LineBuffer, uint16_t BytesRead, uint16_t* pBytesRead)
{
    FRESULT res;
    FIL File;
    res=f_mount(0, &DiskFATState);
    if (res!=FR_OK) return res;
    res=f_open(&File, FileName, FA_READ);
    if (res!=FR_OK) return res;
    if ((res=f_read(&File, LineBuffer, BytesRead, pBytesRead))!=FR_OK) return res;
    f_close(&File);
    f_mount(0, NULL);
    return FR_OK;
}

FRESULT WriteFile(char *FileName, char *LineBuffer, uint16_t BytesWritten, uint16_t* pBytesWritten)
{
    FRESULT res;
    FIL File;
    if ((res=f_mount(0, &DiskFATState))==FR_OK) {
        if (res=(f_open(&File, FileName, FA_OPEN_ALWAYS | FA_WRITE))==FR_OK) {
            if ((res=f_write(&File, LineBuffer, BytesWritten, pBytesWritten))!=FR_OK) return res;
            if ((res=f_sync(&File))!=FR_OK) return res;
            if ((res=f_close(&File))!=FR_OK) return res;
        } else return res;
        f_mount(0, NULL);
    } else return res;
}

void SetupHardware(void)
{
    /* Disable watchdog if enabled by bootloader/fuses */
    MCUSR &= ~(1 << WDRF);
    wdt_disable();

    /* Disable clock division */
    clock_prescale_set(clock_div_1);

    /* Hardware Initialization */
    Dataflash_Init();
    USB_Init();

    /* Check if the Dataflash is working, abort if not */
    if (!(DataflashManager_CheckDataflashOperation()))
    {
        for(;;);
    }

    /* Clear Dataflash sector protections, if enabled */
    DataflashManager_ResetDataflashProtections();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
    bool ConfigSuccess = true;
    ConfigSuccess &= MS_Device_ConfigureEndpoints(&Disk_MS_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
    MS_Device_ProcessControlRequest(&Disk_MS_Interface);
}

/** Mass Storage class driver callback function the reception of SCSI commands from the host, which must be processed.
 *
 *  \param[in] MSInterfaceInfo  Pointer to the Mass Storage class interface configuration structure being referenced
 */
bool CALLBACK_MS_Device_SCSICommandReceived(USB_ClassInfo_MS_Device_t* const MSInterfaceInfo)
{
    bool CommandSuccess;
    CommandSuccess = SCSI_DecodeSCSICommand(MSInterfaceInfo);

    return CommandSuccess;
}
