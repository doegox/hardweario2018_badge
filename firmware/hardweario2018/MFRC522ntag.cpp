/*
* MFRC522.cpp - Library to use ARDUINO RFID MODULE KIT 13.56 MHZ WITH TAGS SPI W AND R BY COOQROBOT.
* NOTE: Please also check the comments in MFRC522.h - they provide useful hints and background information.
* Released into the public domain.
*/

#include <Arduino.h>
#include "MFRC522ntag.h"

/////////////////////////////////////////////////////////////////////////////////////
// Functions for setting up the Arduino
/////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522::PCD_WriteRegister(    PCD_Register reg,    ///< The register to write to. One of the PCD_Register enums.
                                    byte value            ///< The value to write.
                                ) {
    _spiClass->beginTransaction(_spiSettings);    // Set the settings to work with SPI bus
    digitalWrite(_chipSelectPin, LOW);        // Select slave
    _spiClass->transfer(reg);                        // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
    _spiClass->transfer(value);
    digitalWrite(_chipSelectPin, HIGH);        // Release slave again
    _spiClass->endTransaction(); // Stop using the SPI bus
} // End PCD_WriteRegister()

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522::PCD_WriteRegister(    PCD_Register reg,    ///< The register to write to. One of the PCD_Register enums.
                                    byte count,            ///< The number of bytes to write to the register
                                    byte *values        ///< The values to write. Byte array.
                                ) {
    _spiClass->beginTransaction(_spiSettings);    // Set the settings to work with SPI bus
    digitalWrite(_chipSelectPin, LOW);        // Select slave
    _spiClass->transfer(reg);                        // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
    for (byte index = 0; index < count; index++) {
        _spiClass->transfer(values[index]);
    }
    digitalWrite(_chipSelectPin, HIGH);        // Release slave again
    _spiClass->endTransaction(); // Stop using the SPI bus
} // End PCD_WriteRegister()

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
byte MFRC522::PCD_ReadRegister(    PCD_Register reg    ///< The register to read from. One of the PCD_Register enums.
                                ) {
    byte value;
    _spiClass->beginTransaction(_spiSettings);    // Set the settings to work with SPI bus
    digitalWrite(_chipSelectPin, LOW);            // Select slave
    _spiClass->transfer(0x80 | reg);                    // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    value = _spiClass->transfer(0);                    // Read the value back. Send 0 to stop reading.
    digitalWrite(_chipSelectPin, HIGH);            // Release slave again
    _spiClass->endTransaction(); // Stop using the SPI bus
    return value;
} // End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522::PCD_ReadRegister(    PCD_Register reg,    ///< The register to read from. One of the PCD_Register enums.
                                byte count,            ///< The number of bytes to read
                                byte *values,        ///< Byte array to store the values in.
                                byte rxAlign        ///< Only bit positions rxAlign..7 in values[0] are updated.
                                ) {
    if (count == 0) {
        return;
    }
    //oled.print(F("Reading "));     oled.print(count); oled.println(F(" bytes from register."));
    byte address = 0x80 | reg;                // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    byte index = 0;                            // Index in values array.
    _spiClass->beginTransaction(_spiSettings);    // Set the settings to work with SPI bus
    digitalWrite(_chipSelectPin, LOW);        // Select slave
    count--;                                // One read is performed outside of the loop
    _spiClass->transfer(address);                    // Tell MFRC522 which address we want to read
    if (rxAlign) {        // Only update bit positions rxAlign..7 in values[0]
        // Create bit mask for bit positions rxAlign..7
        byte mask = (0xFF << rxAlign) & 0xFF;
        // Read value and tell that we want to read the same address again.
        byte value = _spiClass->transfer(address);
        // Apply mask to both current value of values[0] and the new data in value.
        values[0] = (values[0] & ~mask) | (value & mask);
        index++;
    }
    while (index < count) {
        values[index] = _spiClass->transfer(address);    // Read value and tell that we want to read the same address again.
        index++;
    }
    values[index] = _spiClass->transfer(0);            // Read the final byte. Send 0 to stop reading.
    digitalWrite(_chipSelectPin, HIGH);            // Release slave again
    _spiClass->endTransaction(); // Stop using the SPI bus
} // End PCD_ReadRegister()

/**
 * Sets the bits given in mask in register reg.
 */
void MFRC522::PCD_SetRegisterBitMask(    PCD_Register reg,    ///< The register to update. One of the PCD_Register enums.
                                        byte mask            ///< The bits to set.
                                    ) { 
    byte tmp;
    tmp = PCD_ReadRegister(reg);
    PCD_WriteRegister(reg, tmp | mask);            // set bit mask
} // End PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
void MFRC522::PCD_ClearRegisterBitMask(    PCD_Register reg,    ///< The register to update. One of the PCD_Register enums.
                                        byte mask            ///< The bits to clear.
                                      ) {
    byte tmp;
    tmp = PCD_ReadRegister(reg);
    PCD_WriteRegister(reg, tmp & (~mask));        // clear bit mask
} // End PCD_ClearRegisterBitMask()


/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PCD_CalculateCRC(    byte *data,        ///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
                                                byte length,    ///< In: The number of bytes to transfer.
                                                byte *result    ///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
                     ) {
    PCD_WriteRegister(CommandReg, PCD_Idle);        // Stop any active command.
    PCD_WriteRegister(DivIrqReg, 0x04);                // Clear the CRCIRq interrupt request bit
    PCD_WriteRegister(FIFOLevelReg, 0x80);            // FlushBuffer = 1, FIFO initialization
    PCD_WriteRegister(FIFODataReg, length, data);    // Write data to the FIFO
    PCD_WriteRegister(CommandReg, PCD_CalcCRC);        // Start the calculation
    
    // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73μs.
    // TODO check/modify for other architectures than Arduino Uno 16bit

    // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
    for (uint16_t i = 5000; i > 0; i--) {
        // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
        byte n = PCD_ReadRegister(DivIrqReg);
        if (n & 0x04) {                                    // CRCIRq bit set - calculation done
            PCD_WriteRegister(CommandReg, PCD_Idle);    // Stop calculating CRC for new content in the FIFO.
            // Transfer the result from the registers to the result buffer
            result[0] = PCD_ReadRegister(CRCResultRegL);
            result[1] = PCD_ReadRegister(CRCResultRegH);
            return STATUS_OK;
        }
    }
    // 89ms passed and nothing happend. Communication with the MFRC522 might be down.
    return STATUS_TIMEOUT;
} // End PCD_CalculateCRC()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MFRC522 chip.
 */
void MFRC522::PCD_Init() {
    bool hardReset = false;

    // Set the chipSelectPin as digital output, do not select the slave yet
    pinMode(_chipSelectPin, OUTPUT);
    digitalWrite(_chipSelectPin, HIGH);
    
    // If a valid pin number has been set, pull device out of power down / reset state.
    if (_resetPowerDownPin != UNUSED_PIN) {
        // First set the resetPowerDownPin as digital input, to check the MFRC522 power down mode.
        pinMode(_resetPowerDownPin, INPUT);
    
        if (digitalRead(_resetPowerDownPin) == LOW) {    // The MFRC522 chip is in power down mode.
            pinMode(_resetPowerDownPin, OUTPUT);        // Now set the resetPowerDownPin as digital output.
            digitalWrite(_resetPowerDownPin, HIGH);        // Exit power down mode. This triggers a hard reset.
            // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
            delay(50);
            hardReset = true;
        }
    }

    if (!hardReset) { // Perform a soft reset if we haven't triggered a hard reset above.
        PCD_Reset();
    }
    
    // Reset baud rates
    PCD_WriteRegister(TxModeReg, 0x00);
    PCD_WriteRegister(RxModeReg, 0x00);
    // Reset ModWidthReg
    PCD_WriteRegister(ModWidthReg, 0x26);

    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
    PCD_WriteRegister(TModeReg, 0x80);            // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    PCD_WriteRegister(TPrescalerReg, 0xA9);        // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
    PCD_WriteRegister(TReloadRegH, 0x03);        // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    PCD_WriteRegister(TReloadRegL, 0xE8);
    
    PCD_WriteRegister(TxASKReg, 0x40);        // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    PCD_WriteRegister(ModeReg, 0x3D);        // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
    PCD_AntennaOn();                        // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()

/**
 * Initializes the MFRC522 chip.
 */
void MFRC522::PCD_Init(    byte chipSelectPin,        ///< Arduino pin connected to MFRC522's SPI slave select input (Pin 24, NSS, active low)
                        byte resetPowerDownPin    ///< Arduino pin connected to MFRC522's reset and power down input (Pin 6, NRSTPD, active low)
                    ) {
    _chipSelectPin = chipSelectPin;
    _resetPowerDownPin = resetPowerDownPin; 
    // Set the chipSelectPin as digital output, do not select the slave yet
    PCD_Init();
} // End PCD_Init()

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void MFRC522::PCD_Reset() {
    PCD_WriteRegister(CommandReg, PCD_SoftReset);    // Issue the SoftReset command.
    // The datasheet does not mention how long the SoftRest command takes to complete.
    // But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
    // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
    uint8_t count = 0;
    do {
        // Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
        delay(50);
    } while ((PCD_ReadRegister(CommandReg) & (1 << 4)) && (++count) < 3);
} // End PCD_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void MFRC522::PCD_AntennaOn() {
    byte value = PCD_ReadRegister(TxControlReg);
    if ((value & 0x03) != 0x03) {
        PCD_WriteRegister(TxControlReg, value | 0x03);
    }
} // End PCD_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void MFRC522::PCD_AntennaOff() {
    PCD_ClearRegisterBitMask(TxControlReg, 0x03);
} // End PCD_AntennaOff()

/**
 * Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 * 
 * @return Value of the RxGain, scrubbed to the 3 bits used.
 */
byte MFRC522::PCD_GetAntennaGain() {
    return PCD_ReadRegister(RFCfgReg) & (0x07<<4);
} // End PCD_GetAntennaGain()

/**
 * Set the MFRC522 Receiver Gain (RxGain) to value specified by given mask.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 */
void MFRC522::PCD_SetAntennaGain(byte mask) {
    if (PCD_GetAntennaGain() != mask) {                        // only bother if there is a change
        PCD_ClearRegisterBitMask(RFCfgReg, (0x07<<4));        // clear needed to allow 000 pattern
        PCD_SetRegisterBitMask(RFCfgReg, mask & (0x07<<4));    // only set RxGain[2:0] bits
    }
} // End PCD_SetAntennaGain()

/**
 * Performs a self-test of the MFRC522
 * See 16.1.1 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * 
 * @return Whether or not the test passed. Or false if no firmware reference is available.
 */
bool MFRC522::PCD_PerformSelfTest() {
    // This follows directly the steps outlined in 16.1.1
    // 1. Perform a soft reset.
    PCD_Reset();
    
    // 2. Clear the internal buffer by writing 25 bytes of 00h
    byte ZEROES[25] = {0x00};
    PCD_WriteRegister(FIFOLevelReg, 0x80);        // flush the FIFO buffer
    PCD_WriteRegister(FIFODataReg, 25, ZEROES);    // write 25 bytes of 00h to FIFO
    PCD_WriteRegister(CommandReg, PCD_Mem);        // transfer to internal buffer
    
    // 3. Enable self-test
    PCD_WriteRegister(AutoTestReg, 0x09);
    
    // 4. Write 00h to FIFO buffer
    PCD_WriteRegister(FIFODataReg, 0x00);
    
    // 5. Start self-test by issuing the CalcCRC command
    PCD_WriteRegister(CommandReg, PCD_CalcCRC);
    
    // 6. Wait for self-test to complete
    byte n;
    for (uint8_t i = 0; i < 0xFF; i++) {
        // The datasheet does not specify exact completion condition except
        // that FIFO buffer should contain 64 bytes.
        // While selftest is initiated by CalcCRC command
        // it behaves differently from normal CRC computation,
        // so one can't reliably use DivIrqReg to check for completion.
        // It is reported that some devices does not trigger CRCIRq flag
        // during selftest.
        n = PCD_ReadRegister(FIFOLevelReg);
        if (n >= 64) {
            break;
        }
    }
    PCD_WriteRegister(CommandReg, PCD_Idle);        // Stop calculating CRC for new content in the FIFO.
    
    // 7. Read out resulting 64 bytes from the FIFO buffer.
    byte result[64];
    PCD_ReadRegister(FIFODataReg, 64, result, 0);
    
    // Auto self-test done
    // Reset AutoTestReg register to be 0 again. Required for normal operation.
    PCD_WriteRegister(AutoTestReg, 0x00);
    
    // Determine firmware version (see section 9.3.4.8 in spec)
    byte version = PCD_ReadRegister(VersionReg);
    
    // Pick the appropriate reference values
    const byte *reference;
    switch (version) {
        case 0x88:    // Fudan Semiconductor FM17522 clone
            reference = FM17522_firmware_reference;
            break;
        case 0x90:    // Version 0.0
            reference = MFRC522_firmware_referenceV0_0;
            break;
        case 0x91:    // Version 1.0
            reference = MFRC522_firmware_referenceV1_0;
            break;
        case 0x92:    // Version 2.0
            reference = MFRC522_firmware_referenceV2_0;
            break;
        default:    // Unknown version
            return false; // abort test
    }
    
    // Verify that the results match up to our expectations
    for (uint8_t i = 0; i < 64; i++) {
        if (result[i] != pgm_read_byte(&(reference[i]))) {
            return false;
        }
    }
    
    // Test passed; all is good.
    return true;
} // End PCD_PerformSelfTest()

/////////////////////////////////////////////////////////////////////////////////////
// Power control
/////////////////////////////////////////////////////////////////////////////////////

//IMPORTANT NOTE!!!!
//Calling any other function that uses CommandReg will disable soft power down mode !!!
//For more details about power control, refer to the datasheet - page 33 (8.6)

void MFRC522::PCD_SoftPowerDown(){//Note : Only soft power down mode is available throught software
    byte val = PCD_ReadRegister(CommandReg); // Read state of the command register 
    val |= (1<<4);// set PowerDown bit ( bit 4 ) to 1 
    PCD_WriteRegister(CommandReg, val);//write new value to the command register
}

void MFRC522::PCD_SoftPowerUp(){
    byte val = PCD_ReadRegister(CommandReg); // Read state of the command register 
    val &= ~(1<<4);// set PowerDown bit ( bit 4 ) to 0 
    PCD_WriteRegister(CommandReg, val);//write new value to the command register
    // wait until PowerDown bit is cleared (this indicates end of wake up procedure) 
    const uint32_t timeout = (uint32_t)millis() + 500;// create timer for timeout (just in case) 
    
    while(millis()<=timeout){ // set timeout to 500 ms 
        val = PCD_ReadRegister(CommandReg);// Read state of the command register
        if(!(val & (1<<4))){ // if powerdown bit is 0 
            break;// wake up procedure is finished 
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PCD_TransceiveData(    byte *sendData,        ///< Pointer to the data to transfer to the FIFO.
                                                    byte sendLen,        ///< Number of bytes to transfer to the FIFO.
                                                    byte *backData,        ///< nullptr or pointer to buffer if data should be read back after executing the command.
                                                    byte *backLen,        ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                                                    byte *validBits,    ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
                                                    byte rxAlign,        ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                                    bool checkCRC        ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
                                 ) {
    byte waitIRq = 0x30;        // RxIRq and IdleIRq
    return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PCD_CommunicateWithPICC(    byte command,        ///< The command to execute. One of the PCD_Command enums.
                                                        byte waitIRq,        ///< The bits in the ComIrqReg register that signals successful completion of the command.
                                                        byte *sendData,        ///< Pointer to the data to transfer to the FIFO.
                                                        byte sendLen,        ///< Number of bytes to transfer to the FIFO.
                                                        byte *backData,        ///< nullptr or pointer to buffer if data should be read back after executing the command.
                                                        byte *backLen,        ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                                                        byte *validBits,    ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
                                                        byte rxAlign,        ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                                        bool checkCRC        ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
                                     ) {
    // Prepare values for BitFramingReg
    byte txLastBits = validBits ? *validBits : 0;
    byte bitFraming = (rxAlign << 4) + txLastBits;        // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
    
    PCD_WriteRegister(CommandReg, PCD_Idle);            // Stop any active command.
    PCD_WriteRegister(ComIrqReg, 0x7F);                    // Clear all seven interrupt request bits
    PCD_WriteRegister(FIFOLevelReg, 0x80);                // FlushBuffer = 1, FIFO initialization
    PCD_WriteRegister(FIFODataReg, sendLen, sendData);    // Write sendData to the FIFO
    PCD_WriteRegister(BitFramingReg, bitFraming);        // Bit adjustments
    PCD_WriteRegister(CommandReg, command);                // Execute the command
    if (command == PCD_Transceive) {
        PCD_SetRegisterBitMask(BitFramingReg, 0x80);    // StartSend=1, transmission of data starts
    }
    
    // Wait for the command to complete.
    // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
    // Each iteration of the do-while-loop takes 17.86μs.
    // TODO check/modify for other architectures than Arduino Uno 16bit
    uint16_t i;
    for (i = 2000; i > 0; i--) {
        byte n = PCD_ReadRegister(ComIrqReg);    // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
        if (n & waitIRq) {                    // One of the interrupts that signal success has been set.
            break;
        }
        if (n & 0x01) {                        // Timer interrupt - nothing received in 25ms
            return STATUS_TIMEOUT;
        }
    }
    // 35.7ms and nothing happend. Communication with the MFRC522 might be down.
    if (i == 0) {
        return STATUS_TIMEOUT;
    }
    
    // Stop now if any errors except collisions were detected.
    byte errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
    if (errorRegValue & 0x13) {     // BufferOvfl ParityErr ProtocolErr
        return STATUS_ERROR;
    }
  
    byte _validBits = 0;
    
    // If the caller wants data back, get it from the MFRC522.
    if (backData && backLen) {
        byte n = PCD_ReadRegister(FIFOLevelReg);    // Number of bytes in the FIFO
        if (n > *backLen) {
            return STATUS_NO_ROOM;
        }
        *backLen = n;                                            // Number of bytes returned
        PCD_ReadRegister(FIFODataReg, n, backData, rxAlign);    // Get received data from FIFO
        _validBits = PCD_ReadRegister(ControlReg) & 0x07;        // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
        if (validBits) {
            *validBits = _validBits;
        }
    }
    
    // Tell about collisions
    if (errorRegValue & 0x08) {        // CollErr
        return STATUS_COLLISION;
    }
    
    // Perform CRC_A validation if requested.
    if (backData && backLen && checkCRC) {
        // In this case a MIFARE Classic NAK is not OK.
        if (*backLen == 1 && _validBits == 4) {
            return STATUS_MIFARE_NACK;
        }
        // We need at least the CRC_A value and all 8 bits of the last byte must be received.
        if (*backLen < 2 || _validBits != 0) {
            return STATUS_CRC_WRONG;
        }
        // Verify CRC_A - do our own calculation and store the control in controlBuffer.
        byte controlBuffer[2];
        MFRC522::StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
        if (status != STATUS_OK) {
            return status;
        }
        if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
            return STATUS_CRC_WRONG;
        }
    }
    
    return STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PICC_RequestA(    byte *bufferATQA,    ///< The buffer to store the ATQA (Answer to request) in
                                            byte *bufferSize    ///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
                                        ) {
    return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PICC_WakeupA(    byte *bufferATQA,    ///< The buffer to store the ATQA (Answer to request) in
                                            byte *bufferSize    ///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
                                        ) {
    return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
MFRC522::StatusCode MFRC522::PICC_REQA_or_WUPA(    byte command,         ///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
                                                byte *bufferATQA,    ///< The buffer to store the ATQA (Answer to request) in
                                                byte *bufferSize    ///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
                                            ) {
    byte validBits;
    MFRC522::StatusCode status;
    
    if (bufferATQA == nullptr || *bufferSize < 2) {    // The ATQA response is 2 bytes long.
        return STATUS_NO_ROOM;
    }
    PCD_ClearRegisterBitMask(CollReg, 0x80);        // ValuesAfterColl=1 => Bits received after collision are cleared.
    validBits = 7;                                    // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
    status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits);
    if (status != STATUS_OK) {
        return status;
    }
    if (*bufferSize != 2 || validBits != 0) {        // ATQA must be exactly 16 bits.
        return STATUS_ERROR;
    }
    return STATUS_OK;
} // End PICC_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 *         - The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 *         - The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 *         UID size    Number of UID bytes        Cascade levels        Example of PICC
 *         ========    ===================        ==============        ===============
 *         single                 4                        1                MIFARE Classic
 *         double                 7                        2                MIFARE Ultralight
 *         triple                10                        3                Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PICC_Select(    Uid *uid,            ///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
                                            byte validBits        ///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
                                         ) {
    bool uidComplete;
    bool selectDone;
    bool useCascadeTag;
    byte cascadeLevel = 1;
    MFRC522::StatusCode result;
    byte count;
    byte index;
    byte uidIndex;                    // The first index in uid->uidByte[] that is used in the current Cascade Level.
    int8_t currentLevelKnownBits;        // The number of known UID bits in the current Cascade Level.
    byte buffer[9];                    // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
    byte bufferUsed;                // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
    byte rxAlign;                    // Used in BitFramingReg. Defines the bit position for the first bit received.
    byte txLastBits;                // Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
    byte *responseBuffer;
    byte responseLength;
    
    // Description of buffer structure:
    //        Byte 0: SEL                 Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
    //        Byte 1: NVB                    Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
    //        Byte 2: UID-data or CT        See explanation below. CT means Cascade Tag.
    //        Byte 3: UID-data
    //        Byte 4: UID-data
    //        Byte 5: UID-data
    //        Byte 6: BCC                    Block Check Character - XOR of bytes 2-5
    //        Byte 7: CRC_A
    //        Byte 8: CRC_A
    // The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
    //
    // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
    //        UID size    Cascade level    Byte2    Byte3    Byte4    Byte5
    //        ========    =============    =====    =====    =====    =====
    //         4 bytes        1            uid0    uid1    uid2    uid3
    //         7 bytes        1            CT        uid0    uid1    uid2
    //                        2            uid3    uid4    uid5    uid6
    //        10 bytes        1            CT        uid0    uid1    uid2
    //                        2            CT        uid3    uid4    uid5
    //                        3            uid6    uid7    uid8    uid9
    
    // Sanity checks
    if (validBits > 80) {
        return STATUS_INVALID;
    }
    
    // Prepare MFRC522
    PCD_ClearRegisterBitMask(CollReg, 0x80);        // ValuesAfterColl=1 => Bits received after collision are cleared.
    
    // Repeat Cascade Level loop until we have a complete UID.
    uidComplete = false;
    while (!uidComplete) {
        // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
        switch (cascadeLevel) {
            case 1:
                buffer[0] = PICC_CMD_SEL_CL1;
                uidIndex = 0;
                useCascadeTag = validBits && uid->size > 4;    // When we know that the UID has more than 4 bytes
                break;
            
            case 2:
                buffer[0] = PICC_CMD_SEL_CL2;
                uidIndex = 3;
                useCascadeTag = validBits && uid->size > 7;    // When we know that the UID has more than 7 bytes
                break;
            
            case 3:
                buffer[0] = PICC_CMD_SEL_CL3;
                uidIndex = 6;
                useCascadeTag = false;                        // Never used in CL3.
                break;
            
            default:
                return STATUS_INTERNAL_ERROR;
                break;
        }
        
        // How many UID bits are known in this Cascade Level?
        currentLevelKnownBits = validBits - (8 * uidIndex);
        if (currentLevelKnownBits < 0) {
            currentLevelKnownBits = 0;
        }
        // Copy the known bits from uid->uidByte[] to buffer[]
        index = 2; // destination index in buffer[]
        if (useCascadeTag) {
            buffer[index++] = PICC_CMD_CT;
        }
        byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
        if (bytesToCopy) {
            byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
            if (bytesToCopy > maxBytes) {
                bytesToCopy = maxBytes;
            }
            for (count = 0; count < bytesToCopy; count++) {
                buffer[index++] = uid->uidByte[uidIndex + count];
            }
        }
        // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
        if (useCascadeTag) {
            currentLevelKnownBits += 8;
        }
        
        // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
        selectDone = false;
        while (!selectDone) {
            // Find out how many bits and bytes to send and receive.
            if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
                //oled.print(F("SELECT: currentLevelKnownBits=")); oled.println(currentLevelKnownBits, DEC);
                buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
                // Calculate BCC - Block Check Character
                buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
                // Calculate CRC_A
                result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
                if (result != STATUS_OK) {
                    return result;
                }
                txLastBits        = 0; // 0 => All 8 bits are valid.
                bufferUsed        = 9;
                // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
                responseBuffer    = &buffer[6];
                responseLength    = 3;
            }
            else { // This is an ANTICOLLISION.
                //oled.print(F("ANTICOLLISION: currentLevelKnownBits=")); oled.println(currentLevelKnownBits, DEC);
                txLastBits        = currentLevelKnownBits % 8;
                count            = currentLevelKnownBits / 8;    // Number of whole bytes in the UID part.
                index            = 2 + count;                    // Number of whole bytes: SEL + NVB + UIDs
                buffer[1]        = (index << 4) + txLastBits;    // NVB - Number of Valid Bits
                bufferUsed        = index + (txLastBits ? 1 : 0);
                // Store response in the unused part of buffer
                responseBuffer    = &buffer[index];
                responseLength    = sizeof(buffer) - index;
            }
            
            // Set bit adjustments
            rxAlign = txLastBits;                                            // Having a separate variable is overkill. But it makes the next line easier to read.
            PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);    // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
            
            // Transmit the buffer and receive the response.
            result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
            if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
                byte valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
                if (valueOfCollReg & 0x20) { // CollPosNotValid
                    return STATUS_COLLISION; // Without a valid collision position we cannot continue
                }
                byte collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
                if (collisionPos == 0) {
                    collisionPos = 32;
                }
                if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
                    return STATUS_INTERNAL_ERROR;
                }
                // Choose the PICC with the bit set.
                count            = (collisionPos - 1) % 8; // The bit to modify
                index            = 1 + ((collisionPos - 1) / 8) + (count ? 1 : 0); // First byte is index 0.
                buffer[index]    |= (1 << count);
                //currentLevelKnownBits = collisionPos; // FIXME not used further, maybe bug
            }
            else if (result != STATUS_OK) {
                return result;
            }
            else { // STATUS_OK
                if (currentLevelKnownBits >= 32) { // This was a SELECT.
                    selectDone = true; // No more anticollision 
                    // We continue below outside the while.
                }
                else { // This was an ANTICOLLISION.
                    // We now have all 32 bits of the UID in this Cascade Level
                    currentLevelKnownBits = 32;
                    // Run loop again to do the SELECT.
                }
            }
        } // End of while (!selectDone)
        
        // We do not check the CBB - it was constructed by us above.
        
        // Copy the found UID bytes from buffer[] to uid->uidByte[]
        index            = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
        bytesToCopy        = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
        for (count = 0; count < bytesToCopy; count++) {
            uid->uidByte[uidIndex + count] = buffer[index++];
        }
        
        // Check response SAK (Select Acknowledge)
        if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
            return STATUS_ERROR;
        }
        // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
        result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
        if (result != STATUS_OK) {
            return result;
        }
        if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
            return STATUS_CRC_WRONG;
        }
        if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
            cascadeLevel++;
        }
        else {
            uidComplete = true;
            uid->sak = responseBuffer[0];
        }
    } // End of while (!uidComplete)
    
    // Set correct uid->size
    uid->size = 3 * cascadeLevel + 1;

    return STATUS_OK;
} // End PICC_Select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
MFRC522::StatusCode MFRC522::PICC_HaltA() {
    MFRC522::StatusCode result;
    byte buffer[4];
    
    // Build command buffer
    buffer[0] = PICC_CMD_HLTA;
    buffer[1] = 0;
    // Calculate CRC_A
    result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
    if (result != STATUS_OK) {
        return result;
    }
    
    // Send the command.
    // The standard says:
    //        If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
    //        HLTA command, this response shall be interpreted as 'not acknowledge'.
    // We interpret that this way: Only STATUS_TIMEOUT is a success.
    result = PCD_TransceiveData(buffer, sizeof(buffer), nullptr, 0);
    if (result == STATUS_TIMEOUT) {
        return STATUS_OK;
    }
    if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
        return STATUS_ERROR;
    }
    return result;
} // End PICC_HaltA()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with MIFARE PICCs
/////////////////////////////////////////////////////////////////////////////////////


/**
 * Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.
 * 
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 * 
 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
 * The MF0ICU1 returns a NAK for higher addresses.
 * The MF0ICU1 responds to the READ command by sending 16 bytes starting from the page address defined by the command argument.
 * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
 * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
 * 
 * The buffer must be at least 18 bytes because a CRC_A is also returned.
 * Checks the CRC_A before returning STATUS_OK.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::MIFARE_Read(    byte blockAddr,     ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
                                            byte *buffer,        ///< The buffer to store the data in
                                            byte *bufferSize    ///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
                                        ) {
    MFRC522::StatusCode result;
    
    // Sanity check
    if (buffer == nullptr || *bufferSize < 18) {
        return STATUS_NO_ROOM;
    }
    
    // Build command buffer
    buffer[0] = PICC_CMD_MF_READ;
    buffer[1] = blockAddr;
    // Calculate CRC_A
    result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
    if (result != STATUS_OK) {
        return result;
    }
    
    // Transmit the buffer and receive the response, validate CRC_A.
    return PCD_TransceiveData(buffer, 4, buffer, bufferSize, nullptr, 0, true);
} // End MIFARE_Read()



/////////////////////////////////////////////////////////////////////////////////////
// Support functions
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Wrapper for MIFARE protocol communication.
 * Adds CRC_A, executes the Transceive command and checks that the response is MF_ACK or a timeout.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522::StatusCode MFRC522::PCD_MIFARE_Transceive(    byte *sendData,        ///< Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
                                                    byte sendLen,        ///< Number of bytes in sendData.
                                                    bool acceptTimeout    ///< True => A timeout is also success
                                                ) {
    MFRC522::StatusCode result;
    byte cmdBuffer[18]; // We need room for 16 bytes data and 2 bytes CRC_A.
    
    // Sanity check
    if (sendData == nullptr || sendLen > 16) {
        return STATUS_INVALID;
    }
    
    // Copy sendData[] to cmdBuffer[] and add CRC_A
    memcpy(cmdBuffer, sendData, sendLen);
    result = PCD_CalculateCRC(cmdBuffer, sendLen, &cmdBuffer[sendLen]);
    if (result != STATUS_OK) { 
        return result;
    }
    sendLen += 2;
    
    // Transceive the data, store the reply in cmdBuffer[]
    byte waitIRq = 0x30;        // RxIRq and IdleIRq
    byte cmdBufferSize = sizeof(cmdBuffer);
    byte validBits = 0;
    result = PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, sendLen, cmdBuffer, &cmdBufferSize, &validBits);
    if (acceptTimeout && result == STATUS_TIMEOUT) {
        return STATUS_OK;
    }
    if (result != STATUS_OK) {
        return result;
    }
    // The PICC must reply with a 4 bit ACK
    if (cmdBufferSize != 1 || validBits != 4) {
        return STATUS_ERROR;
    }
    if (cmdBuffer[0] != MF_ACK) {
        return STATUS_MIFARE_NACK;
    }
    return STATUS_OK;
} // End PCD_MIFARE_Transceive()

/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 * 
 * @return PICC_Type
 */
MFRC522::PICC_Type MFRC522::PICC_GetType(byte sak        ///< The SAK byte returned from PICC_Select().
                                        ) {
    // http://www.nxp.com/documents/application_note/AN10833.pdf 
    // 3.2 Coding of Select Acknowledge (SAK)
    // ignore 8-bit (iso14443 starts with LSBit = bit 1)
    // fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
    sak &= 0x7F;
    switch (sak) {
        case 0x04:    return PICC_TYPE_NOT_COMPLETE;    // UID not complete
        case 0x09:    return PICC_TYPE_MIFARE_MINI;
        case 0x08:    return PICC_TYPE_MIFARE_1K;
        case 0x18:    return PICC_TYPE_MIFARE_4K;
        case 0x00:    return PICC_TYPE_MIFARE_UL;
        case 0x10:
        case 0x11:    return PICC_TYPE_MIFARE_PLUS;
        case 0x01:    return PICC_TYPE_TNP3XXX;
        case 0x20:    return PICC_TYPE_ISO_14443_4;
        case 0x40:    return PICC_TYPE_ISO_18092;
        default:    return PICC_TYPE_UNKNOWN;
    }
} // End PICC_GetType()


/**
 * Dumps memory contents of a NTAG PICC.
 * Function authored by @doegox for Hardwear.io 2018 badge
 */
int MFRC522::PICC_DumpNtagToOled(SSD1306AsciiWire oled, byte* vcard, uint16_t* ppayloadlength, byte* name, uint16_t* pnamelength) {
    MFRC522::StatusCode status;
    byte byteCount;
    byte buffer[18];
    uint16_t offset = 0;
    uint32_t payloadlength;
    byte namelength=0;
    byte i;
    byte capacity = 0;
    
/*
   *   cc p0x3      ndef mem size
   * 213: byte2=12h 144=byte2*8 => max ndef: 140, max payload: 125 (short)
   * 215: byte2=3eh 496=byte2*8 => max ndef: 492, max payload: 474 (long)
   * 216: byte2=6dh 872=byte2*8 => max ndef: 868, max payload: 850 (long)
   * 
   * We don't implement a full NDEF parser, just 1 NDEF Message with 1 vCARD
   * 
   * Short NDEF Message TLV:
   * 03 LL
   *    ndef message length (< 256)
   * Long NDEF Message TLV:
   * 03 FF LL LL
   *       ndef message length
   * Terminator TLV:
   * FE
   * 
   * Short NDEF message:
   * D2 0C LL text/x-vCard
   *&10=01: short
   *&07=02: media type
   *    type length
   *       payload length < 257
   * 
   * Long NDEF message:
   * C2 0C LL LL LL LL text/x-vCard
   *&10=00: long
   *&07=02: media type
   *    type length
   *       payload length
   * 
 */
    byteCount = sizeof(buffer);
    status = MIFARE_Read(0x03, buffer, &byteCount);
    if (status != STATUS_OK) {
        oled.println(F("Tag read failed."));
        return -1;
    }
    if (buffer[0] != 0xE1) { // NDEF magic
        oled.println(F("Not NDEF formatted."));
        return -1;
    }
    if ((buffer[1] & 0xF0) != 0x10) { // NDEF v1.x
        oled.println(F("Wrong NDEF version."));
        return -1;
    }
    capacity = buffer[2];
    if (capacity > 0x6D) { // phishy
        oled.println(F("Phishy NDEF."));
        return -1;
    }
    if ((buffer[3] & 0xF0) != 0x00) { // Read access
        oled.println(F("Read access: disabled."));
        return -1;
    }

    offset = 4;
    // 00=null, 01=lock, 02=memory, fe=terminator
    if (buffer[offset++] != 0x03) { // NDEF Message TLV
        oled.println(F("Not a NDEF Message."));
        return -1;
    }

    uint16_t ndeflength;
    if ((ndeflength = buffer[offset++]) == 0xFF) {
        ndeflength = (((uint16_t) buffer[offset++]) << 8) + buffer[offset++];
    }
    if ((buffer[offset] & 0xef) != 0xc2) { // Only 1 NDEF TLLTV media type record
        oled.println(F("Unsupported NDEF record."));
        return -1;
    }
    uint8_t mimelength = buffer[offset+1];
    
    if ((mimelength != 12)&&(mimelength != 10)) { // vCARD type should be 10 or 12 bytes long
        oled.println(F("Unsupported MIME type."));
        return -1;
    }
    if (buffer[offset] & 0x10) {
        payloadlength = buffer[offset+2];
        offset += 3;
    }
    else {
        payloadlength = (buffer[offset+2] << 24) + (buffer[offset+3] << 16) + (buffer[offset+4] << 8) + buffer[offset+5];
        offset += 6;
    }

    byte page = offset >> 2;
    offset &= 0x3;
    status = MIFARE_Read(page + 3, buffer, &byteCount);

    if (status != STATUS_OK) {
        oled.println(F("Tag read failed."));
        return -1;
    }

    bool mime_validated=false;
    if ((buffer[offset] == 't') &&
        (buffer[offset+1] == 'e') &&
        (buffer[offset+2] == 'x') &&
        (buffer[offset+3] == 't') &&
        (buffer[offset+4] == '/') &&
        (buffer[offset+5] == 'x') &&
        (buffer[offset+6] == '-') &&
        (buffer[offset+7] == 'v') &&
        ((buffer[offset+8] == 'C') || (buffer[offset+8] == 'c')) &&
        (buffer[offset+9] == 'a') &&
        (buffer[offset+10] == 'r') &&
        (buffer[offset+11] == 'd')) {
        mime_validated=true;
    }
    if ((buffer[offset] == 't') &&
        (buffer[offset+1] == 'e') &&
        (buffer[offset+2] == 'x') &&
        (buffer[offset+3] == 't') &&
        (buffer[offset+4] == '/') &&
        (buffer[offset+5] == 'v') &&
        ((buffer[offset+6] == 'C') || (buffer[offset+6] == 'c')) &&
        (buffer[offset+7] == 'a') &&
        (buffer[offset+8] == 'r') &&
        (buffer[offset+9] == 'd')) {
        mime_validated=true;
    }
    if (!mime_validated) {
        oled.println(F("Unsupported MIME type."));
        return -1;
    }
    offset+=mimelength;

    page += offset >> 2;
    offset &= 0x3;
    uint16_t payloadoffset = 0;

    for (;payloadoffset < payloadlength + 1;page+=4) {
        status = MIFARE_Read(page + 3, buffer, &byteCount);
        if (status != STATUS_OK) {
            oled.println(F("Tag read failed."));
            return -1;
        }
        for (uint16_t i = offset; i < 16; i++) {
            vcard[payloadoffset++] = buffer[i];
        }
        offset=0;
    }

    // Check terminator
    if (vcard[payloadlength] != 0xfe) {
        oled.println(F("NDEF Terminator not found."));
        return -1;
    }
    vcard[payloadlength] = 0;

///////////////////////////////////////////////
// vCard:  vcard[payloadlength]
///////////////////////////////////////////////

    for (uint16_t i = 0; i < payloadlength - 5;) {
        if ((vcard[i++] == 0x0a) &&
            (vcard[i++] == 'F') &&
            (vcard[i++] == 'N') &&
            (vcard[i++] == ':')) {
                for(;;) {
                    byte c = vcard[i++];
                    if ((c == 0x0d) || (c == 0x0a)) {
                        break;
                    }
                    name[namelength++] = c;
                    if (i >= payloadlength)
                        break;
                }
            break;
        }
    }

///////////////////////////////////////////////
// Name:  name[namelength]
///////////////////////////////////////////////

    *ppayloadlength = payloadlength;
    *pnamelength = namelength;
    return 0;

} // End PICC_DumpNtagToOled()


/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 * 
 * @return bool
 */
bool MFRC522::PICC_IsNewCardPresent() {
    byte bufferATQA[2];
    byte bufferSize = sizeof(bufferATQA);

    // Reset baud rates
    PCD_WriteRegister(TxModeReg, 0x00);
    PCD_WriteRegister(RxModeReg, 0x00);
    // Reset ModWidthReg
    PCD_WriteRegister(ModWidthReg, 0x26);

    MFRC522::StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
    return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 * 
 * @return bool
 */
bool MFRC522::PICC_ReadCardSerial() {
    MFRC522::StatusCode result = PICC_Select(&uid);
    return (result == STATUS_OK);
} // End 
