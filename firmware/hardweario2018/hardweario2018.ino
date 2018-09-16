#include "LUFAConfig.h"
#include <LUFA.h>
#include "FatStuff.h"

#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#define I2C_ADDRESS 0x3C
#define RST_PIN -1
SSD1306AsciiWire oled;

#include <SPI.h>
#include "MFRC522ntag.h"
MFRC522 mfrc522;
constexpr uint8_t SS_PIN = 10;

#define VERSION "v1.2"

//#define BADGEFLAG "FLAG{NfcHangMan}"
#define BADGEFLAG "GNBC~HakAkek@oam"
#define BADGEFLAGSIZE 16

#define FT(i,j,x);  const char FT_##i##j[] PROGMEM = {x};
#define FP(i,j)     (__FlashStringHelper*)(FT_##i##j)

int16_t facedisplayindex=0;
bool facedisplayed=false;
#define FACES 4
FT(1,A,"   *   *")
FT(1,B,"     //")
FT(2,A,"  |.   .|")
FT(2,B,"     ^")
FT(3,A,"   *   *")
FT(3,B,"     ==")
FT(4,A,"    \x7f \x7f")
FT(4,B,"     <>")
__FlashStringHelper* FTTA[]={FP(1,A), FP(2,A), FP(3,A), FP(4,A)};
__FlashStringHelper* FTTB[]={FP(1,B), FP(2,B), FP(3,B), FP(4,B)};

FRESULT fatstatus=FR_OK;

void setup()
{
    SetupHardware();
    fatstatus=TestFat();
    GlobalInterruptEnable();

    pinMode(6, INPUT);
    pinMode(9, INPUT);

    Wire.begin();
    Wire.setClock(400000L);

    oled.begin(&Adafruit128x64, I2C_ADDRESS);
    oled.setFont(Adafruit5x7);

    oled.clear();
    oled.set2X();
    oled.println(F("Booting..."));
    oled.set1X();
    oled.println(F(VERSION));
    if (fatstatus == FR_NO_FILESYSTEM) {
        oled.println(F("No FAT, format me!"));
    } else if (fatstatus != FR_OK) {
        oled.println(F("Filesystem error"));
    }
    SPI.begin();
}

// We've to wait to see if a USB host is enumerating us or not
uint32_t counter=600000;
uint8_t setup_done=0; // 1 = NFC, 2 = USB MS

#define MAXNAMELENGTH 64
char ownername[MAXNAMELENGTH+1]; // will be null-terminated
byte ownernamelength=0;
uint8_t ownernameoffs[4]= {0,0,0,0};

void fill_ownername(byte* name, uint16_t namelength) {
    uint8_t l=0;
    ownernameoffs[l++]=0;
    if (namelength >= sizeof(ownername)) {
        namelength = sizeof(ownername)-1;
    }
    for (uint8_t i=0; i<namelength; i++) {
        if ((name[i]==' ')||(name[i]=='\n')) {
            ownername[i]='\0';
            if ((l<sizeof(ownernameoffs))&&(i<namelength-1)) {
                ownernameoffs[l++]=i+1;
            }
        } else {
            ownername[i]=name[i];
        }
    }
    ownername[namelength]='\0';
    ownernamelength=namelength+1;
    for (; l<sizeof(ownernameoffs); l++) {
        ownernameoffs[l++]=0;
    }
}

void display_owner() {
    oled.clear();
    oled.set2X();
    if (ownernameoffs[3]==0) {
        oled.println("");
    }
    if (ownernamelength>0) {
        uint8_t n;
        uint8_t oldn=0;
        for(n=0; (n<ownernamelength)&&(ownername[n]!='\0'); n++);
        oled.set1X();
        for(uint8_t i=0; i<(11-n); i++) {
            oled.print(' ');
        }
        oled.set2X();
        oled.println(ownername);
        for (uint8_t l=1; l<sizeof(ownernameoffs); l++) {
            if (ownernameoffs[l]==0) break;
            oldn+=n+1;
            for(n=0; (n<(ownernamelength-oldn))&&((ownername+ownernameoffs[l])[n]!='\0'); n++);
            oled.set1X();
            for(uint8_t i=0; i<(11-n); i++) {
                oled.print(' ');
            }
            oled.set2X();
            oled.println(ownername+ownernameoffs[l]);
        }
    } else {
        oled.println(F(" adopt me"));
    }
    oled.set1X();
}

void display_face() {
    if (!facedisplayed) {
        if (facedisplayindex==0) {
            display_owner();
        } else {
            oled.clear();
            oled.set2X();
            oled.println(FTTA[facedisplayindex-1]);
            oled.println();
            oled.println(FTTB[facedisplayindex-1]);
            oled.set1X();
        }
        facedisplayed=true;
    }
}

void setup_nfcmode()
{
    byte name[MAXNAMELENGTH];
    uint16_t namelength=0;
    if (fatstatus == FR_OK) {
        oled.clear();
    }
    oled.println(F("NFC mode"));
    mfrc522.PCD_Init(SS_PIN, RST_PIN);
    // Optionally tune antenna gain by software
    // RxGain_18dB, RxGain_23dB, RxGain_33dB, RxGain_38dB, RxGain_43dB, RxGain_48dB
    // mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_33dB);
    delay(200);
    setup_done=1;
    if ((ReadFile("owner.txt", name, sizeof(name)-1, &namelength)==FR_OK) && (namelength>0)) {
        fill_ownername(name, namelength);
    }
}

void setup_msmode()
{
    if (fatstatus == FR_OK) {
        oled.clear();
    }
    oled.println(F("Mass Storage mode"));
    setup_done=2;
}

void poll_for_tag(void)
{
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        facedisplayed=false;
        oled.clear();
        oled.println(F("Reading tag..."));
        MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
        if (piccType ==MFRC522::PICC_TYPE_MIFARE_UL) {
            byte vcard[866]; // max payload = 850 + one more read 16
            uint16_t payloadlength;
            byte name[MAXNAMELENGTH];
            uint16_t namelength=sizeof(name);
            payloadlength=sizeof(vcard);
            if (mfrc522.PICC_DumpNtagToOled(oled, vcard, &payloadlength, name, &namelength) < 0) {
                oled.println(F(""));
                oled.println(F(""));
                oled.set2X();
                oled.println(F("   Abort!"));
                oled.set1X();
            } else {
                oled.clear();
                if ((namelength>=4) && (name[0]=='F') && (name[1]=='L') && (name[2]=='A') && (name[3]=='G')) {
                    // We don't save "FLAG..." vcards to avoid DoS
                    char* flag=BADGEFLAG;
                    for (uint16_t i = 0; (i < BADGEFLAGSIZE) && (i < namelength); i++) {
                        if(name[i]!=(flag[i]^(i+1))) {
                            oled.println(F("Incorrect flag chars!"));
                            oled.println(F("Masking them with \"*\""));
                            break;
                        }
                    }
                    if (namelength < BADGEFLAGSIZE) {
                        oled.println("Incomplete flag!");
                    }
                    oled.println("");
                    for (uint16_t i = 0; (i < BADGEFLAGSIZE) && (i < namelength); i++) {
                        if((i<namelength) && (name[i]==(flag[i]^(i+1)))) {
                            oled.write(name[i]);
                        } else {
                            oled.write('*');
                        }
                    }
                    oled.println();
                } else {
                    oled.print(F("Hi "));
                    for (uint16_t i = 0; i < namelength; i++) {
                        oled.write(name[i]);
                    }
                    oled.println();
                    if (fatstatus==FR_OK) {
                        uint16_t payloadlengthwritten=0;
                        // create 8.3 filename from UID
                        // UID is 7 bytes: 04aabbccddeeff with aabbccddeeff serial in little endian -> keep ddccbbaa as filename
                        char VcardFileName[13];
                        uint8_t iVcardFileName=0;
                        for (uint8_t i=4; i>0; i--) {
                            VcardFileName[iVcardFileName++]="0123456789ABCDEF"[mfrc522.uid.uidByte[i] >> 4];
                            VcardFileName[iVcardFileName++]="0123456789ABCDEF"[mfrc522.uid.uidByte[i] & 0x0F];
                        }
                        VcardFileName[iVcardFileName++]='.';
                        VcardFileName[iVcardFileName++]='v';
                        VcardFileName[iVcardFileName++]='c';
                        VcardFileName[iVcardFileName++]='f';
                        VcardFileName[iVcardFileName++]='\0';
                        oled.println(F("Storing vCard in"));
                        oled.println(VcardFileName);
                        WriteFile(VcardFileName, vcard, payloadlength, &payloadlengthwritten);
                        if (payloadlength != payloadlengthwritten) {
                            oled.print(F("Error "));
                            oled.print(payloadlengthwritten);
                            oled.print(F(" b written"));
                        }
                        if (ownernamelength==0) {
                            uint16_t foo;
                            WriteFile("owner.txt", name, namelength, &foo);
                        }
                        oled.println(F(""));
                        oled.set2X();
                        oled.println(F("Thank you!"));
                        oled.set1X();
                    } else {
                        oled.println(F("No FAT, can't store!"));
                    }
                    if (ownernamelength==0) {
                        fill_ownername(name, namelength);
                    }
                }
            }
        } else {
            oled.println(F("Card not supported."));
            oled.println(F(""));
            oled.set2X();
            oled.println(F("   Abort!"));
            oled.set1X();
        }
        // Halt PICC
        mfrc522.PICC_HaltA();
        delay(2000);
    }
}

void loop_nfcmode()
{
    poll_for_tag();
    if (digitalRead(6) == HIGH){
        delay(100);
        if (digitalRead(6) == HIGH)
        facedisplayindex++;
        if (facedisplayindex>FACES) {
          facedisplayindex=0;
        }
        facedisplayed=false;
    }
    if (digitalRead(9) == HIGH){
        delay(100);
        if (digitalRead(9) == HIGH)
        facedisplayindex--;
        if (facedisplayindex<0) {
          facedisplayindex=FACES;
        }
        facedisplayed=false;
    }
    display_face();
}

void loop_msmode()
{}

void loop()
{
    if (setup_done!=1) {
        MS_Device_USBTask(&Disk_MS_Interface);
        USB_USBTask();
    }
    if ((counter>0) && (USB_DeviceState != DEVICE_STATE_Configured)) {
        counter--;
        return;
    }
    if (setup_done==0) {
        // Setup after wait loop
        if (USB_DeviceState != DEVICE_STATE_Configured) {
            setup_nfcmode();
        } else {
            setup_msmode();
        }
    }
    if (setup_done==1) {
        loop_nfcmode();
    } else if (setup_done==2) {
        loop_msmode();
    }
}

