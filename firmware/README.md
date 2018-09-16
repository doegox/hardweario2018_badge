# Setup of a toolchain

## Install LUFA in Arduino

The firmware is using both Arduino libraries and LUFA libraries. LUFA wasn't meant to be an Arduino library so the setup is a bit hectic.

We'll follow https://github.com/Palatis/Arduino-Lufa, with some differences. Using arduino-1.8.3.

Go to `~/Arduino/libraries` and install LUFA:

```
git clone --recursive https://github.com/Palatis/Arduino-Lufa.git LUFA
```

Then we need to "activate" the installation, which basically means patching the core Arduino files to replace Arduino USB stack by LUFA stack.

A helper script is available:

```
python3 LUFA/activate.py
```

But in our setup, we had to patch the core files in `~/.arduino15/packages/arduino/hardware/avr/1.6.21/`. See our helper scripts `activateLUFAinPrivateDir.py` and `deactivateLUFAinPrivateDir.py` to revert it.

## Bring Dataflash support to Leonardo

```
cd ~/Arduino/libraries/LUFA/LUFA/
patch -p0 < path_to/Dataflash4leonardo.diff
```

## Other project dependencies

Install SSD1306Ascii, an OLED small library (because Adafruit_SSD1306 + Adagruit_GFX is way too large):

```
cd ~/Arduino/libraries
git clone https://github.com/greiman/SSD1306Ascii.git
```

NFC library: based on https://github.com/miguelbalboa/rfid.git but heavily modified and shrunk. It is now part of the badge source files (cf `MFRC522ntag.*` files).

SCSI/Dataflash/diskio/FAT are also already present in the sources.

# Project compilation

Launch `arduino` IDE.

* File/Open : `hardweario2018/hardweario2018.ino`
* Tools/Board : SparkFun Pro Micro
* Tools/Processor: ATmega32U4 (5V, 16MHz)

Compile project

To upload: start "Upload" on the IDE, then quickly shortcut GND & RST PINs on the ProMicro.

In case of troubles flashing it, shortcut *twice* quickly RST & GND to enter bootloader mode for a longer time.

# First badge use

On a new badge, the EEPROM must be formatted as FAT:

`sudo ./format_badge_fat.sh`

Unplug and replug when requested by the script.
