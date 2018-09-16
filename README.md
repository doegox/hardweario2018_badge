# Hardwear.io 2018 badge

The Hardwear.io 2018 badge is a reader that will read NFC business cards and store the details in its 512kB memory.

When connected to a PC, it will be seen as a flash drive with a FAT file-system and you will be able to retrieve the business cards as vCard files.

It's designed to have a second life: just unplug the parts you want to reuse in your own projects!

## Modes

There is a 3-way switch on the back side.

* UP: NFC mode, powered by the battery
* MIDDLE: off, or USB mode if connected to PC
* DOWN: off, or USB mode + charging

Always power off the badge when switching modes, so never leave the switch UP when plugging to USB. Nothing bad will happen but modes will be confused till power-cycled.

## Initialization

When empty and set in NFC mode (UP), it will display "adopt me".
Just touch it with your tag. It will be yours and display your name.

If you want to change the display and e.g. add your Twitter handle or fix a bad char, set it to USB mode and edit "owner.txt".
When back to NFC mode, the first four words will be displayed on the four OLED lines.

## NFC mode

Switch in UP position.

The firmware can read NFC Forum Type 2 tags (NTAG) configured as NFC business cards.

To create or edit such NFC business cards, you can use e.g. NXP TagWriter or NFC Tools (since firmware v1.3) Android applications.

A NFC business card contains a standard [vCard](https://en.wikipedia.org/wiki/VCard), so the tag will store vCards as separate files.

If you read several times the same tag (possibly edited), it will store only the last version.

## USB mode

WARNING: USB mode seems to fail under Windows. It works properly under Linux and MAC OSX.

Switch in MIDDLE or DOWN position.

The badge will be mounted as a flash drive.
You can edit its content (remove vCards, edit owner.txt,...), just remember to unmount it cleanly to save your changes.

To charge the battery, set switch in DOWN position. At the bottom of the badge, D1 turns on when charging and D2 turns on when charged.

## Challenge

For those participating to our hardware CTF, a challenge is hidden in the badge. You'll have to meet Mr FLAG... or maybe you'll find a way without his business card...

## Firmware

Read [firmware] instructions.

* v1.2: initial version of the badges distributed at Hardwear.io 2018
* v1.3: accepts more variants of vCard MIME types, contains more faces

## Hardware upgrade

Quoting [https://github.com/miguelbalboa/rfid/blob/master/README.rst] :

*Some boards bought from chinese manufactures do not use the best components and this can affect the detection of different types of tag/card. In some of these boards, the L1 and L2 inductors do not have a high enough current so the signal generated is not enough to get Ultralight C and NTAG203 tags to work, replacing those with same inductance (2.2uH) but higher operating current inductors should make things work smoothly. Also, in some of those boards the harmonic and matching circuit needs to be tuned, for this replace C4 and C5 with 33pf capacitors and you are all set. (Source: [Mikro Elektronika](https://forum.mikroe.com/viewtopic.php?f=147&t=64203))*

We confirm changing L1, L2, C4 & C5 beefs up the performances and allows reading all tags including UltralightC.

## Reusing

The badge contains several standard elements you can harvest for your own projects:

* OLED SSD1306 128x64
  * info on [https://www.adafruit.com/product/938] (but only I2C)
* MFRC522 board
  * info on [https://playground.arduino.cc/Learning/MFRC522]
* TXB0108 level shifter board
  * info on [https://www.adafruit.com/product/395]
* Arduino Pro Micro Atmega32U4 5V 16MHz
  * info on [https://www.sparkfun.com/products/12640]

## Credits

Designed by:

* [@doegox](https://twitter.com/doegox)
* [@PapaZours](https://twitter.com/PapaZours)
* [@hei5enbrg](https://twitter.com/hei5enbrg)
