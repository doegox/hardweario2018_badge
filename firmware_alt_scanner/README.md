
Firmware based on https://github.com/cbm80amiga/RFID_Scanner_OLED commit `af77cd8`
with the following modifications for getting it working with Hardwear.io badge and remove compilation errors and warnings:

* Pragmas for ProMicro 5V
* `#define RST_PIN -1`
* `char*` => `const char*`
* fonts: negative x as uint8_t => `256-x`
* local copy of modified OLEDSoftI2C_SSD1306 (from https://github.com/cbm80amiga/OLEDSoftI2C_SSD1306 commit `af77cd8`)
  * `char*` => `const char*`
  * `#define USEHW 1`
  * all `rcall` => `call`
  * all `rjmp` => `jmp`
  * all `[...] __attribute__ ((noinline)) [...] asm("[...]");` => append `__attribute__ ((used))`
* local copy of modified MFRC522 (from https://github.com/mdxs/MFRC522 commit `4dd8522`)

Compiled firmware available in ../firmware_releases
