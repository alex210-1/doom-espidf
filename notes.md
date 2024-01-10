# references

Buid system: `https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#idf-py`

# esp-idf commands

Run on new terminal: `get_idf`  
Configure project settings: `idf.py menuconfig`  
Build: `idf.py build`  
Build + Flash: `idf.py -p /dev/ttyACM0 flash`  
Monitor: `idf.py -p /dev/ttyACM0 monitor`  

# usbipd commands (Admin powershell)

List available devices: `usbipd list`  
Bind: `usbipd bind -b <bus-id>`  
Attach to WSL: `usbipd attach --wsl --busid <bus-id>`  

# JTAG commands

Connect OpenOcd: `openocd -f board/esp32s3-builtin.cfg`  

# pin mapping

LCD main (top to bottom) (SPI2)
* VCC   -> +5V
* GND   -> GND
* CS    -> GPIO10
* RST   -> GPIO18
* DC    -> GPIO9
* MOSI  -> GPIO11
* SCK   -> GPIO12
* LED   -> GPIO8
* MISO  -> GPIO13
* _touch pins_

LCD SD (top to bottom) (SPI3)
* SD_CS     -> GPIO7
* SD_MOSI   -> GPIO6
* SD_MISO   -> GPIO5
* SD_SCK    -> GPIO4

# notes

* Component Config -> ESP System Setting
* increase PSRAM clock speed
* gdbstub on panic
