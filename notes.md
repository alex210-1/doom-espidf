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
