# ESP32C6_Zigbee_Example
Use Zigbee with ESP32C6 and Arduino

Turns a ESP32C6 dev board into a dimmable zigbee light.

## Arduino setup
- Add ESP32 board support in Arduino File :: Preferences :: Add. boards manager URLs
  Add _https://espressif.github.io/arduino-esp32/package_esp32_dev_index.json_
  (notice the _dev_ for development/alpha)
- In Tools :: Board :: Boards Manager
  search for 'esp32 by Expressif Systems', install _3.0.0-alphaX_ release
- Select your board from the Tools :: Board menu. E.g. _ESP32C6 Dev Module_
- Select Tools :: Partition Scheme _Zigbee 4MB with spiffs_
- Select Tools :: Zigbee Mode _Zigbee ED (end device)_
- Select Tools :: Core Debug Level _Verbose_ to get log output in the serial monitor

## Reset device
For reset/rejoin uncomment the line that reads
```esp_zb_nvram_erase_at_start(true);```

