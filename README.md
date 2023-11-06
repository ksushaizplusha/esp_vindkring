This project contains code to add zigbee support to ikea vindriktning sensor.

This projects based on [esp-idf](https://github.com/espressif/esp-idf) framework, and uses [https://github.com/espressif/esp-zigbee-sdk](esp-zigbee-sdk).

Tested on esp32-c6, with BME280 sensor, ikea vindriktning sensor and zigbee2mqtt.

## Building

First, checkout esp-idf managed components:

```
idf.py update-dependencies
```

And then build project in your prefered way.
For more details, read [official esp-idf documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/).


## Connecting to esp

### vindriktning:

GPIO2 as uart rx


### BME280:

GPIO_NUM_10 to SDA
GPIO_NUM_11 to SCL,


## z2m

add custom converter from this repository "durka_model_V.js".

Permit join in Z2M and reboot device, it will connect automatically.


# make it clear:

vindriktning have absolutly awfull and unusable name, to short it i use "vindkring" in this project, just because it simpler for me to remember and type.

You can remove bmp280/bme280 just by removing include and lines of code, which communicate with it. It's not necessary to have bme280/bmp280 to get info from vindkring.