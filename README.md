This project contains code to add zigbee support to ikea vindriktning sensor.

This projects based on [esp-idf](https://github.com/espressif/esp-idf) framework, and uses [esp-zigbee-sdk](https://github.com/espressif/esp-zigbee-sdk).

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

GPIO2 as uart rx (rest pin on vindriktning sensor pcb, [image for example](https://community.home-assistant.io/t/ikea-vindriktning-air-quality-sensor/324599/45))

### BME280:

GPIO_NUM_10 to SDA
GPIO_NUM_11 to SCL

Make sure which bmx280 sensor you have, some powers from 5v, somes from 3.3v. If you connect 3.3v sensor to 5v, you will burn your sensor forever, there is no way back.

## z2m

add custom converter from this repository "durka_model_V.js".

Permit join in Z2M and reboot device, it will connect automatically.


# make it clear:

vindriktning have absolutly awfull and unusable name, to short it i use "vindkring" in this project, just because it simpler for me to remember and type.

You can remove bmp280/bme280 just by removing include and lines of code, which communicate with it. It's not necessary to have bme280/bmp280 to get info from vindkring.

Make sure you're understand what you are doing, this is just some crappy DIY project from internet. If you break your esp32 or vindkring, it's your fault.