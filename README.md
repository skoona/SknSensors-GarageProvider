# GarageProvider
Homie V3 based Garage Door Operator and Environment Multi-Sensor.

![Mini32 ESP32-WROVER-B](https://github.com/LilyGO/TTGO-T7-Demo/blob/master/images/T7V1.4.jpg)

### Features
* Transmits temperature, humidity, motion, door position, every 15 minutes.
* Transmits Motion ON status ONCE for 60 seconds since last motion trigger.
* Detects and transmits counter when saftey beam has been broken while door is operating.
* Determines and transmits door direction, current position, and final position after door toggle.
* Displays metrics every 1/2 seconds while Motion indicates Human presence.

### Operation sequence

    Device     Time shift          HomieEvent Stats  
    Sensor --> CharBuffer/Var ---> sendSensors()

### Basic Sequences

* OnMotionInterrupt:
* * Set Var and (reset)MotionDuration  for 10 minutes
* * Report once per/until MotionDuration interval
* Oled Display:
* * Display Sensor BUffers
* * Every 1/2 second until MotionDuration expires
* Temp/Humidity:
* * Save sample to Vars 
* * Every 10 Minutes
* ADCSafetyBeam:
* * Detect beam brake when voltage moves from 20.vdc to 2.15v 
* * Increment and send counter
* * Until RangingDuration expires
* MQTT-ONCmd: ON/on/true
* * Set RangingDuration
* * Toggle RELAY High for 750ms
* OnRangingInterrupt:
* * DataReady, compute direction and position as ranging vars
* * Read, save, and send ranging vars
* * Until RangingDuration expires
* * Shutdown Ranging until next door command
* Homie Statistics
* * Send all sensor value and Homie stats every config.json:device_stats_interval or 5 minutes


### Build
* PlatformIO Based Project
* Designed for use with ESP32, D1-Mini style hardware; 
* * Like: [LILYGO® TTGO T7 V1.4 Mini32 ESP32-WROVER-B](http://www.lilygo.cn/prod_view.aspx?TypeId=50033&Id=1258&FId=t3:50033:3)


### Requires The following Libraries

    Updating ArduinoJson                     @ 6.14.1         [Up-to-date]
    Updating AsyncMqttClient                 @ 0.8.2          [Up-to-date]
    Updating AsyncTCP                        @ 1.1.1          [Up-to-date]
    Updating Bounce2                         @ 2.53           [Up-to-date]
    Updating DHT sensor library for ESPx     @ 1.17           [Up-to-date]
    Updating ESP Async WebServer             @ 1.2.3          [Up-to-date]
    Updating ESP8266 and ESP32 OLED driver for SSD1306 displays @ 4.1.0   [Up-to-date]
    Updating ESPAsyncTCP                     @ 1.2.2          [Up-to-date]
    Updating Homie                           @ 3.0.0          [Up-to-date]
    Updating VL53L1X                         @ 1.2.0          [Up-to-date]

### Wiring

    *   PIN Description    Supply  Trigger        Comment
    * =============================================================================
    *    D7 RCWL-0516      5V      HIGH (3.3V)   if motion detected. 0V normally.
    *    D7 Motion-Detected Interrupt            INTR-RISING
    *    D5 RELAY          3.3V    HIGH          3.3v door relay
    *    D6 DHT-11         5V      Data/OneWire  Temperature/Humidity
    *    A0 Volt/Divider  21.5V    ADC           Sensor 100K/16 (120K/12K={21.5=1.955v}{20.0/1.818v})
    * D1/D2 VL53L1X        3.3V    I2c           Line of Sight Ranger 4m max
    *    D8 Data-Ready Interrupt   GPIO          INTR-FALLING 
    *    D3 Shutdown Pin   3.3V    GPIO          High for normal Operations, resets VL53L1X
    * D1/D2 OLED 128x64    3.3V    I2c SSD1306   Display

### Programming Notes
* An attempt was made to use all ESP32 RTOS Features without success.  The Ardunio libraries do not work well together on multiple cores; YET!


## Contributing

1. Fork it 
2. Create your feature branch (`git checkout -b my-new-feature`)
3. Commit your changes (`git commit -am 'Add some feature'`)
4. Push to the branch (`git push origin my-new-feature`)
5. Create a new Pull Request


## License

The gem is available as open source under the terms of the [MIT License](http://opensource.org/licenses/MIT).
