/**
 * Module: GarageDoor
 *  Data:  data/homie/config.json, 
 *         data/homie/ui_bundle.gz
 *  Created:  02/22/2020
 *  
 *  Description:
 *  At sensorsInterval (15 min) send 
 *  - Temperature, Humidity, Motion, EntryCount, and Door Position.
 *  When humans are detected send notice once per stats interval, interval will reset
 *  When analog change from saftey beam, record entry/exit.
 *  When door operates broadcast door position change by 4% for 30 seconds until movement stops
 *  - maybe decode moving up or down, and open/closed from home/max
 *  On handler command pulse Relay to open/close door opener, and reset position timer
 *  Position uses an interrupt to read data, disable VL53L0X when not in used by ignoring it, as shutdown resets device
 *  - also attach/detach interrupt with WiFi enablement and/or non-use
 * 
 * 
 * Library Storage: /Users/jscott/ccPlatform/DoorOpener/.pio/libdeps/esp32doit-devkit-v1
    Updating ArduinoJson                     @ 6.14.1         [Up-to-date]
    Updating AsyncMqttClient                 @ 0.8.2          [Up-to-date]
    Updating AsyncTCP                        @ 1.1.1          [Up-to-date]
    Updating Bounce2                         @ 2.52           [Up-to-date]
    Updating DHT sensor library for ESPx     @ 1.17           [Up-to-date]
    Updating ESP Async WebServer             @ 1.2.3          [Up-to-date]
    Updating ESP8266 and ESP32 OLED driver for SSD1306 displays @ 4.1.0   [Up-to-date]
    Updating ESPAsyncTCP                     @ 1.2.2          [Up-to-date]
    Updating Homie                           @ 3.0.0          [Up-to-date]
    Updating VL53L0X                         @ 1.2.0          [Up-to-date]
 */

#include <Arduino.h>
#include <driver/adc.h>
#include <Wire.h>
#include "SSD1306Wire.h"

#include <Homie.h>
#include <VL53L0X.h>
#include "DHTesp.h"
#include "ExponentialFilter.h"

#define SKN_MOD_NAME    "GarageDoor"
#define SKN_MOD_VERSION "0.1.0"
#define SKN_MOD_BRAND   "SknSensors"
#define SKN_MOD_TITLE   "Door Operations"

#define NODE_NAME       "Door Control"

#define PROP_ENTRY        "entry"
#define PROP_ENTRY_TITLE  "Entry Cnt"
#define PROP_RELAY        "operator"
#define PROP_RELAY_TITLE  "Operator"
#define PROP_MOTION       "motion"
#define PROP_MOTION_TITLE "Motion"
#define PROP_TEMP         "temperature"
#define PROP_TEMP_TITLE   "Temperature"
#define PROP_HUM          "humdity"
#define PROP_HUM_TITLE    "Humidity"
#define PROP_POS          "positon"
#define PROP_POS_TITLE    "Position"

#define PIN_DHT           16
#define PIN_MOTION        17
#define PIN_L0X           18
#define PIN_RELAY         19
#define PIN_SDA           21
#define PIN_SCL           22
#define PIN_ENTRY         36

/*
 * I2c == 22, 21
*/
#define DHT_TYPE          DHT11

/*
 * Sensor Values
 */
volatile unsigned long guiTimeBase   = 0,
                  gulLastMotionRead  = 0,
                  gulLoxDuration     = 0;
volatile int      giEntry            = 0,
                  giEntryMax         = 0,
                  giEntryValue       = 0,
                  giLastEntryValue   = 0;
volatile bool     gbLOXReady         = false,
                  gvDuration         = false,
                  gbValue            = false,
                  gvMotion           = false,  
                  gvLastMotion       = false,
                  gbStatsCycle       = false;
volatile  uint16_t giLOX             = 0,
                  giValue            = 0,
                  giLastLOXValue     = 0;

volatile float    gfTemperature      = 0.0f, 
                  gfHumidity         = 0.0f,
                  gfValue            = 0.0f;

float             value = 0.0;
char              gBuffer[48],
                  gcDisplayBuf[48];

/*
 * Temperature Sensor
*/	
DHTesp dht;
TempAndHumidity newValues;

/*
 * Time of Flight Sensor
*/
VL53L0X lox;

/*
 * 128 x 64 OLED
*/
SSD1306Wire display(0x3c, PIN_SDA, PIN_SCL);

/*
 * ExponentialFilter for Distance Value and ADC Readings
*/
ExponentialFilter<uint16_t> FilteredPosition(5, 0);
ExponentialFilter<uint32_t> FilteredEntry(20, 0);

HomieNode garageNode(SKN_MOD_NAME, NODE_NAME, SKN_MOD_BRAND);

void IRAM_ATTR motionInterruptHandler() {
  gvMotion = true;
}

void IRAM_ATTR loxInterruptHandler() {
  gbLOXReady = true; // set the new data ready flag to true on interrupt
}

bool broadcastHandler(const String& level, const String& value) {
  Homie.getLogger() << "Received broadcast level " << level << ": " << value << endl;
  return true;
}

bool operateDoorHandler(const HomieRange& range, const String& value) {
  Homie.getLogger() << "Operator Command: " << value << endl;
  if (value == "ON" || value=="true" || value == "on") {
    gulLoxDuration = millis() + 30000; // read for 30 seconds

    digitalWrite(PIN_RELAY, HIGH );
    garageNode.setProperty(PROP_RELAY).send( "ON" );
    delay(250);
    digitalWrite(PIN_RELAY, LOW );
    garageNode.setProperty(PROP_RELAY).send( "OFF" );
    
    giLastEntryValue = 0;
  } 
  return true;
}

void handleLOX() {
  if (gbLOXReady) {
    giLOX = lox.readRangeContinuousMillimeters(); // read data value
    if (!lox.timeoutOccurred()) {
      FilteredPosition.Filter(giLOX);
      giLastLOXValue = FilteredPosition.Current();
      Homie.getLogger() << "VL53L0X Reading: " << giLastLOXValue << endl;
      garageNode.setProperty(PROP_POS).send(String( giLastLOXValue ));
    }
    gbLOXReady = false;
  }   
}

void handlePosition() {
  Homie.getLogger() << "Position: " << giLastLOXValue << endl;
  garageNode.setProperty(PROP_POS).send(String(giLastLOXValue));

  snprintf(gcDisplayBuf, sizeof(gcDisplayBuf), "Pos: %d", giLastLOXValue );
  display.drawString(0,0, gcDisplayBuf);
}

void handleEntry(bool loop = true ) {
  /*
    * 4.9vdc produces a reading of 1058 ish 
    * should run inside doorPosition min/max limits   */    
  if (loop) {
    FilteredEntry.Filter( adc1_get_raw(ADC1_CHANNEL_0) );
    giEntry = FilteredEntry.Current();
    if ( giEntry >= 1000 && giEntry >= giLastEntryValue ) { // 20 to 21.5 volts, 
      giEntryValue++;  
      giEntryMax = giEntry > giEntryMax ? giEntry : giEntryMax ;
      giLastEntryValue = giEntry;
      Homie.getLogger() << "Entries: " << giEntryValue 
                        << ", Raw: " << giEntry 
                        << ", Max: " << giEntryMax << endl;
    }
  } else {
      Homie.getLogger() << "Entries: " << giEntryValue 
                        << ", Raw: " << giLastEntryValue 
                        << ", Max: " << giEntryMax << endl;
    garageNode.setProperty(PROP_ENTRY).send(String(giEntryValue));                                        
    giLastEntryValue = 0;

    snprintf(gcDisplayBuf, sizeof(gcDisplayBuf), "Entries: %d", giEntryValue );
    display.drawString(0,48, gcDisplayBuf);
  }
}

void handleMotion(bool loop = true) { 
  if ( gvMotion && loop && (gvMotion != gvLastMotion) ) {  
    garageNode.setProperty(PROP_MOTION).send((gvMotion ? "ON" : "OFF"));
    Homie.getLogger() << "Motion: " << (gvMotion ? "ON" : "OFF") << endl;
    gvLastMotion = gvMotion;
  } else if (!loop) {
    garageNode.setProperty(PROP_MOTION).send((gvMotion ? "ON" : "OFF"));
    Homie.getLogger() << "Motion: " << (gvMotion ? "ON" : "OFF") << endl;

    snprintf(gcDisplayBuf, sizeof(gcDisplayBuf), "Motion: %s", (gvMotion ? "ON" : "OFF"));
    display.drawString(0,32, gcDisplayBuf);
    gvLastMotion = gvMotion = loop;
  }
}

// ref: https://github.com/beegee-tokyo/DHTesp
void handleTemps() {   // called every status interval 
  newValues = dht.getTempAndHumidity();
  if( dht.getStatus() == 0 ){    
    gfTemperature = dht.toFahrenheit(newValues.temperature);
    snprintf(gBuffer, sizeof(gBuffer), "%.1f", gfTemperature);
    Homie.getLogger() << "Temperature: " << gfTemperature << " °F" << endl;
    garageNode.setProperty(PROP_TEMP).send(gBuffer);
  
    gfHumidity = newValues.humidity;
    snprintf(gBuffer, sizeof(gBuffer), "%.1f", gfHumidity);
    Homie.getLogger() << "Humidity: " << gfHumidity << " %" << endl;
    garageNode.setProperty(PROP_HUM).send(gBuffer);

    snprintf(gcDisplayBuf, sizeof(gcDisplayBuf), "%.1f °F, %.1f %%", gfTemperature, gfHumidity);
    display.drawString(0,16, gcDisplayBuf);
  }     
}

void onHomieEvent(const HomieEvent& event) {
  switch (event.type) {
    case HomieEventType::NORMAL_MODE:
      Serial << "Normal mode started" << endl;
      break;
    case HomieEventType::WIFI_DISCONNECTED:
      Serial << "Wi-Fi disconnected, reason: " << (int8_t)event.wifiReason << endl;
      detachInterrupt(PIN_L0X); 
      detachInterrupt(PIN_MOTION); 
      break;      
    case HomieEventType::MQTT_READY:
      Serial << "MQTT connected" << endl;
      attachInterrupt(PIN_L0X, loxInterruptHandler, FALLING); 
      attachInterrupt(PIN_MOTION, motionInterruptHandler, RISING); 
      break;
    case HomieEventType::MQTT_DISCONNECTED:
      Serial << "MQTT disconnected, reason: " << (int8_t)event.mqttReason << endl;
      break;
    case HomieEventType::SENDING_STATISTICS:
      Serial << "Sending statistics" << endl;
      display.clear();

      handleMotion(false);
      handleEntry(false);
      handlePosition();
      handleTemps();

      display.display();
      break;
    default:
      break;
  }
}

/*
 * Called when Homie isConnected
 */
void loopHandler() {
  guiTimeBase = millis();
  gvDuration = ((guiTimeBase - gulLastMotionRead) >= 500);
  
  if (gvDuration) {   // Every half second poll 
    handleEntry(true);
  }

  if (gvDuration) {  // Interrupt Driven Handler
    handleMotion(true);    
  }

  if (gbLOXReady && gulLoxDuration >= guiTimeBase) {
    handleLOX(); // Interrupt Driven Handler
  }
}

void setupHandler() {
   dht.setup(PIN_DHT, DHTesp::DHT11);
   yield();
   
   Wire.begin(PIN_SDA, PIN_SCL, 100000);
   yield();

   delay(2000);

  lox.setTimeout(500);
  gbValue = lox.init();
  if (!gbValue)
  {
    Serial.println("Failed to detect and initialize sensor!");
    delay(2000);
    ESP.restart();
  }

  /*
   * lower the return signal rate limit (default is 0.25 MCPS)
   * increase laser pulse periods (defaults are 14 and 10 PCLKs)
   * increase timing budget to 200 ms
   * Use continuous timed mode and provide the desired 
   * inter-measurement period in ms (e.g. sensor.startContinuous(250)).
  */
  lox.setSignalRateLimit(0.1);
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  lox.setMeasurementTimingBudget(200000);
  lox.startContinuous(50); // total should be around 250ms per reading

   // Initialising the display.
  display.init();
  display.flipScreenVertically();
  display.clear();

  // Font Demo1
  // create more fonts at http://oleddisplay.squix.ch/
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);

  display.drawString(0, 0, "Hello world");
  display.drawString(0, 16, "0123456789ABCDEF");
  display.display();
}

void setup() {
  delay(50);
  Serial.begin(115200);
  Serial << endl << endl;
  yield();

  pinMode(PIN_ENTRY,  INPUT);     // analog sensor of IR door break 20.0vdc = off, 21.5vdc = on
  pinMode(PIN_MOTION, INPUT);     // RCWL sensor
  pinMode(PIN_L0X,    INPUT);     // VL53L0X Interrupt Ready Pin
  pinMode(PIN_RELAY, OUTPUT);     // Door operator
  digitalWrite(PIN_RELAY, LOW);  // Init door to off

  /*
   * Voltage divider analog in pins
   * https://dl.espressif.com/doc/esp-idf/latest/api-reference/peripherals/adc.html
   * set up A:D channels and attenuation
   *   * Read the sensor by
   *   * uint16_t value =  adc1_get_raw(ADC1_CHANNEL_0);
   *  150mv-3.9vdc inside 0-4095 range   (constrained by 3.3vdc supply)
  */
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); // Pin 36
  
  yield();
   
  Homie_setFirmware(SKN_MOD_NAME, SKN_MOD_VERSION);
  Homie_setBrand(SKN_MOD_BRAND);
  
  Homie.setSetupFunction(setupHandler)
      .setLoopFunction(loopHandler)
      .setBroadcastHandler(broadcastHandler)
      .onEvent(onHomieEvent);  
  
  garageNode.advertise(PROP_RELAY)
            .setName(PROP_RELAY_TITLE)
            .setRetained(true)
            .settable(operateDoorHandler)
            .setDatatype("boolean")
            .setFormat("%s");

  garageNode.advertise(PROP_MOTION)
            .setName(PROP_MOTION_TITLE)
            .setRetained(true)
            .setDatatype("string")
            .setFormat("%s");

  garageNode.advertise(PROP_TEMP)
            .setName(PROP_TEMP_TITLE)
            .setDatatype("float")
            .setFormat("%.1f")
            .setUnit("ºF");

  garageNode.advertise(PROP_HUM)
            .setName(PROP_HUM_TITLE)
            .setDatatype("float")
            .setFormat("%.1f")
            .setUnit("%");  

  garageNode.advertise(PROP_ENTRY)
            .setName(PROP_ENTRY_TITLE)
            .setDatatype("integer")
            .setRetained(true)
            .setFormat("%d")
            .setUnit("#");

  garageNode.advertise(PROP_POS)
            .setName(PROP_POS_TITLE)
            .setDatatype("integer")
            .setRetained(true)
            .setFormat("%d")
            .setUnit("#");


  Homie.setup();
}

void loop() {
  Homie.loop();
}
