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
 * *   PIN Description  Supply  Trigger
 * =============================================================================
 * *    17 RCWL-0516      5V    HIGH (3.3V)   if motion detected. 0V normally.
 * -    17 Motion-Detected Interrupt          INTR-RISING
 * *    19 RELAY        3.3V    HIGH
 * *    16 DHT-11         5V    Data/OneWire  Temperature/Humidity
 * *    36 Volt/Divider 21.5V   ADC           Sensor
 * * 22/21 VL53L0X      3.3V    I2c           Line of Sight Ranger
 * -    18 Data-Ready Interrupt               INTR-FALLING 
 * * 22/21 OLED 128x64  3.3V    I2c SSD1306   Display
 * =============================================================================
 * 
 * 
 * Library Storage: /Users/jscott/ccPlatform/DoorOpener/.pio/libdeps/esp32doit-devkit-v1
    Updating ArduinoJson                     @ 6.14.1         [Up-to-date]
    Updating AsyncMqttClient                 @ 0.8.2          [Up-to-date]
    Updating AsyncTCP                        @ 1.1.1          [Up-to-date]
    Updating Bounce2                         @ 2.52           [Up-to-date]
    Updating DHT sensor library for ESPx     @ 1.17           [Up-to-date]
    Updating ESP Async WebServer             @ 1.2.3          [Up-to-date]
    Updating ESP8266 and ESP32 OLED driver for SSD1306 displays @ 4.1.0  [Up-to-date]
    Updating ESPAsyncTCP                     @ 1.2.2          [Up-to-date]
    Updating Homie                           @ 3.0.0          [Up-to-date]
    Updating VL53L0X                         @ 1.2.0          [Up-to-date]
 */

#include <Arduino.h>
#include <driver/adc.h>
#include "SSD1306Wire.h"

#include <Homie.h>
#include <VL53L0X.h>

#include "DHTesp.h"
#include "ExponentialFilter.h"

#ifndef ESP32
#pragma message(THIS MODULE IS FOR ESP32 ONLY!)
#error Select ESP32 board.
#endif

#define SKN_MOD_NAME    "GarageDoor"
#define SKN_MOD_VERSION "0.2.1"
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
                  gulLastTimeBase    = 0,
                  gulTempsDuration   = 0,
                  gulMotionDuration  = 0,
                  gulLoxDuration     = 0,
                  gulEntryDuration   = 0;

volatile int      giEntry            = 0,
                  giEntryMax         = 600,
                  giEntryMin         = 600,
                  giEntryCount       = 0,
                  giLastEntry        = 10;        

volatile bool     gbLOXReady         = false,
                  gvDuration         = false,
                  gbValue            = false,
                  gvMotion           = false,  
                  gvLastMotion       = false,
                  gbCoreCycle        = false; // trigger to use core0

volatile  uint16_t giLOX             = 0,
                  giValue            = 0,
                  giLastLOXValue     = 0;

volatile float    gfTemperature      = 0.0f, 
                  gfHumidity         = 0.0f,
                  gfValue            = 0.0f,
                   value             = 0.0f;

char              gBuffer[48],
                  gcTemps[48],
                  gcPosition[48],
                  gcEntry[48];



String            pchmotionON  = F("Motion: ON "),
                  pchmotionOFF = F("Motion: OFF"),
                  pchOFF       = F("OFF"),
                  pchON        = F("ON");


/*
 * ESP32 Tasks for Core 0
*/
void sensorTask(void *pvParameters);
TaskHandle_t sensorTaskHandle = NULL;

/*
 * Temperature Sensor
*/	
DHTesp dht;
TempAndHumidity newValues;


/*
 * 128 x 64 OLED 
 * - This should be first, since it initializes Wire
*/
SSD1306Wire display(0x3c, PIN_SDA, PIN_SCL, GEOMETRY_128_64);

/*
 * Time of Flight Sensor
*/
VL53L0X lox;

/*
 * ExponentialFilter for Distance Value and ADC Readings
*/
ExponentialFilter<uint16_t> FilteredPosition(55, 8000);
ExponentialFilter<uint32_t> FilteredEntry(55, 1000);

HomieNode garageNode(SKN_MOD_NAME, NODE_NAME, SKN_MOD_BRAND);

/**
 * Utility to handle Duration Roll Overs
*/
unsigned long setDuration(unsigned long duration) {
  unsigned long value = millis() + duration;
  if (value <= duration) { // rolled
    value = duration;
  } 
  return value;
}

void IRAM_ATTR motionInterruptHandler() {
  gvMotion = true;
  gvLastMotion = false;
  gulMotionDuration = setDuration( 600000UL ); // Wait 10 min for an entry
}

void IRAM_ATTR loxInterruptHandler() {
  gbLOXReady = true; // set the new data ready flag to true on interrupt
}

bool broadcastHandler(const String& level, const String& value) {
  Homie.getLogger() << "Received broadcast level " << level << ": " << value << endl;
  return true;
}

/**
 * When door is operated release the Entry counter and the Distance Ranger
*/
bool doorOperatorHandler(const HomieRange& range, const String& value) {
  Homie.getLogger() << "Operator Command: " << value << endl;
  if (value == pchON || value == "true" || value == "on") {
    // handle rollover
    gulLoxDuration   = setDuration( 30000UL ); // LOX read for 30 seconds
    gbLOXReady = true;
    gulEntryDuration = setDuration( 900000UL ); // Wait 15 min for an entry

    digitalWrite(PIN_RELAY, HIGH );
    delay(500);
    digitalWrite(PIN_RELAY, LOW );
    garageNode.setProperty(PROP_RELAY).send( pchOFF );
  } 
  return true;
}

/**
 * Read on Interrupt Ready set gbLOXReady
 * - Read until time expires
 * - Time controlled by caller and/or Door Operator
 */
void gatherPosition() {
  if (gbLOXReady) {
    giLOX = lox.readRangeContinuousMillimeters(); // read data value
    if (!lox.timeoutOccurred()) {
      FilteredPosition.Filter(giLOX);
      giLastLOXValue = FilteredPosition.Current();
      garageNode.setProperty(PROP_POS).send(String(giLastLOXValue));

    } else {
      Homie.getLogger() << F("VL53L0X Timeout: ") << lox.last_status  << endl;
    }
    gbLOXReady = false;
  }   
}

/**
 * Triggered by Door Opener for gulEntryDuration
 * Entry is constrained by ADC Max value
 * - Value ranges from 20 to 21.5 when triggered
 * - runs for 15 min after door operates
*/
void gatherEntry( ) {
  /*
    * 4.9vdc produces a reading of 1058 ish 
    * should run inside doorPosition min/max limits   */    
  FilteredEntry.Filter( adc1_get_raw(ADC1_CHANNEL_0) );
  giEntry = FilteredEntry.Current();
  if ( giEntry != giLastEntry ) { // 20 to 21.5 volts, 
    if (giEntry > giEntryMax) {
      giEntryCount++;  
      garageNode.setProperty(PROP_ENTRY).send(String(giEntryCount));                                        
      Homie.getLogger() << "Entries: " << giEntryCount 
                        << ", Raw: " << giEntry 
                        << ", Min: " << giEntryMin
                        << ", Max: " << giEntryMax << endl;
    }
    giEntryMax = giEntry > giEntryMax ? giEntry : giEntryMax ;
    giEntryMin = giEntry < giEntryMin ? giEntry : giEntryMin ;
    giLastEntry = giEntry;  

  }
}

/**
 * Interrupt Driven turns on gvMotion
 * - ON with Interrupt
 * - OFF with Stats interval
*/
void gatherMotion(bool stats = false ) { 
  if (gvLastMotion != gvMotion) {
    garageNode.setProperty(PROP_MOTION).send((gvMotion ? pchON : pchOFF));
    Homie.getLogger() << (gvMotion ? pchmotionON : pchmotionOFF) << endl;
    gvLastMotion = gvMotion;
  }

  if (stats && gvMotion) {
    gvLastMotion = gvMotion = false;
    garageNode.setProperty(PROP_MOTION).send(pchOFF);
    Homie.getLogger() << pchmotionOFF << endl;
  }
}

// ref: https://github.com/beegee-tokyo/DHTesp
void gatherTemps() {   // called every status interval 
  newValues = dht.getTempAndHumidity();
  if( (dht.getStatus() == 0) ){    
    gfTemperature = dht.toFahrenheit(newValues.temperature); 
    gfHumidity = newValues.humidity;

    snprintf(gBuffer, sizeof(gBuffer), "%.1f", gfTemperature);
    garageNode.setProperty(PROP_TEMP).send(gBuffer);
  
    snprintf(gBuffer, sizeof(gBuffer), "%.1f", gfHumidity);
    garageNode.setProperty(PROP_HUM).send(gBuffer);      
  }     
}

/**
 * Update Local OLED Display
*/
void displaySensors() {
  if (gbCoreCycle) {
 
    snprintf(gcEntry, sizeof(gcEntry), "C:%d M:%d", giEntryCount, giEntryMax );
    snprintf(gcTemps, sizeof(gcTemps), "%.1f °F, %.1f %%", gfTemperature, gfHumidity);
    snprintf(gcPosition, sizeof(gcPosition), "P:%hd R:%hd", giLastLOXValue, giLOX );

    display.clear();
      display.drawString(0,0, gcPosition);
      display.drawString(0,16, gcTemps);
      display.drawString(0,32, (gvMotion ? pchmotionON : pchmotionOFF));
      display.drawString(0,48, gcEntry);
    display.display();
  }
}

/**
 * Send Current sensor data to monitor
*/
void sendSensorStats() {
  snprintf(gBuffer, sizeof(gBuffer), "%.1f", gfTemperature);
  garageNode.setProperty(PROP_TEMP).send(gBuffer);
  snprintf(gBuffer, sizeof(gBuffer), "%.1f", gfHumidity);
  garageNode.setProperty(PROP_HUM).send(gBuffer); 
  
  garageNode.setProperty(PROP_MOTION).send((gvMotion ? pchON : pchOFF)); 
  garageNode.setProperty(PROP_ENTRY).send(String(giEntryCount));  
  garageNode.setProperty(PROP_POS).send(String(giLastLOXValue));
}

void onHomieEvent(const HomieEvent& event) {
  switch (event.type) {
    case HomieEventType::NORMAL_MODE:
      Serial << "Normal mode started, core: " << xPortGetCoreID() << endl;
      break;
    case HomieEventType::WIFI_DISCONNECTED:
      Serial << "Wi-Fi disconnected, reason: " << (int8_t)event.wifiReason << endl;
      detachInterrupt(PIN_L0X); 
      detachInterrupt(PIN_MOTION); 
      break;      
    case HomieEventType::MQTT_READY:
      Serial << "MQTT connected, core: " << xPortGetCoreID() << endl;
      attachInterrupt(PIN_L0X, loxInterruptHandler, FALLING); 
      attachInterrupt(PIN_MOTION, motionInterruptHandler, RISING); 
      break;
    case HomieEventType::MQTT_DISCONNECTED:
      Serial << "MQTT disconnected, reason: " << (int8_t)event.mqttReason << endl;
      break;
    case HomieEventType::SENDING_STATISTICS:
      Serial << "Sending statistics, core: " << xPortGetCoreID() << endl;
      sendSensorStats();
      break;
    default:
      break;
  }
}

/*
 * Core 0 Sensor Task
 */
void sensorHandler() {
  guiTimeBase = millis();
  gvDuration = ((guiTimeBase - gulLastTimeBase) >= 500);

  if (gvDuration && gulEntryDuration >= guiTimeBase ) {   // Every half second poll 
    gatherEntry(); 
  }

  if (gvDuration && gulMotionDuration >= guiTimeBase ) {  // Interrupt Driven Handler
    gatherMotion();    
  } else if (gvMotion && gvDuration) {
    gatherMotion(true);  // turn off
  }

  if (gvDuration && gbLOXReady && gulLoxDuration >= guiTimeBase) { // Interrupt Driven Handler
    gatherPosition(); 
  }

  if (gulTempsDuration >= guiTimeBase ) {    
    gatherTemps();
    gulTempsDuration = setDuration( 600000UL ); // Wait 10 min for next measurement
  }

  if (gvDuration) { // Every half second poll 
    gulLastTimeBase = guiTimeBase;
  }
}

/*
 * Primary Sensor Task
 * - Runs all elements
*/
void sensorTask(void *pvParameters) {
  Homie.getLogger() << "SensorTask running on core: " << xPortGetCoreID() << endl;
    gatherPosition(); 
    gatherTemps();
    // gatherEntry();

  while (1) {
    sensorHandler();
    vTaskDelay(1);
    taskYIELD();
  }
}

void homieLoopHandler() {
  if (gbCoreCycle) {
    if (gvDuration) {
      displaySensors();
    }
    return;
  }

  gbCoreCycle = true; // make this a oneShot

  /* 
   * Start ESP32 Sensor Task in Core 0
  */
  xTaskCreatePinnedToCore(
			sensorTask,                     /* Function to implement the task */
			"Sensors ",                     /* Name of the task */
			8192,                           /* Stack size in words */
			NULL,                           /* Task input parameter */
			5,                              /* Priority of the task */
			&sensorTaskHandle,              /* Task handle. */
			1);                             /* Core where the task should run tskNO_AFFINITY */

  if (sensorTaskHandle == NULL) {
    Serial.println("Failed to start Sensor task!");
    gbCoreCycle = false;
    delay(2000);
    ESP.restart();
  }

}

void homieSetupHandler() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  yield();

  dht.setup(PIN_DHT, DHTesp::DHT11);
  yield();

   // Initialising the display.
  display.init();
  display.flipScreenVertically();
  display.clear();
  yield();

  // Font Demo1
  // create more fonts at http://oleddisplay.squix.ch/
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);

  display.drawString(0, 0, SKN_MOD_TITLE);
  display.drawString(40, 16, SKN_MOD_VERSION);
  display.drawString(0, 32, "0123456789ABC");
  display.display();
  yield();

  /*
   * lower the return signal rate limit (default is 0.25 MCPS)
   * increase laser pulse periods (defaults are 14 and 10 PCLKs)
   * increase timing budget to 200 ms
   * Use continuous timed mode and provide the desired 
   * inter-measurement period in ms (e.g. sensor.startContinuous(250)).
  */
  lox.setTimeout(200);
  gbValue = lox.init();
  if (!gbValue) {
    Serial.println("Failed to detect and initialize VL53L0X sensor!");
    delay(2000);
    ESP.restart();
  }
  lox.setSignalRateLimit(0.1);
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  lox.setMeasurementTimingBudget(200000);
  lox.startContinuous(50); // total should be around 250ms per reading
  yield();
  
}

void setup() {
  delay(50);
  Serial.begin(115200);
  Serial << endl << endl;
  Serial.print("Homie running on core ");
  Serial.println(xPortGetCoreID());
  yield();

  pinMode(PIN_ENTRY,  INPUT);     // analog sensor of IR door break 20.0vdc = off, 21.5vdc = on
  pinMode(PIN_MOTION, INPUT);     // RCWL sensor
  pinMode(PIN_L0X,    INPUT);     // VL53L0X Interrupt Ready Pin
  pinMode(PIN_RELAY, OUTPUT);     // Door operator
  digitalWrite(PIN_RELAY, LOW);   // Init door to off

  /* Turn off Distance and Entry Readings */
  gulTempsDuration = gulEntryDuration = gulLoxDuration = setDuration( 1000UL );

  FilteredPosition.Filter(8000);
  FilteredEntry.Filter(1000);

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
  
  Homie.setSetupFunction(homieSetupHandler)
      .setLoopFunction(homieLoopHandler)
      .setBroadcastHandler(broadcastHandler)
      .onEvent(onHomieEvent);  
  
  garageNode.advertise(PROP_RELAY)
            .setName(PROP_RELAY_TITLE)
            .setRetained(true)
            .settable(doorOperatorHandler)
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
