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
 *  When door operates broadcast door position change every 1/2 second for 30 seconds
 *  - maybe decode moving up or down, and open/closed from home/max
 *  On handler command pulse Relay to open/close door opener, and reset position timer
 *  Position uses an interrupt to read data, disabling VL53L0X when not in use corrupts the I2c channel!
 *  - also attach/detach interrupt with WiFi enablement/disablement
 * 
 * *   PIN Description  Supply  Trigger
 * =============================================================================
 * *    17 RCWL-0516      5V    HIGH (3.3V)   if motion detected. 0V normally.
 * -    17 Motion-Detected Interrupt          INTR-RISING
 * *    19 RELAY        3.3V    HIGH
 * *    16 DHT-11         5V    Data/OneWire  Temperature/Humidity
 * *    36 Volt/Divider 21.5V   ADC           Sensor 100K/16 (100K/15.942=24/3.3)
 * * 22/21 VL53L0X        5V    I2c           Line of Sight Ranger
 * -    18 Data-Ready Interrupt GPIO          INTR-FALLING 
 * -    23 Shutdown Pin   3.3V  GPIO          High for normal Operations, can reset VL53L0X
 * * 22/21 OLED 128x64    5V    I2c SSD1306   Display
 * =============================================================================
 * 
 * 
 * Library Storage: /Users/jscott/ccPlatform/DoorOpener/.pio/libdeps/esp32doit-devkit-v1
    Updating ArduinoJson                     @ 6.14.1         [Up-to-date]
    Updating AsyncMqttClient                 @ 0.8.2          [Up-to-date]
    Updating AsyncTCP                        @ 1.1.1          [Up-to-date]
    Updating Bounce2                         @ 2.53           [Up-to-date]
    Updating DHT sensor library for ESPx     @ 1.17           [Up-to-date]
    Updating ESP Async WebServer             @ 1.2.3          [Up-to-date]
    Updating ESP8266 and ESP32 OLED driver for SSD1306 displays @ 4.1.0   [Up-to-date]
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
#define SKN_MOD_VERSION "0.4.2"
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
#define PIN_SHDN          23
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
volatile unsigned long guiTimeBase   = 0,     // default time base, target 0.5 seconds
                  gulLastTimeBase    = 0,     // time base delta
                  gulTempsDuration   = 0,     // time between each temp measurement
                  gulMotionDuration  = 0,     // time to hold motion after trigger interrupt
                  gulLoxDuration     = 0,     // time to run VL53L0X after door operates
                  gulEntryDuration   = 0;     // time to run beam break monitor after door operates

volatile int      giEntry            = 0,     // value of beam break ADC
                  giEntryMax         = 600,   // open value of ADC reading -- break
                  giEntryMin         = 600,   // close value of ADC reading -- no break
                  giEntryCount       = 0,     // count of number of entries since startup
                  giLastEntry        = 10;    // ADC reading of last entry    

volatile bool     gbLOXReady         = true,  // VL53L0X startup -- Interrupt
                  gbLOXRunMode       = true,  // VL53L0X startup 
                  gbDisplayOn        = true,  // Display SHOULD be on or off    
                  gvDuration         = false, // 1/2 second marker
                  gbValue            = false, // initialization boolean, general use
                  gvMotion           = false, // Motion indicator value -- Interrupt
                  gvLastMotion       = false; // Previous Motion indicator

volatile  uint16_t giLOX             = 0,     // VL53L0X current reading
                  giLastLOXValue     = 0;     // VL53L0X Previous reading

volatile float    gfTemperature      = 0.0f,  // Current Temperature value
                  gfHumidity         = 0.0f;  // Current Humidity value

char              gcTemp[48],                 // Current Temperature Formatted
                  gcTemps[48],                // Combined Temperature and Humidity Formatted
                  gcHumid[48],                // Current Humidity Formatted
                  gcPosition[48],             // Current VL53L0X Positon Formatted
                  gcEntry[48];                // Current ADC Entry value



String            pchmotionON  = F("Motion: ON "), // String constants for logging
                  pchmotionOFF = F("Motion: OFF"),
                  pchOFF       = F("OFF"),
                  pchON        = F("ON");


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
ExponentialFilter<uint16_t> FilteredPosition(20, 2000);
ExponentialFilter<uint32_t> FilteredEntry(20, 2000);

HomieNode garageNode(SKN_MOD_NAME, NODE_NAME, SKN_MOD_BRAND);

/**
 * Utility to handle Duration Roll Overs
*/
unsigned long IRAM_ATTR setDuration(unsigned long duration) {
  unsigned long value = millis() + duration;
  if (value <= duration) { // rolled
    value = duration;
  } 
  return value;
}

void IRAM_ATTR motionInterruptHandler() {
  gulMotionDuration = setDuration( 600000UL ); // Wait 10 min for an entry
  gvMotion = true;
  gvLastMotion = false;
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
    gulEntryDuration = setDuration( 900000UL ); // Wait 15 min for an entry
    gulLoxDuration   = setDuration( 60000UL ); // LOX read for 60 seconds   
    gbLOXReady       = true;
    gbLOXRunMode     = true;
    digitalWrite(PIN_SHDN, HIGH);   // H to enable, L to disable -- might also reset


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
  if ( gbLOXReady && (guiTimeBase >= gulLoxDuration)) {
    if (gbLOXRunMode) {
      digitalWrite(PIN_SHDN, LOW);   // H to enable, L to disable -- might also reset
      gbLOXRunMode = false;
    }
    gbLOXReady = false;
    Homie.getLogger() << F("VL53L0X OFF~> Read: ") << giLastLOXValue 
                      << ", Raw: " << giLOX 
                      << endl;      
    taskYIELD();
    return;
  } 

  if (gbLOXReady) {
    giLOX = lox.readRangeContinuousMillimeters(); // read data value
    if (!lox.timeoutOccurred() && giLOX > 1) {
      FilteredPosition.Filter(giLOX);
      giLastLOXValue = FilteredPosition.Current();
      gbLOXReady = false;
      
      garageNode.setProperty(PROP_POS).send(String(giLastLOXValue));

      Homie.getLogger() << F("VL53L0X Read: ") << giLastLOXValue 
                        << ", Raw: " << giLOX 
                        << endl;      
      
    } else {
      Homie.getLogger() << F("VL53L0X Timeout: ") << lox.last_status  << endl;
    }
  }   
  
  taskYIELD();
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
 * - ON with Interrupt, duration set by interrupt
 * - OFF with after duration ends
*/
void gatherMotion(bool stats = false ) { 
  if (gvLastMotion != gvMotion) {
    garageNode.setProperty(PROP_MOTION).send((gvMotion ? pchON : pchOFF));
    Homie.getLogger() << (gvMotion ? pchmotionON : pchmotionOFF) << endl;
    gvLastMotion = gvMotion;
  }

  if (stats) {
    gvLastMotion = gvMotion = false;
    garageNode.setProperty(PROP_MOTION).send(pchOFF);
    Homie.getLogger() << pchmotionOFF << endl;
  }
}

/**
 *  ref: https://github.com/beegee-tokyo/DHTesp
 * Every 10 minutes
*/
void gatherTemps() {   // called every status interval 
  newValues = dht.getTempAndHumidity();
  if( (dht.getStatus() == 0) ){    
    gfTemperature = dht.toFahrenheit(newValues.temperature); 
    gfHumidity = newValues.humidity;

    snprintf(gcTemps, sizeof(gcTemps), "%.1f", gfTemperature);
    garageNode.setProperty(PROP_TEMP).send(gcTemps);
  
    snprintf(gcHumid, sizeof(gcHumid), "%.1f", gfHumidity);
    garageNode.setProperty(PROP_HUM).send(gcHumid);      
  }     
  gulTempsDuration = setDuration( 600000UL ); // Wait 10 min for next measurement
}

/**
 * Update Local OLED Display
 * - Only while Motion is Active
*/
void displaySensors() {
  if (gvMotion) {
 
    if (!gbDisplayOn) {
      display.displayOn();
      gbDisplayOn = true;
    } 

    snprintf(gcEntry, sizeof(gcEntry), "C:%d M:%d", giEntryCount, giEntryMax );
    snprintf(gcTemps, sizeof(gcTemps), "%.1f °F, %.1f %%", gfTemperature, gfHumidity);
    snprintf(gcPosition, sizeof(gcPosition), "P:%hd R:%hd", giLastLOXValue, giLOX );

    display.clear();
      display.drawString(0,0, gcPosition);
      display.drawString(0,16, gcTemps);
      display.drawString(0,32, (gvMotion ? pchmotionON : pchmotionOFF));
      display.drawString(0,48, gcEntry);
    display.display();

  } else {
    if (gbDisplayOn) {
      gbDisplayOn = false;
      display.displayOff();
    }
  }
}

/**
 * Send Current sensor data to monitors
 * - Only during stats cycle
*/
void sendSensorStats() {
  garageNode.setProperty(PROP_TEMP).send(gcTemp);
  garageNode.setProperty(PROP_HUM).send(gcHumid); 
  
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
 * Primary Sensor Task
 * - Runs all elements
*/
void homieLoopHandler() {
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

  gatherPosition(); // Interrupt Driven Handler

  if (guiTimeBase >= gulTempsDuration ) {    
    gatherTemps();
  }

  if (gvDuration) { // Every half second poll 
    displaySensors();
    gulLastTimeBase = guiTimeBase;
  }
}

void homieSetupHandler() {
  Homie.getLogger() << "homieSetupHandler() running on core: " << xPortGetCoreID() << endl;

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  delay(50);

  dht.setup(PIN_DHT, DHTesp::DHT11);
  delay(50);

  /*
   * Initialising the display.
   * create fonts at http://oleddisplay.squix.ch/
  */ 
  display.init();
  display.flipScreenVertically();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, SKN_MOD_TITLE);
  display.drawString(40, 16, SKN_MOD_VERSION);
  display.drawString(0, 32, "0123456789ABC");
  display.display();
  delay(50);

  /*
   * lower the return signal rate limit (default is 0.25 MCPS)
   * increase laser pulse periods (defaults are 14 and 10 PCLKs)
   * increase timing budget to 200 ms
   * Use continuous timed mode and provide the desired 
   * inter-measurement period in ms (e.g. sensor.startContinuous(250)).
  */
  lox.setTimeout(500);
  gbValue = lox.init();
  lox.setTimeout(500);
  if (!gbValue) { 
    Serial.println("Failed to detect and initialize VL53L0X sensor! (1)");
      digitalWrite(PIN_SHDN, LOW); 
      taskYIELD();
      delay(1000);
      digitalWrite(PIN_SHDN, HIGH); 
      taskYIELD();
    gbValue = lox.init(); // try again
  }
  if (!gbValue) {
    Serial.println("Failed to detect and initialize VL53L0X sensor! (2)");
    display.drawString(0, 48, "VL53L0X Failed!");
    display.display();
    delay(2000);
    ESP.restart();
  }
  delay(50);
  lox.setSignalRateLimit(0.1);
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  lox.setMeasurementTimingBudget(200000);
  lox.startContinuous(500); // total should be around 250ms per reading

  delay(50);
  digitalWrite(PIN_SHDN, LOW);   // H to enable, L to disable -- might also reset
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
  pinMode(PIN_SHDN, OUTPUT);      // VL53L0X Shutdown/Enable
  digitalWrite(PIN_RELAY, LOW);   // Init door to off
  digitalWrite(PIN_SHDN, HIGH);   // H to enable, L to disable -- might also reset

  /* Turn off Distance and Entry Readings */
  gulTempsDuration = gulEntryDuration = gulLoxDuration = millis(); // setDuration( 1000UL );

  FilteredPosition.Filter(2000);
  FilteredEntry.Filter(2000);

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
  
  Homie
      .setLoopFunction(homieLoopHandler)
      .setBroadcastHandler(broadcastHandler)
      .onEvent(onHomieEvent)  
      .setSetupFunction(homieSetupHandler);
  
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
