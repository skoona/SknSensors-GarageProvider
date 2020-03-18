/**
 * Module: GarageProvider
 *  Nodes: door, environmentMonitor
 *  Data:  data/homie/config.json, 
 *         data/homie/ui_bundle.gz
 *  Created:  02/22/2020
 *  
 *  Description:
 *  At sensorsInterval (15 min) send 
 *  - Temperature, Humidity, Motion, EntryCount, and Door Position.
 *  When humans are detected send notice once per stats interval, interval will reset
 *  When analog change from saftey beam, record entry/exit.
 *  When door operates broadcast door position change every 1/2 second for 60 seconds
 *  - maybe decode moving up or down, and open/closed from home/max
 *  On handler command pulse Relay to open/close door opener, and reset position timer
 *  Position uses an interrupt to read data, disabling VL53L1X when not in use.
 *  - OLED Display must be enabled and on for I2c to be stable for VL53L1X operations
 *  - also attach/detach interrupt with MQTT enablement/disablement
 * 
 * *   PIN Description    Supply  Trigger
 * =============================================================================
 * *    D7 RCWL-0516      5V      HIGH (3.3V)   if motion detected. 0V normally.
 * -    D7 Motion-Detected Interrupt          INTR-RISING
 * *    D5 RELAY          3.3V    HIGH
 * *    D6 DHT-11         5V      Data/OneWire  Temperature/Humidity
 * *    A0 Volt/Divider  21.5V    ADC           Sensor 100K/16 (120K/12K={21.5=1.955v}{20.0/1.818v})
 * * D1/D2 VL53L1X        3.3V    I2c           Line of Sight Ranger 4m max
 * -    D8 Data-Ready Interrupt   GPIO          INTR-FALLING 
 * -    D3 Shutdown Pin   3.3V    GPIO          High for normal Operations, can reset VL53L1X
 * * D1/D2 OLED 128x64    3.3V    I2c SSD1306   Display
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
    Updating VL53L1X                         @ 1.2.0          [Up-to-date]
 */

#include <Arduino.h>
#include <driver/adc.h>

#include <Homie.h>

#include <Wire.h>
#include "SSD1306Wire.h"
#include <VL53L1X.h>

#include "DHTesp.h"

#ifndef ESP32
#pragma message(THIS MODULE IS FOR ESP32 ONLY!)
#error Select ESP32 board.
#endif

#define SKN_MOD_NAME    "GarageProvider"
#define SKN_MOD_VERSION "0.9.0"
#define SKN_MOD_BRAND   "SknSensors"
#define SKN_MOD_TITLE   "Operations"

#define SKN_DOOR_NODE_NAME  "door"
#define SKN_DOOR_NODE_TITLE "Operations"

#define SKN_ENV_NODE_NAME "environmentMonitor"
#define SKN_ENV_NODE_TITLE "Multi-Sensor"

#define PROP_ENTRY        "entry"
#define PROP_ENTRY_TITLE  "Entries"
#define PROP_RELAY        "operator"
#define PROP_RELAY_TITLE  "Operator"
#define PROP_MOTION       "motion"
#define PROP_MOTION_TITLE "Motion"
#define PROP_TEMP         "temperature"
#define PROP_TEMP_TITLE   "Temperature"
#define PROP_HUM          "humdity"
#define PROP_HUM_TITLE    "Humidity"
#define PROP_POS          "positon"
#define PROP_POS_TITLE    "Position MM"
#define PROP_ACTION       "direction"
#define PROP_ACTION_TITLE "Travel Direction"
#define POSITION_OPEN      162 // mm, within 6 inches
#define POSITION_CLOSED   2280 // mm, outside 7.5 feet


#define PIN_DHT           19
#define PIN_MOTION        23
#define PIN_L0X_INTR      5 
#define PIN_L0X_SHDN      16
#define PIN_RELAY         18
#define PIN_SDA           21
#define PIN_SCL           22
#define PIN_ENTRY         36

#define DHT_TYPE          DHT11
#define ENTRY_VOLTAGE_BASE 0.065440348345433     // correction for voltage divider

/*
 * Sensor Values
 */
volatile unsigned long guiTimeBase   = 0,     // default time base, target 0.5 seconds
                  gulLastTimeBase    = 0,     // time base delta
                  gulTempsDuration   = 0,     // time between each temp measurement
                  gulMotionDuration  = 0,     // time to hold motion after trigger interrupt
                  gulRangingDuration = 0,     // time to run VL53L1X after door operates
                  gulEntryDuration   = 0;     // time to run beam break monitor after door operates

volatile int      giEntry            = 0,     // value of beam break ADC
                  giEntryMax         = 2375,  // computed range 2426: 21.5v open value of ADC reading -- break
                  giEntryMin         = 2200,  // computed value 2256: 20.0v close value of ADC reading -- no break
                  giEntryCount       = 0,     // count of number of entries since startup
                  giLastEntry        = 10;    // ADC reading of last entry    

volatile bool     gbLOXReady         = true,  // VL53L1X startup -- Interrupt
                  gbLOXRunMode       = true,  // VL53L1X startup 
                  gbDisplayOn        = true,  // Display SHOULD be on or off    
                  gbDisplayInitialized = false, // Has display be setup? Alerts for HomieEvents
                  gvDuration         = false, // 1/2 second marker
                  gbValue            = false, // initialization boolean, general use
                  gvMotion           = false, // Motion indicator value -- Interrupt
                  gvLastMotion       = false; // Previous Motion indicator

volatile  uint16_t giLOX             = 0,     // VL53L1X current reading
                  giLastLOXValue     = 0;     // VL53L1X Previous reading

volatile float    gfTemperature      = 0.0f,  // Current Temperature value
                  gfHumidity         = 0.0f;  // Current Humidity value

char              gcTemp[48],                 // Current Temperature Formatted
                  gcTemps[48],                // Combined Temperature and Humidity Formatted
                  gcHumid[48],                // Current Humidity Formatted
                  gcPosition[48],             // Current VL53L1X Positon Formatted
                  gcDirection[48],            // Direction of Door Travel
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
VL53L1X lox;

/* 
 * Two Homie Nodes to reflect device activity
 * - Door Operations and Status
 * - Environment and Human Presence
*/
HomieNode doorNode(SKN_DOOR_NODE_NAME, SKN_DOOR_NODE_TITLE, SKN_MOD_BRAND);
HomieNode envNode(SKN_ENV_NODE_NAME, SKN_ENV_NODE_TITLE, SKN_MOD_BRAND);

/*
 * Utility to handle Duration Roll Overs
*/
unsigned long IRAM_ATTR setDuration(unsigned long duration) {
  unsigned long value = millis() + duration;
  if (value < duration) { // rolled
    value = duration;
  } 
  return value;
}

void IRAM_ATTR motionInterruptHandler() {
  gulMotionDuration = setDuration( 600000UL ); // Wait 10 min for an entry
  gvMotion = true;
}

void IRAM_ATTR loxInterruptHandler() {
  gbLOXReady = true; // set the new data ready flag to true on interrupt
}

bool broadcastHandler(const String& level, const String& value) {
  Homie.getLogger() << "Received broadcast level " << level << ": " << value << endl;
  return true;
}

/** 
 *  Re Enable VL53L1X after havin been ShutDown
*/
void enableRanging(unsigned long durationMS) {
  Homie.getLogger() << "Restoring VL53L1X ranging. " << endl;
  digitalWrite(PIN_L0X_SHDN, HIGH);   // H to enable, L to disable -- might also reset
  gbLOXReady = false;

  lox.setDistanceMode( VL53L1X::Long );
  lox.setMeasurementTimingBudget( 150000 );
  lox.startContinuous( 150 ); // total should be around 250ms per reading

  // Homie.getLogger() << "VL53L1X getDistanceMode()=" <<  lox.getDistanceMode()  << endl;
  // Homie.getLogger() << "VL53L1X getMeasurementTimingBudget()=" <<  lox.getMeasurementTimingBudget()  << endl;

  gulRangingDuration = setDuration( durationMS );  // LOX read for 60 seconds   
  gbLOXRunMode   = true;                       // Release Ranging data collection routines
  Homie.getLogger() << "VL53L1X ranging enabled for " << (durationMS/1000.0) << " seconds." << endl;
  delay(300);
}

/**
 * When door is operated release the Entry counter and the Distance Ranger
*/
bool operationsHandler(const HomieRange& range, const String& value) {
  Homie.getLogger() << "Operator Command: " << value << endl;
  if (value == pchON || value == "true" || value == "on") {
    gulEntryDuration = setDuration( 900000UL ); // Wait 15 min for an entry

    enableRanging(60000UL);                  // Enable Ranging

    digitalWrite(PIN_RELAY, HIGH );     // activate door relay
      delay(750);
    digitalWrite(PIN_RELAY, LOW );      // de-activate door relay

    // yield(); 
    taskYIELD();
    doorNode.setProperty(PROP_RELAY).send( pchOFF );

    Homie.getLogger() << "Exit ON operation. " << endl;
  } 
  return true;
}

/**
 * Read on Interrupt Ready set gbLOXReady
 * - Read until time expires
 * - Time controlled by caller and/or Door Operator
 */
void gatherPosition() {
  if ( gbLOXReady && (guiTimeBase >= gulRangingDuration)) {
    if (gbLOXRunMode) {
      digitalWrite(PIN_L0X_SHDN, LOW);   // H to enable, L to disable -- might also reset
      gbLOXRunMode = false;
    }
    gbLOXReady = false;
    Homie.getLogger() << F("VL53L1X OFF~> Read: ") << giLastLOXValue 
                      << ", Raw: " << giLOX 
                      << endl;      
    taskYIELD();
    return;
  } 

  if (gbLOXReady) {
    giLOX = lox.read(); // read data value
    if (!lox.timeoutOccurred() && giLOX > 1 && giLOX < 8100) {
                              
      if (giLOX < giLastLOXValue) { // moving up
        snprintf(gcDirection, sizeof(gcDirection), "%s", "OPENING" );
      } else if (giLOX > giLastLOXValue) { // moving down
        snprintf(gcDirection, sizeof(gcDirection), "%s", "CLOSING" );
      } else if (giLOX < POSITION_OPEN  ) { // open
        snprintf(gcDirection, sizeof(gcDirection), "%s", "OPEN" );
      } else if (giLOX > POSITION_CLOSED) { // closed
        snprintf(gcDirection, sizeof(gcDirection), "%s", "CLOSED" );
      }

      doorNode.setProperty(PROP_POS).send(String(giLastLOXValue));
      doorNode.setProperty(PROP_ACTION).send(gcDirection);

      Homie.getLogger() << F("VL53L1X Read: ") << giLastLOXValue 
                        << ", Raw: " << giLOX 
                        << ", Direction: " << gcDirection
                        << endl;      

      giLastLOXValue = giLOX;
      gbLOXReady = false;
      
    } else {
      Homie.getLogger() << F("VL53L1X Timeout or Out of Range: ") << lox.last_status  << endl;
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
double sknAdcToVolts(int value) {
  if(value < 1 || value > 4095) {
    return 1.0;
  }

  double reading = value * 1.0; // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095

  reading =  -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
  return ((reading + ENTRY_VOLTAGE_BASE) * 10.0); // scale to d.dd
}

void gatherEntry( ) {
  /*
    * 4.9vdc produces a reading of 1058 ish 
    * should run inside doorPosition min/max limits   */    
  giEntry = adc1_get_raw(ADC1_CHANNEL_0);
  if ( giEntry != giLastEntry ) { // 20 to 21.5 volts, 
    if (giEntry >= giEntryMax) {
      giEntryCount++;  
      doorNode.setProperty(PROP_ENTRY).send(String(giEntryCount));                                        
      Homie.getLogger() << "Entries: " << giEntryCount 
                        << ", Volts: " << sknAdcToVolts(giEntry)
                        << ", Raw: " << giEntry 
                        << ", Min: " << giEntryMin
                        << ", Max: " << giEntryMax << endl;
    }
    giEntryMax = giEntry > giEntryMax ? giEntry : giEntryMax ;
    giEntryMin = giEntry < giEntryMin ? giEntry : giEntryMin ;
    giLastEntry = giEntry;  
  }
  yield();
}

/**
 * Interrupt Driven turns on gvMotion
 * - ON with Interrupt, duration set by interrupt
 * - OFF with after duration ends
*/
void gatherMotion() { 
  if (gvLastMotion != gvMotion) {
    envNode.setProperty(PROP_MOTION).send((gvMotion ? pchON : pchOFF));
    Homie.getLogger() << (gvMotion ? pchmotionON : pchmotionOFF) << endl;
    gvLastMotion = gvMotion;
  } 
  if ( gvMotion && gulMotionDuration < guiTimeBase ) { 
      gvLastMotion = gvMotion = false;
      envNode.setProperty(PROP_MOTION).send(pchOFF);
      Homie.getLogger() << pchmotionOFF << endl;
  }

  yield();
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

    snprintf(gcTemp, sizeof(gcTemp), "%.1f", gfTemperature);
    envNode.setProperty(PROP_TEMP).send(gcTemp);
  
    snprintf(gcHumid, sizeof(gcHumid), "%.1f", gfHumidity);
    envNode.setProperty(PROP_HUM).send(gcHumid);      

    snprintf(gcTemp, sizeof(gcTemp), "%.1f °F, %.1f %%", gfTemperature, gfHumidity);
    Homie.getLogger() << gcTemp << endl;
  }     
  gulTempsDuration = setDuration( 600000UL ); // Wait 10 min for next measurement
  yield();
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
    
    snprintf(gcEntry, sizeof(gcEntry), "C:%d Vdc:%.2f", giEntryCount, sknAdcToVolts(giEntry));
    snprintf(gcTemps, sizeof(gcTemps), "%.1f °F, %.1f %%", gfTemperature, gfHumidity);
    snprintf(gcPosition, sizeof(gcPosition), "P:%hd R:%hd", giLastLOXValue, giLOX );

    display.clear();
      display.drawString(0,0, gcPosition);
      display.drawString(0,16, gcTemps);
      display.drawString(0,32, (gvMotion ? pchmotionON : pchmotionOFF));
      display.drawString(0,48, gcEntry);
    display.display();

  } else {
    if ( gbDisplayOn && !gbLOXRunMode) {
      gbDisplayOn = false;
      display.displayOff();
    }
  }
  yield();
}

/**
 * Send Current sensor data to monitors
 * - Only during stats cycle
*/
void sendSensorStats() {
  envNode.setProperty(PROP_TEMP).send(gcTemp);
  envNode.setProperty(PROP_HUM).send(gcHumid); 
  
  envNode.setProperty(PROP_MOTION).send((gvMotion ? pchON : pchOFF)); 
  doorNode.setProperty(PROP_ENTRY).send(String(giEntryCount));  
  doorNode.setProperty(PROP_POS).send(String(giLastLOXValue));
  doorNode.setProperty(PROP_ACTION).send(gcDirection);

  yield();
}

void displayEvent(String eventName, String eventMessage) {
  if (gbDisplayInitialized) {
    if(!gbDisplayOn) {
      display.displayOn();
      gbDisplayOn = true;
    }

    display.clear();
      display.drawString(0,0, eventName);
      display.drawString(0,16, eventMessage);
    display.display();
  }
  yield();
}

void onHomieEvent(const HomieEvent& event) {
  switch (event.type) {
    case HomieEventType::NORMAL_MODE:
      Serial << "Normal mode started, core: " << xPortGetCoreID() << endl;      
      break;
    case HomieEventType::WIFI_DISCONNECTED:
      Serial << "Wi-Fi disconnected, reason: " << (int8_t)event.wifiReason << endl;
      displayEvent( "Wi-Fi disconnected", String((int8_t)event.wifiReason));
      break;      
    case HomieEventType::MQTT_READY:
      Serial << "MQTT connected, core: " << xPortGetCoreID() << endl;
      displayEvent( "MQTT", "READY");
      attachInterrupt(PIN_L0X_INTR, loxInterruptHandler, FALLING); 
      attachInterrupt(PIN_MOTION, motionInterruptHandler, RISING); 
      break;
    case HomieEventType::MQTT_DISCONNECTED:
      Serial << "MQTT disconnected, reason: " << (int8_t)event.mqttReason << endl;
      displayEvent( "MQTT disconnected", String((int8_t)event.mqttReason));
      detachInterrupt(PIN_L0X_INTR); 
      detachInterrupt(PIN_MOTION); 
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

  if ( gulEntryDuration >= guiTimeBase  ) {   // when enabled by operator
    gatherEntry(); 
  }

  if ( guiTimeBase <= gulMotionDuration ) {  // duration hold the current value Interrupt Driven Handler
    gatherMotion();    
  }

  gatherPosition(); // Interrupt Driven Handler

  if (gulTempsDuration <= guiTimeBase ) {    
    gatherTemps();
  }

  if (gvDuration) { // Every half second poll 
    displaySensors();
    gulLastTimeBase = guiTimeBase;
  }
  yield();
}

 /*
  * Initialising the display.
  * create fonts at http://oleddisplay.squix.ch/
*/ 
void initializeDisplay() {
  display.init();
  display.flipScreenVertically();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, SKN_MOD_TITLE);
  display.drawString(40, 16, SKN_MOD_VERSION);
  display.drawString(0, 32, "0123456789ABC");
  display.display();
  gbDisplayInitialized = true;
  yield();
}

/**
 * lower the return signal rate limit (default is 0.25 MCPS)
 * increase timing budget to 200 ms
 * inter-measurement period in ms (e.g. sensor.startContinuous(250)).
*/
bool initializeRanging() {
  lox.setTimeout(500);
  gbValue = lox.init();
  lox.setTimeout(500);
  if (!gbValue) { 
    Serial.println("Failed to detect and initialize VL53L1X sensor! (1)");
      digitalWrite(PIN_L0X_SHDN, LOW); 
      taskYIELD();
      delay(1000);
      digitalWrite(PIN_L0X_SHDN, HIGH); 
      taskYIELD();
    gbValue = lox.init(); // try again
  }
  if (!gbValue) {
    Serial.println("Failed to detect and initialize VL53L1X sensor! (2)");
    display.drawString(0, 48, "VL53L1X Failed!");
    display.display();
    delay(5000);
    ESP.restart();
  }

  enableRanging(2000UL);                // Enable Ranging
  delay(500);

  return gbValue;
}

void homieSetupHandler() {
  Homie.getLogger() << "homieSetupHandler() running on core: " << xPortGetCoreID() << endl;

  Wire.begin(PIN_SDA, PIN_SCL, 400000UL);
  delay(50);

  dht.setup(PIN_DHT, DHTesp::DHT11);
  delay(50);

  initializeDisplay();
  initializeRanging();
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
  pinMode(PIN_L0X_INTR, INPUT);   // VL53L1X Interrupt Ready Pin
  pinMode(PIN_RELAY, OUTPUT);     // Door operator
  pinMode(PIN_L0X_SHDN, OUTPUT);  // VL53L1X Shutdown/Enable
  digitalWrite(PIN_RELAY, LOW);   // Init door to off
  digitalWrite(PIN_L0X_SHDN, HIGH);   // H to enable, L to disable -- might also reset

  /* Collect a few values for temps, entry. */
  gulTempsDuration = gulEntryDuration = setDuration( 2000UL );

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
  
  doorNode.advertise(PROP_RELAY)
            .setName(PROP_RELAY_TITLE)
            .setRetained(true)
            .settable(operationsHandler)
            .setDatatype("boolean")
            .setFormat("%s");

  doorNode.advertise(PROP_ENTRY)
            .setName(PROP_ENTRY_TITLE)
            .setDatatype("integer")
            .setRetained(true)
            .setFormat("%d")
            .setUnit("#");

  doorNode.advertise(PROP_POS)
            .setName(PROP_POS_TITLE)
            .setDatatype("integer")
            .setRetained(true)
            .setFormat("%d")
            .setUnit("#");

  doorNode.advertise(PROP_ACTION)
            .setName(PROP_ACTION_TITLE)
            .setDatatype("string")
            .setRetained(true)
            .setFormat("%s");

  envNode.advertise(PROP_MOTION)
            .setName(PROP_MOTION_TITLE)
            .setRetained(true)
            .setDatatype("string")
            .setFormat("%s");

  envNode.advertise(PROP_TEMP)
            .setName(PROP_TEMP_TITLE)
            .setDatatype("float")
            .setFormat("%.1f")
            .setUnit("ºF");

  envNode.advertise(PROP_HUM)
            .setName(PROP_HUM_TITLE)
            .setDatatype("float")
            .setFormat("%.1f")
            .setUnit("%");  

  Homie.setup();
}

void loop() {
  Homie.loop();
  yield();
}
