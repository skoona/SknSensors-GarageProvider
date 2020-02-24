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
 */

#include <Arduino.h>

#include <driver/adc.h>

#include <Homie.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>

#include <Wire.h>
#include <VL53L0X.h>



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
                  giEntryValue       = 0,
                  giLastEntryValue   = 0;
volatile bool     gbLOXReady         = false,
                  gvDuration         = false,
                  gbValue            = false,
                  gvMotion           = false,  
                  gvLastMotion       = false;
volatile  uint16_t giLOX             = 0,
                  giValue            = 0,
                  giLastLOXValue     = 0;

volatile float    gfTemperature      = 0.0f, 
                  gfHumidity         = 0.0f,
                  gfValue            = 0.0f;

float             value = 0.0;
String            gsMotionString     = "false";
char              gBuffer[48],
                  gcDisplayBuf[48];

/*
 * Temperature Sensor
*/	
DHT dht(PIN_DHT, DHT_TYPE);

/*
 * Time of Flight Sensor
*/
VL53L0X lox;

HomieNode garageNode(SKN_MOD_NAME, NODE_NAME, SKN_MOD_BRAND);

void IRAM_ATTR loxInterruptHandler()
{
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

    digitalWrite(PIN_RELAY, LOW );
    garageNode.setProperty(PROP_RELAY).send( "ON" );
    delay(250);
    digitalWrite(PIN_RELAY, HIGH );
    garageNode.setProperty(PROP_RELAY).send( "OFF" );
  } 
  return true;
}

int smoothDoorPosition() { // average every 3 readings
  static uint16_t values[4] = {0,0,0,0};
  static uint16_t index = 0;

  uint16_t value =  lox.readRangeContinuousMillimeters(); // read data value
  if (!lox.timeoutOccurred()) {
    if (index < 3) {
      values[index++] = value;
      return 0;
    } else if (index > 2) { // third new value
      if (values[3] != 0) {
        giLastLOXValue = (uint16_t)(((uint32_t)(values[0] + values[1] + values[2] + values[3])) / 4);
      } else {
        giLastLOXValue = (uint16_t)(((uint32_t)(values[0] + values[1] + values[2])) / 3);
      }
      index = 0;    
      values[3] = giLastLOXValue;
      values[index++] = value;
      return giLastLOXValue;
    }
  }
  return 0; // no value
}

void handleLOX() {
  if (gbLOXReady) {
    giLOX = smoothDoorPosition();
    if (0 != giLOX) {
      Homie.getLogger() << "VL53L0X Reading: " << giLOX << endl;
      snprintf(gBuffer, sizeof(gBuffer), "%d", giLOX);
      garageNode.setProperty(PROP_POS).send(gBuffer);
    }
    gbLOXReady = false;
  }   
}

void handleEntryAndPosition() {
  Homie.getLogger() << "Entries: " << giEntryValue << endl;
  garageNode.setProperty(PROP_ENTRY).send(String(giEntryValue));                                        
  giLastEntryValue = 0;

  Homie.getLogger() << "Position: " << giLastLOXValue << endl;
  garageNode.setProperty(PROP_POS).send(String(giLastLOXValue));
}

void handleMotion(bool state) { // called on demand trigger once per status interval
  gvLastMotion = state;  
  gsMotionString = ((state == HIGH) ? "ON" : "OFF");
  garageNode.setProperty(PROP_MOTION).send(gsMotionString);      // might need .setRetain(true);
  Homie.getLogger() << "Motion: " << gsMotionString << endl;
}

void handleTemperature() {   // called every status interval 
  value = dht.readTemperature(true);
  if( !isnan(value) ){
   gfTemperature = value; 
   snprintf(gBuffer, sizeof(gBuffer), "%.1f", gfTemperature);
   Homie.getLogger() << "Temperature: " << gfTemperature << " °F" << endl;
   garageNode.setProperty(PROP_TEMP).send(gBuffer);
  
   value = dht.readHumidity();
   if( !isnan(value) ){
     gfHumidity = value;
     snprintf(gBuffer, sizeof(gBuffer), "%.1f", gfHumidity);
     Homie.getLogger() << "Humidity: " << gfHumidity << " %" << endl;
     garageNode.setProperty(PROP_HUM).send(gBuffer);
   }   
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
      break;      
    case HomieEventType::MQTT_READY:
      Serial << "MQTT connected" << endl;
      attachInterrupt(PIN_L0X, loxInterruptHandler, FALLING); 
      break;
    case HomieEventType::MQTT_DISCONNECTED:
      Serial << "MQTT disconnected, reason: " << (int8_t)event.mqttReason << endl;
      break;
    case HomieEventType::SENDING_STATISTICS:
      Serial << "Sending statistics" << endl;
      handleMotion(gvMotion);
      handleEntryAndPosition();
      handleTemperature();
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

  if (gvDuration) { // Every half second poll
    gvMotion = digitalRead(PIN_MOTION);
    gulLastMotionRead = guiTimeBase;
    
    /*
     * 4.9vdc produces a reading of 1058 ish */
    giEntry = adc1_get_raw(ADC1_CHANNEL_0); // analogRead(PIN_ENTRY); 
    if ( giEntry >= 900 && giEntry >= giLastEntryValue ) { // 20 to 21.5 volts, 
      giEntryValue++;  
      giLastEntryValue = giEntry;
      Homie.getLogger() << "Entry Reading: " << giEntry << endl;
    }
  }

  // call when present on poll interval and when newly present; on-demand
  // will be turned off by status cycle which runs every 15 minutes
  if ( (gvMotion && gvDuration) && (gvMotion != gvLastMotion) ) {  
    handleMotion(gvMotion);    
  }

  if (gbLOXReady && gulLoxDuration >= guiTimeBase) {
    handleLOX(); // Interrupt Driven Handler
  }
}

void setupHandler() {
   dht.begin();
   yield();
   
   Wire.begin(PIN_SDA, PIN_SCL, 400000);
   yield();

   delay(2000);

  lox.setTimeout(500);
  gbValue = lox.init();
  if (!gbValue)
  {
    Serial.println("Failed to detect and initialize sensor!");
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
  digitalWrite(PIN_RELAY, HIGH);  // Init door to off

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
            .setDatatype("float")
            .setRetained(true)
            .setFormat("%.1f")
            .setUnit("#");


  Homie.setup();
}

void loop() {
  Homie.loop();
}
