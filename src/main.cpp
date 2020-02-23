/**
 * Module: GarageDoor
 *  Data:  data/homie/config.json, 
 *         data/homie/ui_bundle.gz
 *  Created:  02/22/2020
 *  
 *  Description:
 *  At sensorsInterval (15 min) send Temperature, Humidity, 
 *  - Motion, EntryCount, and Door Position.
 *  When humans are detected send notice once per stats interval, interval will reset
 *  When analog change from saftey beam, record entry/exit.
 *  When position change by 4% broadcast door position until movement stops inside 4%
 *  - maybe decode moving up or down, and open/closed from home/max
 *  On handler command pulse Relay to open/close door opener
 */

#include <Arduino.h>
#include <Homie.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>

#include <Wire.h>
#include <VL53L0X.h>



#define SKN_MOD_NAME    "GarageDoorManager"
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

#define DHT_PIN           16
#define PIN_MOTION        17
#define PIN_L0X           18
#define PIN_RELAY         19
#define PIN_ENTRY         36
#define PIN_SDA           21
#define PIN_SCL           22

/*
 * I2c == 22, 21
*/
#define DHT_TYPE          DHT11

/*
 * Sensor Values
 */
volatile unsigned long guiTimeBase        = 0;
volatile unsigned long gulLastMotionRead  = 0;
volatile int      giEntry            = 0;
volatile int      giEntryValue       = 0;
volatile int      giLastEntryValue   = 0;

volatile bool     gbLOXReady         = false;
volatile bool     gvDuration         = false;
volatile bool     gvMotion           = LOW;  
volatile bool     gvLastMotion       = LOW;  
String            gsMotionString     = "false";

volatile float    gfTemperature      = 0.0f, 
                  gfHumidity         = 0.0f,
                  gfLOX              = 0.0f,
                  gfLastLOXValue     = 0.0f,
                  gfValue            = 0.0f;
char              gBuffer[48];             
char              gcDisplayBuf[48];
float             value = 0.0;

/*
 * Temperature Sensor
*/	
DHT dht(DHT_PIN, DHT_TYPE);

/*
 * Time of Flight Sensor
*/
VL53L0X lox;

HomieNode garageNode(SKN_MOD_NAME, NODE_NAME, SKN_MOD_BRAND);

void loxInterruptHandler()
{
  gbLOXReady = true; // set the new data ready flag to true on interrupt
}

bool broadcastHandler(const String& level, const String& value) {
  Homie.getLogger() << "Received broadcast level " << level << ": " << value << endl;
  return true;
}

bool operateDoor(const HomieRange& range, const String& value) {
  if (value == "ON" || value=="true" || value == "on") {
    digitalWrite(PIN_RELAY, HIGH );
    garageNode.setProperty(PROP_RELAY).send( "ON" );
    delay(250);
    digitalWrite(PIN_RELAY, LOW );
    garageNode.setProperty(PROP_RELAY).send( "OFF" );
  } 
  return true;
}

void handleLOX() {
  if (gbLOXReady) {
    gfValue =  lox.readRangeContinuousMillimeters();
    if (lox.timeoutOccurred()) { 
      Homie.getLogger() << "VL53L0X TIMEOUT" << endl;
    } else {
      Homie.getLogger() << "VL53L0X Reading: " << gfLOX << endl;
      gfLastLOXValue = gfLOX;
      gbLOXReady = false;

      snprintf(gBuffer, sizeof(gBuffer), "%.1f", gfLastLOXValue);
      garageNode.setProperty(PROP_POS).send(gBuffer);
    }
  }
}

void handleEntry() {
  Homie.getLogger() << "Entries: " << giEntryValue << endl;
  garageNode.setProperty(PROP_ENTRY).send(String(giEntryValue));                                        
  giLastEntryValue = 0;
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
      handleEntry();
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
    
    giEntry = analogRead(PIN_ENTRY); 
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

  handleLOX(); // Interrupt Driven Handler
}


void setupHandler() {
   dht.begin();
   Wire.begin(PIN_SDA, PIN_SCL, 400000);

  lox.setTimeout(500);
  if (!lox.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {} // todo find a better exit
  }

  // lower the return signal rate limit (default is 0.25 MCPS)
  lox.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
   // increase timing budget to 200 ms
  lox.setMeasurementTimingBudget(200000);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(250)).
  lox.startContinuous(250);
}

void setup() {
  delay(50);
  Serial.begin(115200);
  Serial << endl << endl;
  yield();

  pinMode(PIN_ENTRY, INPUT);     // analog sensor of IR door break 20.0vdc = off, 21.5vdc = on
  pinMode(PIN_MOTION, INPUT);    // RCWL sensor
  pinMode(PIN_RELAY, OUTPUT);    // Door operator
  digitalWrite(PIN_RELAY, LOW);  // Init door to off
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
            .settable(operateDoor)
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
