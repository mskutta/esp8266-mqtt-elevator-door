//#define FRONT_DOOR 1
//#define REAR_DOOR 1

#ifdef ESP8266 || ESP32
#define ISR_PREFIX ICACHE_RAM_ATTR
#else
#define ISR_PREFIX
#endif

#include <Arduino.h>

#include <ESP8266WiFi.h> // WIFI support
#include <ArduinoOTA.h> // Updates over the air

// WiFi Manager
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> 

// I2C
#include <Wire.h>

// Display (SSD1306)
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// Stepper Motor Control
#include <Tic.h>

// Distance Sensors
#include <VL53L0X.h>

// Port Expander
#include <Adafruit_MCP23008.h>

// MQTT
#include <PubSubClient.h>

/* Constants */
#ifdef FRONT_DOOR
const char* ESP_NAME = "elev-door-front";
const char* DOOR_NAME = "front";
#endif
#ifdef REAR_DOOR
const char* ESP_NAME = "elev-door-rear";
const char* DOOR_NAME = "rear";
#endif

const int CLOSED_POSITION = 0;
const int OPEN_POSITION = 18600; // 18650

const int HIGH_ACCURACY_TIMING_BUDGET = 200000;
const int HIGH_SPEED_TIMING_BUDGET = 26000; // 20000 min

const unsigned long DOOR_DWELL_1 = 5000; // 3000
const unsigned long DOOR_DWELL_2 = 2000;

//#define XSHUT_pin3 not required for address change
const int SENSOR2_XSHUT_PIN = D3;
const int SENSOR3_XSHUT_PIN = D4;

const int SENSOR1_ADDRESS = 43;
const int SENSOR2_ADDRESS = 42;
//const int SENSOR3_ADDRESS = 41; // Default

/* Constants */
const unsigned long RUN_INTERVAL = 100;

/* Variables */
unsigned long nextRun = 0;
unsigned long currentMillis = 0;

enum class CallState {None, Up, Down};
static const char *CallStateString[] = {"none", "up", "down"};
CallState callState = CallState::None;
CallState lastCallState = CallState::None;

enum class DoorState {Unknown, Homing, Close, Closed, Closing, Open, Opening, Reopen, Reopening, Waiting};
static const char *DoorStateString[] = {"unknown", "homing", "close", "closed", "closing", "open", "opening", "reopen", "reopening", "waiting"};
DoorState doorState = DoorState::Unknown;
DoorState lastDoorState = DoorState::Unknown;

unsigned long homingTimeout = 0;
unsigned long closeTimeout = 0;
unsigned long openTimeout = 0;
unsigned long waitTimeout = 0;
bool isHomed = false;
bool lastLimitActive = false;

volatile int encoderCount = 0; //This variable will increase or decrease depending on the rotation of encoder
int lastEncoderPosition = 0;

bool doorOpenReceived = false;

/* Display */
SSD1306AsciiWire oled;
uint8_t rowHeight; // pixels per row.

/* WIFI */
char hostname[21] = {0};

/* MQTT */
WiFiClient wifiClient;
PubSubClient client(wifiClient);
const char* broker = "10.81.95.165";
char topic[40] = {0};
char topicDoorOpen[40] = {0};

/* TIC */
TicSerial tic(Serial);

/* VL53L0X */
VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

/* Port Expander */
Adafruit_MCP23008 mcp;

int lastRange1 = 0;
int lastRange2 = 0;
int lastRange3 = 0;

bool getTripped() {
  // any change > 100 is considered tripping

  bool tripped1 = false;
  int range1 = _max(sensor1.readRangeContinuousMillimeters(), 30);
  if (!sensor1.timeoutOccurred() && range1 < 1200) { 
    tripped1 = (range1 < lastRange1 - 100) || (range1 > lastRange1 + 100);
    lastRange1 = range1;
  }

  bool tripped2 = false;
  int range2 = _max(sensor2.readRangeContinuousMillimeters(), 30);
  if (!sensor2.timeoutOccurred() && range2 < 1200) { 
    tripped2 = (range2 < lastRange2 - 100) || (range2 > lastRange2 + 100);
    lastRange2 = range2;
  }

  bool tripped3 = false;
  int range3 = _max(sensor3.readRangeContinuousMillimeters(), 30);
  if (!sensor3.timeoutOccurred() && range3 < 1200) {
    tripped3 = (range3 < lastRange3 - 100) || (range3 > lastRange3 + 100);
    lastRange3 = range3;
  }

  bool tripped = tripped1 || tripped2 || tripped3;
  
  // Display Results if tripped
  if (tripped) {
    char buffer [5];
    
    oled.print("trip ");

    if (range1 < 1200) {
      sprintf (buffer, "%4d", range1);
      oled.print(buffer);
    } else {
      oled.print(F(" err"));
    }
    
    if (range2 < 1200) {
      sprintf (buffer, "%4d", range2);
      oled.print(buffer);
    } else {
      oled.print(F(" err"));
    }
    
    if (range3 < 1200) {
      sprintf (buffer, "%4d", range3);
      oled.println(buffer);
    } else {
      oled.println(F(" err"));
    }
  }
  
  return tripped;
}

void setEncoderPosition(int position) {
  encoderCount = map(position, 0, 1600, 0, 600); // Encoder 600p/r, Stepper 200steps/rev * 8
}

int getEncoderPosition() {
  return map(encoderCount, 0, 600, 0, 1600); // Encoder 600p/r, Stepper 200steps/rev * 8
}

void waitDoor(unsigned long timeout) {
  tic.deenergize();
  doorState = DoorState::Waiting;
  waitTimeout = millis() + timeout;
}

void setHighAccuracy() {
  // High Accuracy
  sensor1.setMeasurementTimingBudget(HIGH_ACCURACY_TIMING_BUDGET);
  sensor2.setMeasurementTimingBudget(HIGH_ACCURACY_TIMING_BUDGET);
  sensor3.setMeasurementTimingBudget(HIGH_ACCURACY_TIMING_BUDGET);
}

void setHighSpeed() {
  // High Speed
  sensor1.setMeasurementTimingBudget(HIGH_SPEED_TIMING_BUDGET);
  sensor2.setMeasurementTimingBudget(HIGH_SPEED_TIMING_BUDGET);
  sensor3.setMeasurementTimingBudget(HIGH_SPEED_TIMING_BUDGET);
}

void home() {
  tic.energize();
  tic.setStepMode(TicStepMode::Microstep8);
  tic.setCurrentLimit(1500);
  tic.setMaxAccel(100000); // 10000 steps/sec
  tic.setMaxDecel(100000); // 10000 steps/sec
  tic.haltAndSetPosition(0); // Needed to set velocity
  tic.exitSafeStart();

  tic.setTargetVelocity(-10000000);
  
  doorState = DoorState::Homing;
  homingTimeout = millis() + 5000; // Wait 5s
}

void homingTimedOut() {
  oled.println(F("Homimg Timeout"));
  oled.println(F("!!! CLODE DOOR !!!"));

  if (tic.getEnergized()) {
    tic.deenergize();
  }
  isHomed = false;
}

ISR_PREFIX void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(D6)==LOW) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void preCloseDoor() {
  // Allow the door to settle
  if (tic.getEnergized()) {
    tic.deenergize();
    closeTimeout = millis() + 250; // Wait 250ms
  }
  
  doorState = DoorState::Close;
}

void closeDoor() {
  tic.energize();
  tic.setStepMode(TicStepMode::Microstep8);
  tic.setCurrentLimit(900); // 750 is the minimum required to mostly open the door
  tic.setMaxSpeed(30000000); // ~11.25 revolutions (18000 steps) to open door. 2 second open time = 9000 steps/sec
  tic.setMaxAccel(100000); // 10000 steps/sec
  tic.setMaxDecel(100000); // 10000 steps/sec
  tic.haltAndSetPosition(getEncoderPosition());
  tic.setTargetPosition(CLOSED_POSITION);
  tic.exitSafeStart();
  
  doorState = DoorState::Closing;
}

void preOpenDoor(bool reopen) {
  if (tic.getEnergized()) {
    tic.deenergize();
    openTimeout = millis() + 250; // Wait 250ms
  }

  doorState = (reopen == true) ? DoorState::Reopen : DoorState::Open;
}

void openDoor(bool reopen) {
  tic.energize();
  tic.setStepMode(TicStepMode::Microstep8);
  tic.setCurrentLimit(1500);
#ifdef FRONT_DOOR
  tic.setMaxSpeed(90000000); // ~11.25 revolutions (18000 steps) to open door. 2 second open time = 9000 steps/sec
#endif
#ifdef REAR_DOOR
  if (reopen == true) {
    tic.setMaxSpeed(90000000);
  } else {
    tic.setMaxSpeed(20000000);
  }
#endif
  tic.setMaxAccel(400000); // 10000 steps/sec
  tic.setMaxDecel(700000); // 10000 steps/sec
  tic.haltAndSetPosition(getEncoderPosition());
  tic.setTargetPosition(OPEN_POSITION);
  tic.exitSafeStart();

  doorState = (reopen == true) ? DoorState::Reopening : DoorState::Opening;
}

void callUp(){
  callState = CallState::Up;
  mcp.digitalWrite(3, HIGH); // UP
  mcp.digitalWrite(1, LOW); // Down
}

void callDown(){
  callState = CallState::Down;
  mcp.digitalWrite(3, LOW); // UP
  mcp.digitalWrite(1, HIGH); // Down
}

void callNone(){
  callState = CallState::None;
  mcp.digitalWrite(3, LOW); // UP
  mcp.digitalWrite(1, LOW); // Down
}

void configModeCallback (WiFiManager *myWiFiManager) {
  oled.println(F("Config Mode"));
  oled.println(WiFi.softAPIP());
  oled.println(myWiFiManager->getConfigPortalSSID());
}

void reconnect() {
  while (!client.connected()) {
    oled.println(F("MQTT Connecting..."));
    if (client.connect(hostname)) {
      oled.println(F("MQTT connected"));
      client.subscribe(topicDoorOpen); // "elev-door-?/open"
    } else {
      oled.print(F("."));
      delay(1000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  //oled.print(F("t:"));
  oled.println(topic);
  //oled.print(F("p:"));

  //for (int i = 0; i < length; i++) {
  //  oled.print((char)payload[i]);
  //}
  //oled.println();

  if(strcmp(topic, topicDoorOpen) == 0) 
  {
    doorOpenReceived = true;
  }
}

void setup() {
  
  /* Serial and I2C */
  Serial.begin(9600);
  Wire.begin(D2, D1); // join i2c bus with SDA=D1 and SCL=D2 of NodeMCU

  delay(1000);

  /* Display */
  oled.begin(&Adafruit128x64, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  oled.setFont(System5x7);
  oled.setScrollMode(SCROLL_MODE_AUTO);
  oled.clear();
  rowHeight = oled.fontRows();

  /* Function Select */
  oled.println(ESP_NAME);
  
  /* WiFi */
  sprintf(hostname, "%s-%06X", ESP_NAME, ESP.getChipId());
  WiFiManager wifiManager;
  wifiManager.setAPCallback(configModeCallback);
  if(!wifiManager.autoConnect(hostname)) {
    oled.println("WiFi Connect Failed");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  oled.println(hostname);
  oled.print(F("  "));
  oled.println(WiFi.localIP());
  oled.print(F("  "));
  oled.println(WiFi.macAddress());

  /* OTA */
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    oled.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    oled.println(F("End"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    oled.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    oled.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      oled.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      oled.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      oled.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      oled.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      oled.println("End Failed");
    }
  });
  ArduinoOTA.begin();

  // Wait to view display
  delay(2000);
  
  /* TIC */
  // Set the TIC product
  tic.setProduct(TicProduct::T500);
  tic.deenergize();
 
  /* VL53L0X */
  // WARNING: Shutdown pins of VL53L0X ACTIVE-LOW-ONLY NO TOLERANT TO 5V will fry them
  pinMode(SENSOR2_XSHUT_PIN, OUTPUT);
  pinMode(SENSOR3_XSHUT_PIN, OUTPUT);

  // Change address of sensor and power up next one
  // Sensor 1
  sensor1.setAddress(SENSOR1_ADDRESS);
  
  // Sensor 2
  pinMode(SENSOR2_XSHUT_PIN, INPUT);
  delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"
  sensor2.setAddress(SENSOR2_ADDRESS);
  
  // Sensor 3
  pinMode(SENSOR3_XSHUT_PIN, INPUT);
  delay(10);
  
  sensor1.init();
  sensor2.init();
  sensor3.init();
  
  // High Speed
  setHighSpeed();
  
  sensor1.setTimeout(500);
  sensor2.setTimeout(500);
  sensor3.setTimeout(500);
  
  // Start continuous back-to-back mode (take readings as fast as possible).  
  sensor1.startContinuous();
  sensor2.startContinuous();
  sensor3.startContinuous();

  /* encoder */
  // Set up pins
  pinMode(D5, INPUT_PULLUP);
  pinMode(D6, INPUT_PULLUP);
  
  // Set up interrupts
  attachInterrupt(digitalPinToInterrupt(D5), ai0, RISING);

  /* limit switch */
  pinMode(D7, INPUT_PULLUP);

  /* Port Expander (MCP23008) */
  mcp.begin(0); // 0x20
  
  mcp.pinMode(0, INPUT); // Down Button
  mcp.pullUp(0, HIGH);  // turn on a 100K pullup internally
  
  mcp.pinMode(1, OUTPUT);  // Down Acceptance Light
  mcp.digitalWrite(1, LOW);
  
  mcp.pinMode(2, INPUT); // Up Button
  mcp.pullUp(2, HIGH);  // turn on a 100K pullup internally
  
  mcp.pinMode(3, OUTPUT);  // Up Acceptance Light
  mcp.digitalWrite(3, LOW);

#ifdef FRONT_DOOR
  mcp.pinMode(4, OUTPUT);  // Down Lanturn
  mcp.digitalWrite(4, LOW);
  
  mcp.pinMode(5, OUTPUT);  // Up Lanturn
  mcp.digitalWrite(5, LOW);
#endif
#ifdef REAR_DOOR
  mcp.pinMode(6, OUTPUT);  // Door Frame
  mcp.digitalWrite(6, HIGH);
  
  mcp.pinMode(7, OUTPUT);  // Door Frame
  mcp.digitalWrite(7, HIGH);
#endif

  /* MQTT */
  sprintf(topicDoorOpen, "%s/door/open", ESP_NAME);
  client.setServer(broker, 1883);
  client.setCallback(callback);

  // Wait to view display and connect
  delay(2000);

  oled.println("!!! CLOSE DOOR !!!");
}

void loop() {
  ArduinoOTA.handle();
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  tic.resetCommandTimeout();

  // limit switch
  bool limitActive = (digitalRead(D7)==LOW);
  if (limitActive == true && limitActive != lastLimitActive) {
    if (tic.getEnergized()) {
      tic.deenergize();
    }
    setEncoderPosition(0);
    doorState = DoorState::Closed;
    isHomed = true;
    oled.println(F("Limit Switch"));
  }
  lastLimitActive = limitActive;

  if (!isHomed) {
    oled.invertDisplay(true);
    return;
  } else {
    oled.invertDisplay(false);
  }

  // NOTE: for some reason the following (tic) needs to run at full speed for tic.getCurrentPosition to return reliable results.
  // i.e. do not add a delay before calling the below.

  // Get position info
  int currentPosition = tic.getCurrentPosition();
  int targetPosition = tic.getTargetPosition();
  
  // Run the remainder of the loop once every 100ms
  currentMillis = millis();
  if (currentMillis < nextRun) {
    return;
  }
  nextRun = currentMillis + RUN_INTERVAL;

  // Get position and tripped info
  int encoderPosition = getEncoderPosition();
  bool tripped = getTripped(); // TODO: May need to only retrieve if encoder position is > 1000

  // Indicate if range error
  if (tripped) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  // Deenergize if stopped
  bool stopped = currentPosition == targetPosition;
  if (stopped && tic.getEnergized()) {
    tic.deenergize();
  }
  
  // Position Correction
  int positionCorrection = currentPosition - encoderPosition;

  // Messages
  bool openDoorRequested = doorOpenReceived;
  doorOpenReceived = false;

  // Handle Door States
  if (doorState == DoorState::Waiting) {
    if (millis() > waitTimeout) {
      preCloseDoor();
    }
    else if (tripped || openDoorRequested || (encoderPosition != lastEncoderPosition)) {
      waitDoor(DOOR_DWELL_2);
    }
  }
  else if (doorState == DoorState::Homing) {
    if (millis() > homingTimeout) {
      homingTimedOut();
    }
  }
  else if (doorState == DoorState::Close) {
    if (millis() > closeTimeout) {
      closeDoor();
    }
  }
  else if (doorState == DoorState::Closing) {
    if (stopped) {
      // door needs to be homed. Door should not stop before hitting limit switch
      home();
    }
    else if ((tripped && encoderPosition > 1000) || // beam break - reopen
              positionCorrection < -64 || // door is being pushed - reopen
              openDoorRequested) { // open door requested
      if (positionCorrection < -64) {
        oled.print(F("drift: "));
        oled.println(positionCorrection);
      }
      preOpenDoor(true);
    }
    else if (positionCorrection > 64) { // door is being pulled - wait
      waitDoor(DOOR_DWELL_2);
    }
  }
  else if (doorState == DoorState::Open || doorState == DoorState::Reopen) {
    if (millis() > openTimeout) {
      if (doorState == DoorState::Open) {
        openDoor(false);
      } 
      else if (doorState == DoorState::Reopen) {
        openDoor(true);
      }
    }
  }
  else if (doorState == DoorState::Opening || doorState == DoorState::Reopening) {
    if (stopped || 
      encoderPosition > OPEN_POSITION) { // Door passed jam - wait
      //abs(positionCorrection) > 128) { // Door is being pushed or pulled - wait

#ifdef REAR_DOOR
      mcp.digitalWrite(6, HIGH); // Turn off EL wire
      mcp.digitalWrite(7, HIGH); // Turn off EL wire
#endif
      
      if(doorState == DoorState::Reopening) {
        waitDoor(DOOR_DWELL_2);
      } else {
        waitDoor(DOOR_DWELL_1);
      }
    }
  }
  else if (doorState == DoorState::Closed) {

#ifdef FRONT_DOOR
    // Read buttons if doors closed
    if (mcp.digitalRead(0) == LOW) { // Down Button
      callDown();
    }
    if (mcp.digitalRead(2) == LOW) { // Up Button
      callUp();
    }
#endif

    if (openDoorRequested) {
#ifdef FRONT_DOOR
      callNone();
#endif
#ifdef REAR_DOOR
      mcp.digitalWrite(6, LOW); // Turn on EL wire
      mcp.digitalWrite(7, LOW); // Turn on EL wire
#endif
      preOpenDoor(false);
    }
  }
  
  // Send call state
  if (callState != lastCallState) {
    oled.print(F("call: "));
    oled.println(CallStateString[(int)callState]);

    sprintf(topic, "%s/call/%s", ESP_NAME, CallStateString[(int)callState]);
    client.publish(topic, "");

    lastCallState = callState;
  }

  // Send door state
  if (doorState != lastDoorState) {
    oled.print(F("door: "));
    oled.println(DoorStateString[(int)doorState]);

    sprintf(topic, "%s/door/%s", ESP_NAME, DoorStateString[(int)doorState]);
    client.publish(topic, "");

    lastDoorState = doorState;
  }

  lastEncoderPosition = encoderPosition;
}


