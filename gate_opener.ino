#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>

#include <FiniteStateMachine.h>

//**** MQTT ****/
const char* mqtt_server = "192.168.0.112";
const char* mqtt_username = "***";
const char* mqtt_password = "**";

//**** WiFi ****/
const char* ssid = "**";
const char* password = "**";

//**** Config section ****/
const float SLOW_START_TIME = 2.0;
float MOVE_TIME = 10.0;
float CLOSE_AFTER_TIME = 35.0;
const float MAX_AMPS = 300.0;

//**** Input output section ****/
const int OPEN_PIN = D1;
const int CLOSE_PIN = D2;
const int AUTO_BUTTON_PIN = D5;
const int MANUAL_OPEN_PIN = D6;
const int MANUAL_CLOSE_PIN = D7;
const int CURRENT_SENSOR_PIN = A0;
const int MAX_DUTY_CYCLE = 1024;

//**** State machine and gate operation ****/
const byte NUMBER_OF_SELECATBLE_STATES = 5;
int moving_pin;
int stopped_pin;
void gateIsClosedUpdate();
void setOpen();
void moveRoutine();
void setBrake();
void gateIsOpenUpdate();
void setClose();
void currentOverloadUpdate();
State gateIsClosed = State(gateIsClosedUpdate);
State gateIsOpening = State(setOpen, moveRoutine, setBrake);
State gateIsOpen = State(gateIsOpenUpdate);
State gateIsClosing = State(setClose, moveRoutine, setBrake);
State currentOverload = State(setBrake, currentOverloadUpdate, setBrake);
FSM stateMachine = FSM(gateIsClosed); //initialize state machine, start in state: gateIsClosed

//**** Current sensing ****/
const int SMOOTHING_SIZE = 10;
int current_readings[SMOOTHING_SIZE];
int current_index = 0;
int current_sum = 0;
int ignore_amps_buffer = 0;
float readAmps();

//**** Bump detection ****/
int recover_attempts = 0;
float bumped_object_time = 0;

//**** External Control ****/
bool stay_open = false;
bool manual_apply_brake = false;
int open_button_state = LOW;
int close_button_state = LOW;
int auto_button_state = LOW;
void manualMove();

/*Wifi*/
WiFiClient espClient;
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  randomSeed(micros());

   // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

/*MQTT*/
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected to mqtt");
      client.subscribe("gate/#");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // If we got an open message, transition state machine
  if (strcmp(topic, "gate/Cycle") == 0) {
    payload[length] = '\0';
    String s = String((char*)payload);
    CLOSE_AFTER_TIME = s.toFloat();
    stay_open = false;
    stateMachine.transitionTo(gateIsOpening);
  }
  if (strcmp(topic, "gate/Open") == 0) {
    stay_open = true;
    stateMachine.transitionTo(gateIsOpening);
  }
  if (strcmp(topic, "gate/Close") == 0) {
    stay_open = false;
  }

  if (strcmp(topic, "gate/SetMoveTime") == 0) {
    payload[length] = '\0';
    String s = String((char*)payload);
    MOVE_TIME = s.toFloat();
    setMoveTime(MOVE_TIME);
  }
}

void gateIsClosedUpdate() {
  //Serial.println("closed update");
  recover_attempts = 0;
  auto_button_state = digitalRead(AUTO_BUTTON_PIN);
  //Serial.println(digitalRead(MANUAL_OPEN_PIN));

  // check if the pushbutton is pressed.
  if (auto_button_state == HIGH) {
    stateMachine.transitionTo(gateIsOpening);
  }
  manualMove();
}

void gateIsOpenUpdate(){
  if(stateMachine.timeInCurrentState() > CLOSE_AFTER_TIME && recover_attempts == 0 && !stay_open){
    stateMachine.transitionTo(gateIsClosing);
  }
}

void currentOverloadUpdate(){
  Serial.println("Stalled please reset gate position manually");
  manualMove();
}

void manualMove(){
  open_button_state = digitalRead(MANUAL_OPEN_PIN);
  close_button_state = digitalRead(MANUAL_CLOSE_PIN);
  // check if the pushbutton is pressed.
  if (open_button_state == HIGH) {
    //Manually Open Gate while button held
    setOpen();
    manual_apply_brake = true;
    analogWrite(moving_pin,MAX_DUTY_CYCLE);   
  }else if (close_button_state == HIGH) {
    //Manually Close gate while button held
    setClose();
    manual_apply_brake = true;
    analogWrite(moving_pin,MAX_DUTY_CYCLE);
  }else if (close_button_state != HIGH && open_button_state != HIGH){
    if(manual_apply_brake) {
      setBrake();  
    }
  }
}

void setOpen(){
  moving_pin = OPEN_PIN;
  stopped_pin = CLOSE_PIN;
  digitalWrite(stopped_pin, LOW);
}

void setClose(){
  moving_pin = CLOSE_PIN;
  stopped_pin = OPEN_PIN;
  digitalWrite(stopped_pin, LOW);
}

void setBrake(){
  //Set pins to brake position
  analogWrite(moving_pin, MAX_DUTY_CYCLE);
  analogWrite(stopped_pin, MAX_DUTY_CYCLE);
  //allow brake to take effect
  delay(150);
}

void moveRoutine(){
  bool closed = false;
  float time_in_state = bumped_object_time > 0 ? MOVE_TIME - bumped_object_time + stateMachine.timeInCurrentState() : stateMachine.timeInCurrentState(); //If we bumped object, place gate back to prior position via timing
  float amps = readAmps();
  int duty_cycle = MAX_DUTY_CYCLE;
  
  if(time_in_state < SLOW_START_TIME){
    duty_cycle = MAX_DUTY_CYCLE/SLOW_START_TIME*time_in_state;    //convert to 1:MAX_DUTY_CYCLE duty cycle range based on time
    analogWrite(moving_pin,duty_cycle);
  }else if(SLOW_START_TIME <= time_in_state && time_in_state <= (MOVE_TIME - SLOW_START_TIME)){
    analogWrite(moving_pin,duty_cycle);
  }else if(SLOW_START_TIME <= time_in_state && time_in_state <= MOVE_TIME){
    duty_cycle = MAX_DUTY_CYCLE/SLOW_START_TIME*(MOVE_TIME - time_in_state);    //convert to MAX_DUTY_CYCLE:1 duty cycle range based on time
    analogWrite(moving_pin,duty_cycle);
    if(amps > MAX_AMPS){
      closed = true;
      resetAmpReadings();
      transitionOutOfMove();
    }
  }else{
    //time is up, transition out
    bumped_object_time = 0;
    transitionOutOfMove();
  }
  
  if(amps > MAX_AMPS && !closed){
    Serial.println("Gate bumped object!!");
    bumped_object_time = stateMachine.timeInCurrentState();
    setBrake();
    if(recover_attempts >= 1){
      stateMachine.transitionTo(currentOverload);
    }else{
      recover_attempts++;
      resetAmpReadings();
      ignore_amps_buffer = 1000;
      if(stateMachine.isInState(gateIsOpening)){
        stateMachine.transitionTo(gateIsClosing);
      }else{
        stateMachine.transitionTo(gateIsOpening);
      }
    }  
  }
}

void transitionOutOfMove(){
  analogWrite(moving_pin,LOW);
  if(stateMachine.isInState(gateIsOpening)){
    stateMachine.transitionTo(gateIsOpen);
  }else{
    stateMachine.transitionTo(gateIsClosed);
  }
}

float readAmps(){
  if(ignore_amps_buffer > 0){
    ignore_amps_buffer--;
    return 0;
  }
  current_sum = current_sum - current_readings[current_index];
  current_readings[current_index] = analogRead(A0);
  current_sum = current_sum + current_readings[current_index];  
  current_index++;
  current_index = current_index >= SMOOTHING_SIZE ? 0 : current_index;
  
  return current_sum / SMOOTHING_SIZE;
}

void resetAmpReadings(){
  memset(current_readings, 0, sizeof(current_readings));
  current_index = 0;
  current_sum = 0;
}

void setMoveTime(float time){
   Serial.println(time);
  uint addr = 0;
  struct { 
    float val = 0;
    char label[20] = "";
  } data;
  data.val = time;
  strncpy(data.label, "MOVE_TIME",20);
  EEPROM.put(addr,data);
  EEPROM.commit(); 
}

void setup() {
  //Initialize gate to Open state
  Serial.begin(9600);
  Serial.println("GateBrain v 0.2");
  
  //Set up pins
  pinMode(moving_pin, OUTPUT);
  pinMode(stopped_pin, OUTPUT);
  pinMode(AUTO_BUTTON_PIN, INPUT);
  pinMode(MANUAL_OPEN_PIN, INPUT);
  pinMode(MANUAL_CLOSE_PIN, INPUT);
  pinMode(CURRENT_SENSOR_PIN, INPUT);

  //Initialize current sensing array
  resetAmpReadings();
  
  //Get timings from EEPROM
  uint addr = 0;
  struct { 
    float val = 0;
    char label[20] = "";
  } data;

  EEPROM.begin(512);

  EEPROM.get(addr,data);
  Serial.println("Move Time Values are: " + String(data.val) + "," + String(data.label));

  if ( strcmp(data.label,"MOVE_TIME") == 0 )
  {    
     MOVE_TIME = data.val;
  } 

//  data.val = 0; 
//  strncpy(data.label,"",20);

  //Connect to wifi
  setup_wifi();
  //MQTT connect
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  ArduinoOTA.handle();
  stateMachine.update();
}
