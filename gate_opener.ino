#include <FiniteStateMachine.h>

//**** Config section ****/
const float SLOW_START_TIME = 2.0;
const float MOVE_TIME = 6.0;
const float CLOSE_AFTER_TIME = 5.0;
const float MAX_AMPS = 200.0;

//**** Input output section ****/
const int OPEN_PIN = 10;
const int CLOSE_PIN = 11;
const int AUTO_BUTTON_PIN = 2;
const int MANUAL_OPEN_PIN = 3;
const int MANUAL_CLOSE_PIN = 4;
const int CURRENT_SENSOR_PIN = 1;
const int MAX_DUTY_CYCLE = 255;

//**** State machine and gate operation ****/
const byte NUMBER_OF_SELECATBLE_STATES = 5;
int movingPin;
int stoppedPin;
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

//**** Bump detection ****/
int recover_attempts = 0;
float bumped_object_time = 0;

//**** External Control ****/
bool stay_open = false;
bool manual_apply_brake = false;
int open_button_state = LOW;
int close_button_state = LOW;
int auto_button_state = LOW;

void gateIsClosedUpdate() {
  //Serial.println("gateIsClosedUpdate");
  recover_attempts = 0;
  auto_button_state = digitalRead(AUTO_BUTTON_PIN);

  // check if the pushbutton is pressed.
  if (auto_button_state == HIGH) {
    stateMachine.transitionTo(gateIsOpening);
  }
  manualMove();
}

void gateIsOpenUpdate(){
  //Serial.println("gateIsOpenUpdate");
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
    analogWrite(movingPin,MAX_DUTY_CYCLE);    
  }else if (close_button_state == HIGH) {
    //Manually Close gate while button held
    setClose();
    manual_apply_brake = true;
    analogWrite(movingPin,MAX_DUTY_CYCLE);
  }else if (close_button_state != HIGH && open_button_state != HIGH){
    if(manual_apply_brake) {
      setBrake();  
    }
  }
}

void setOpen(){
  movingPin = OPEN_PIN;
  stoppedPin = CLOSE_PIN;
  digitalWrite(stoppedPin, LOW);
}

void setClose(){
  movingPin = CLOSE_PIN;
  stoppedPin = OPEN_PIN;
  digitalWrite(stoppedPin, LOW);
}

void setBrake(){
  //Set pins to brake position
  analogWrite(movingPin, MAX_DUTY_CYCLE);
  analogWrite(stoppedPin, MAX_DUTY_CYCLE);
  //allow brake to take effect
  delay(150);
}

void moveRoutine(){
  bool closed = false;
  float timeInState = bumped_object_time > 0 ? MOVE_TIME - bumped_object_time + stateMachine.timeInCurrentState() : stateMachine.timeInCurrentState(); //If we bumped object, place gate back to prior position via timing
  float amps = readAmps();
  int duty_cycle = MAX_DUTY_CYCLE;
  
  if(timeInState < SLOW_START_TIME){
    duty_cycle = MAX_DUTY_CYCLE/SLOW_START_TIME*timeInState;    //convert to 1:MAX_DUTY_CYCLE duty cycle range based on time
    analogWrite(movingPin,duty_cycle);
  }else if(SLOW_START_TIME <= timeInState && timeInState <= (MOVE_TIME - SLOW_START_TIME)){
    analogWrite(movingPin,duty_cycle);
  }else if(SLOW_START_TIME <= timeInState && timeInState <= MOVE_TIME){
    duty_cycle = MAX_DUTY_CYCLE/SLOW_START_TIME*(MOVE_TIME - timeInState);    //convert to MAX_DUTY_CYCLE:1 duty cycle range based on time
    analogWrite(movingPin,duty_cycle);
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
  Serial.println("transitionOutOfMove");
  analogWrite(movingPin,LOW);
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
  current_readings[current_index] = analogRead(CURRENT_SENSOR_PIN);
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

void setup() {
  //Initialize gate to Open state
  Serial.begin(115200);
  Serial.println("GateBrain v 0.2");

  //Set up pins
  pinMode(movingPin, OUTPUT);
  pinMode(stoppedPin, OUTPUT);
  pinMode(AUTO_BUTTON_PIN, INPUT);
  pinMode(MANUAL_OPEN_PIN, INPUT);
  pinMode(MANUAL_CLOSE_PIN, INPUT);
  pinMode(CURRENT_SENSOR_PIN, INPUT);

  //Initialize current sensing array
  resetAmpReadings();
  
  //Get timings from EEPROM

  //Initialize current readings array
  
}

void loop() {
  stateMachine.update();
}
