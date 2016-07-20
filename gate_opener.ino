#include <FiniteStateMachine.h>

//**** Config section ****/
const float SLOW_START_TIME = 1.0;
const float MOVE_TIME = 10.0;
const float CLOSE_AFTER_TIME = 20.0;
const float MAX_AMPS = 3.0;

//**** Input output section ****/
const int OPEN_PIN = 10;
const int CLOSE_PIN = 11;
const int BUTTON_PIN = 2;
const int CURRENT_SENSOR_PIN = 4;


//**** State machine and gate operation ****/
const byte NUMBER_OF_SELECATBLE_STATES = 4;
int movingPin;
int stoppedPin;
int operateAttempts = 0;
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



float readAmps(){
  current_sum = current_sum - current_readings[current_index];
  current_readings[current_index] = analogRead(CURRENT_SENSOR_PIN);
  current_sum = current_sum + current_readings[current_index];
  current_index++;
  current_index = current_index >= SMOOTHING_SIZE ? 0 : current_index;
  return current_sum / SMOOTHING_SIZE;
}

void gateIsClosedUpdate() {
  Serial.println("gateIsClosedUpdate");
  int buttonState = digitalRead(BUTTON_PIN);

  // check if the pushbutton is pressed.
  if (buttonState == HIGH) {
    stateMachine.transitionTo(gateIsOpening);
  }else{
    analogWrite(movingPin, LOW);
    analogWrite(stoppedPin, LOW);
  }
}

void gateIsOpenUpdate(){
  Serial.println("gateIsOpenUpdate");
  if(stateMachine.timeInCurrentState() > CLOSE_AFTER_TIME){
    stateMachine.transitionTo(gateIsClosing);
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
  Serial.println("setBrake");
  //Set pins to brake position
  analogWrite(movingPin, 255);
  analogWrite(stoppedPin, 255);
  //allow brake to take effect
  delay(150);
}

void moveRoutine(){
  bool closed = false;
  float timeInState = stateMachine.timeInCurrentState();
  float amps = readAmps();
  
  if(timeInState < SLOW_START_TIME){
    //Serial.println("slowStart");
    //convert to 1-255 duty cycle
    int duty_cycle = 255/SLOW_START_TIME*timeInState;
    analogWrite(movingPin,duty_cycle);
  }else if(SLOW_START_TIME < timeInState && timeInState < (MOVE_TIME - SLOW_START_TIME)){
    //Serial.println("operate operator");
    analogWrite(movingPin,255);
  }else if(SLOW_START_TIME < timeInState && timeInState < MOVE_TIME){
    //Serial.println("slow stop");
    //convert to 255-1 duty cycle
    int duty_cycle = 255/SLOW_START_TIME*(MOVE_TIME - timeInState);
    analogWrite(movingPin,duty_cycle);
    if(amps > MAX_AMPS){
      closed = true;
      transitionOutOfMove();
    }
  }else{    
    //Serial.println("stop gate and move to next state");
    transitionOutOfMove();
  }
  
  if(amps > MAX_AMPS && !closed){
    operateAttempts++;
    if(operateAttempts > 1){
      stateMachine.transitionTo(currentOverload);
    }
    if(stateMachine.isInState(gateIsOpening)){
      stateMachine.transitionTo(gateIsClosing);
    }else{
      stateMachine.transitionTo(gateIsOpening);
    }
  }
}

void transitionOutOfMove(){
  analogWrite(movingPin,LOW);
  if(stateMachine.isInState(gateIsOpening)){
    stateMachine.transitionTo(gateIsOpen);
  }else{
    stateMachine.transitionTo(gateIsClosed);
  }
}

void currentOverloadUpdate(){
  
}

void setup() {
  //Initialize gate to Open state
  Serial.begin(9600);
  Serial.println("OpenMyGate v 0.1");

  //Set up pins
  pinMode(movingPin, OUTPUT);
  pinMode(stoppedPin, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(CURRENT_SENSOR_PIN, INPUT);

  //Initialize current sensing array
  for (int i = 0; i < SMOOTHING_SIZE; i++) {
    current_readings[i] = 0;
  }
  
  //Get timings from EEPROM

  //Initialize current readings array
  
}

void loop() {
  stateMachine.update();
}
