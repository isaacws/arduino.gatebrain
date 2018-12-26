# gatebrain
Gate Brain is an open source gate controller that can attach to any actuator and provide remote connectivity and control.
Gate Brain utilizes a state machine to transition between states and control the gate actuator.
* Bump detection with current monitoring
* Soft start and stop
* Time based swing times with close/open detection (detects current spikes in soft stop and assumes it hit the limit of the gate swing)
* Manual open/close button support

Gate Brain runs on the ESP8266 and subscribes to MQTT messages for control.
The MQTT commands available are
* gate/Open - Opens the gate and keeps it open
* gate/Close - Closes the gate if it is in open state.
* gate/Cycle - Cycles the gate. Payload sent as a string is converted to float and is the seconds that the gate should stay open.
* gate/SetMoveTime - Sets move time for actuator and saves to EEPROM.
See https://hackaday.io/project/12691-gate-brain for more info
