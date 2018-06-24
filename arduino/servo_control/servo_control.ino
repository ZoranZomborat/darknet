#include <Servo.h>
#include "servo_control.h"

Servo pan_servo;
Servo tilt_servo;

unsigned char pan_pos = 0;
unsigned char tilt_pos = 0;

long long int readback_val = 0;
unsigned char readback_counter = 0;

unsigned int synched = 0;

byte incomingByte;

void reset_servo(){

  pan_pos = PAN_START;
  pan_servo.write(pan_pos);

  tilt_pos = TILT_START;
  tilt_servo.write(tilt_pos);
  
}

void attach() {
  pan_servo.attach(PAN_PIN);
  tilt_servo.attach(TILT_PIN);
}

void setup() {
  //Serial.begin(1000000);
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  attach();
  reset_servo();
}

int state = HIGH;

void lock() {
  reset_servo();
  pan_servo.detach();
  tilt_servo.detach();
  while(1){
    delay(300);
    digitalWrite(LED_BUILTIN, state);
    state = 1 - state;  
  }

}

void loop_reset(){
  while(1){
    reset_servo();
  }
}



void loop() {

  if (Serial.available() > 0) {
    unsigned char count;
    count = Serial.read();
    for(int i = 0; i < count; i++) {
      while(Serial.available() <=0);
      unsigned char command = Serial.read();
      if(command == WAKE_UP_COMMAND) {
         Serial.write(command);
         state = HIGH;
         digitalWrite(LED_BUILTIN, state);
      } else if(command==PAN_COMMAND) {
        while(Serial.available() <=0);
        pan_pos = Serial.read();
        Serial.write(pan_pos);
        pan_servo.write(pan_pos);
      } else if (command==TILT_COMMAND) {
        while(Serial.available() <=0);
        tilt_pos = Serial.read();
        Serial.write(tilt_pos);
        tilt_servo.write(tilt_pos);
      } else {
        lock(); //unknown command
      }
    }
  }
}

