#include <Servo.h>
#include "servo_control.h"

Servo pan_servo;
Servo tilt_servo;

unsigned char pan_pos = 0;
unsigned char tilt_pos = 0;

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
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  attach();
  reset_servo();
}

void lock() {
  reset_servo();
  digitalWrite(LED_BUILTIN, HIGH);
  while(1){
  delay(100);
  }

}

void loop_reset(){
  while(1){
    reset_servo();
  }
}

int state = HIGH;

void loop() {

  if (Serial.available() > 0) {
    unsigned char count;
    count = Serial.read();
    for(int i = 0; i < count; i++) {
      while(Serial.available() <=0);
      unsigned char command = Serial.read();
      if(command==PAN_COMMAND) {
        while(Serial.available() <=0);
        pan_pos = Serial.read();
        if(pan_pos == 10)
          lock();
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

