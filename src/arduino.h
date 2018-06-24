#ifndef ARDUINO_H
#define ARDUINO_H

#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <string.h>
#include <math.h>
#include "darknet.h"

#include "arduino-serial-lib.h"

#define BOTTOM    0
#define TOP       180
#define TOLERANCE 10

//#define L_LIMIT (BOTTOM + TOLERANCE)
#define L_LIMIT (14)
#define U_LIMIT (TOP - TOLERANCE)

#define MIDDLE  ((L_LIMIT + U_LIMIT)>>1)

#define PAN_START       MIDDLE
#define TILT_START      140


//#define BAUDRATE        1000000
#define BAUDRATE        115200
#define DEVPORT         "/dev/ttyUSB0"

#define PAN_COMMAND         48
#define TILT_COMMAND        49
#define WAKE_UP_COMMAND     50
#define SHUT_DOWN_COMMAND   51


#define CENTER_WINDOW_W   75
#define CENTER_WINDOW_H   50

#define READBACK_TIMER  16

#define PAN_STEP         1
#define TILT_STEP        1

void ard_serial_init();
int track_detections(image im, detection *dets, int num, float thresh, int classIdx);
void ard_serial_end();

#endif

