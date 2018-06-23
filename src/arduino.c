#include "arduino.h"
#include "utils.h"
#include "blas.h"
#include <stdio.h>
#include <math.h>

static uint8_t pan_pos;
static uint8_t tilt_pos;

static uint8_t readback_pan = 0;
static uint8_t readback_tilt = 0;

static int fd;
static int readback_counter = 0;
static int readback_val = 0;

void serial_tryWrite(uint8_t val) {
    int status = serialport_writebyte(fd, val);
    if(status < 0) error("Couldn't send data over serial.\n");
}

void serial_tryRead(uint8_t* val){
    int status = serialport_read_until(fd, val, NULL, 1, 1000);
    if(status < 0) {
        printf("read status %d\n", status);
        error("Couldn't read data over serial.\n");
    }
    //printf("r %d \n", *val);
}

void serial_tryClose() {

    int status;
    uint8_t echo;
    //Read all data left on serial
    do {
        status = serialport_read_until(fd, &echo, NULL, 1, 1000);
    } while (status == 0);

    serialport_close(fd);
}

void update_pan() {
    uint8_t echo;
    serial_tryWrite(PAN_COMMAND);
    serial_tryWrite(pan_pos);

    if(readback_counter > 0) {
        serial_tryRead(&echo);
        if(echo != readback_pan){
            printf("Sent pan %d got %d\n",readback_pan ,echo);
            error("Echo mismatch");
        }
    }
}

void update_tilt() {
    uint8_t echo;

    //printf("\n\nstart command\n");

    serial_tryWrite(TILT_COMMAND);
    serial_tryWrite(tilt_pos);

    if(readback_counter > 0) {
        serial_tryRead(&echo);
        if(echo != readback_tilt){
            printf("Sent tilt %d got %d\n",readback_tilt ,echo);
            error("Echo mismatch");
        }
    }
}

void update_pan_tilt() {

    serial_tryWrite(2);
    update_pan();
    update_tilt();

    if(readback_counter == 0){
        readback_counter = 1;
    }
    readback_tilt = tilt_pos;
    readback_pan = pan_pos;

}

void wake_up() {
    uint8_t echo;
    uint32_t status;

    //Read all data left on serial
    do {
        status = serialport_read_until(fd, &echo, NULL, 1, 1000);
    } while (status == 0);
    do {
        serial_tryWrite(1);
        serial_tryWrite(WAKE_UP_COMMAND);
        status = serialport_read_until(fd, &echo, NULL, 1, 1000);
    } while(status);

    if(echo != WAKE_UP_COMMAND){
        printf("Sent command %d got %d\n",pan_pos ,echo);
        error("Echo mismatch");
    }
}

void ard_serial_init() {

    fd = serialport_init(DEVPORT, BAUDRATE);
    if(fd < 0) error("Couldn't open serial port.\n");

    pan_pos = PAN_START;
    tilt_pos = TILT_START;

    wake_up();

}

void limit_val(uint8_t *val){
    if ((*val) < L_LIMIT)
        *val = L_LIMIT;

    if ((*val) > U_LIMIT)
        *val = U_LIMIT;
}

void track_detections(image im, detection *dets, int num, float thresh, int classIdx)
{
    int best_detect = -1;
    float max_area = 0.0f;

    int img_h_center = im.h >> 1;
    int img_w_center = im.w >> 1;

    int det_h_center;
    int det_w_center;

    for(int i = 0; i < num; ++i){

        if (dets[i].prob[classIdx] <= thresh){
            continue;
        }

        box b = dets[i].bbox;
        float area  = b.w * b.h;

        if(area > max_area) {
            max_area = area;
            best_detect = i;
        }

    }

    if(best_detect < 0)
        return;

    box b = dets[best_detect].bbox;

    int left  = (b.x-b.w/2.)*im.w;
    int right = (b.x+b.w/2.)*im.w;
    int top   = (b.y-b.h/2.)*im.h;
    int bot   = (b.y+b.h/2.)*im.h;

    if(left < 0) left = 0;
    if(right > im.w-1) right = im.w-1;
    if(top < 0) top = 0;
    if(bot > im.h-1) bot = im.h-1;

    //printf("left %d right %d top %d bot %d\n", left, right, top, bot);

    det_h_center = top + ((bot - top) >> 1);
    det_w_center = left + ((right - left) >> 1);

    if (det_h_center > (img_h_center + CENTER_WINDOW)) {
        tilt_pos += TILT_STEP;
        limit_val(&tilt_pos);
    }
    if (det_h_center < (img_h_center - CENTER_WINDOW)) {
        tilt_pos -= TILT_STEP;
        limit_val(&tilt_pos);
    }

    if (det_w_center > (img_w_center + CENTER_WINDOW)) {
        pan_pos -= PAN_STEP;
        limit_val(&pan_pos);
    }
    if (det_w_center < (img_w_center - CENTER_WINDOW)) {
        pan_pos += PAN_STEP;
        limit_val(&pan_pos);
    }

    //printf("tilt_pos %d pan_pos %d\n", tilt_pos, pan_pos);
    update_pan_tilt();

}

void ard_serial_end() {
    serial_tryClose();
}


