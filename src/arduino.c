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

static box prev_det;
static int clearhist = 1;

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
}

void serial_tryClose() {

    int status;
    uint8_t echo;

    do {
        status = serialport_read_until(fd, &echo, NULL, 1, 1000);
    } while (status == 0);

    pan_pos = PAN_START;
    tilt_pos = TILT_START;

    serial_tryWrite(1);
    serial_tryWrite(SHUT_DOWN_COMMAND);

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

void track_detections_old(image im, detection *dets, int num, float thresh, int classIdx)
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

    if (det_h_center > (img_h_center + CENTER_WINDOW_W)) {
        tilt_pos += TILT_STEP;
        limit_val(&tilt_pos);
    }
    if (det_h_center < (img_h_center - CENTER_WINDOW_W)) {
        tilt_pos -= TILT_STEP;
        limit_val(&tilt_pos);
    }

    if (det_w_center > (img_w_center + CENTER_WINDOW_W)) {
        pan_pos -= PAN_STEP;
        limit_val(&pan_pos);
    }
    if (det_w_center < (img_w_center - CENTER_WINDOW_W)) {
        pan_pos += PAN_STEP;
        limit_val(&pan_pos);
    }

    //printf("tilt_pos %d pan_pos %d\n", tilt_pos, pan_pos);
    update_pan_tilt();

}

#define SKIP_FRAMES     80

#define FRAME_WINDOW    32
static int gframe_cnt = 0;
static int loop_frame_cnt = 0;

#define MAX_STEP        16

static int step_count = 0;

void track_detections_get_metrics(image im, detection *dets, int num, float thresh, int classIdx)
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

    if (gframe_cnt > SKIP_FRAMES) {

        if(loop_frame_cnt == 0) {
            pan_pos += step_count;
            update_pan_tilt();
            printf("++ %d\n", step_count);
        } else if (loop_frame_cnt == FRAME_WINDOW) {
            pan_pos -= step_count;
            printf("-- %d\n", step_count);
        } else if(loop_frame_cnt == FRAME_WINDOW*2) {
            loop_frame_cnt = -1;
            step_count ++;
            if(step_count > MAX_STEP){
                exit(0);
            }
        }
        update_pan_tilt();
        loop_frame_cnt ++;
        int w_err = abs(det_w_center - img_w_center);
        printf("gframe_cnt %d w_err %d\n", gframe_cnt, w_err);
    }
    gframe_cnt++;

}

int static cooldown = 0;

int track_detections(image im, detection *dets, int num, float thresh, int classIdx)
{
    int best_match_detect = -1;
    int best_match_area   = -1;
    float intersection_max_area = 0.0f;
    float box_max_area = 0.0f;

    int img_h_center = (int)((float)im.h/3.0)*2;
    int img_w_center = im.w >> 1;

    int det_h_center;
    int det_w_center;

    for(int i = 0; i < num; ++i){

        if (dets[i].prob[classIdx] <= thresh){
            continue;
        }

        box b = dets[i].bbox;

        float area;
        if (!clearhist) {
            area = box_intersection(b, prev_det);
            //printf("area %.2f idx %d\n", area, i);
            if (area > intersection_max_area) {
                intersection_max_area = area;
                best_match_detect = i;
            }
        }

        area  = b.w * b.h;

        if(area > box_max_area) {
            box_max_area = area;
            best_match_area = i;
        }

    }

    if (best_match_detect < 0) {
        if (best_match_area >= 0) {

            printf("%d NEW MAX\n\n\n", gframe_cnt);
            gframe_cnt ++;
            best_match_detect = best_match_area;
        } else {
            clearhist = 1;
            return -1;
        }
    }

    box b = dets[best_match_detect].bbox;
    prev_det = b;
    //printf("prev_det x %.2f y %.2f w %.2f h %.2f\n",b.x, b.y, b.w, b.h);
    clearhist = 0;

    int left  = (b.x-b.w/2.)*im.w;
    int right = (b.x+b.w/2.)*im.w;
    int top   = (b.y-b.h/2.)*im.h;
    int bot   = (b.y+b.h/2.)*im.h;

    if(left < 0) left = 0;
    if(right > im.w-1) right = im.w-1;
    if(top < 0) top = 0;
    if(bot > im.h-1) bot = im.h-1;

    det_h_center = top + ((bot - top) >> 1);
    det_w_center = left + ((right - left) >> 1);

//    printf("left %d right %d top %d bot %d\n", left, right, top, bot);
//    printf("img_w_center %d det_w_center %d img_h_center %d det_h_center %d\n", img_w_center, det_w_center,  img_h_center, det_h_center);

    if (det_h_center > (img_h_center + CENTER_WINDOW_H)) {
        tilt_pos += TILT_STEP;
        limit_val(&tilt_pos);
    }
    if (det_h_center < (img_h_center - CENTER_WINDOW_H)) {
        tilt_pos -= TILT_STEP;
        limit_val(&tilt_pos);
    }

    if (det_w_center > (img_w_center + CENTER_WINDOW_W)) {
        if ((left > (img_w_center + CENTER_WINDOW_W)) && !cooldown) {
            pan_pos -= PAN_STEP * 2;
            cooldown = 7;
        } else if (cooldown) {
            pan_pos -= PAN_STEP * 2;
        } else
            pan_pos -= PAN_STEP;
        limit_val(&pan_pos);
    }
    if (det_w_center < (img_w_center - CENTER_WINDOW_W)) {
        if ((right < (img_w_center - CENTER_WINDOW_W)) && !cooldown) {
            pan_pos += PAN_STEP * 2;
            cooldown = 7;
        } else if (cooldown) {
            pan_pos += PAN_STEP * 2;
        } else
            pan_pos += PAN_STEP;
        limit_val(&pan_pos);
    }

    if(cooldown>0)
        cooldown --;
    //printf("tilt_pos %d pan_pos %d\n", tilt_pos, pan_pos);
    update_pan_tilt();

    return best_match_detect;

}

void ard_serial_end() {
    serial_tryClose();
}


