#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "arduino.h"

#include "darknet.h"

#include <sys/time.h>

#ifndef ARDUINO_CTRL
#define ARDUINO_CTRL
#endif

#ifndef PERF
//#define PERF
#endif

#ifndef OPENCV
#define OPENCV
#endif

#define BUFF_SIZE   3

#define WINDOW_NAME "Tracking_app"
#define WINDOW_H 1016
#define WINDOW_W 1328

static char **gNames;
static image **gAlphabet;
static int drawClasses;
static int gClasses;
static network *net;
static image buff [BUFF_SIZE];
static image buff_letter[BUFF_SIZE];
static int buff_index = 0;
static CvCapture * cap;
static IplImage  * ipl;

static float gThresh = .5;
static float gHier = .5;
static int gClassIdx = 0;
static int gFullscreen = 0;
static int gTrack = 1;

static int detection_index = 0;
static float **predictions;
static float *avg;
static int app_done = 0;
static int output_size = 0;
double elapsed_time;

#define SKIP_FRAMES 50
static int frame_count=0;
static float fps = 0;
static double acc_fps = 0;
static double avg_fps = 0;

detection *get_network_boxes(network *net, int w, int h, float thresh, float hier, int *map, int relative, int *num);

int network_size(network *net)
{
    int i;
    int count = 0;
    for(i = 0; i < net->n; ++i){
        layer l = net->layers[i];
        if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
            count += l.outputs;
        }
    }
    return count;
}

//Save output of each detection layer into prediction buffers
void network_remember(network *net)
{
    int i;
    int count = 0;
    for(i = 0; i < net->n; ++i){
        layer l = net->layers[i];
        if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
            memcpy(predictions[detection_index] + count, net->layers[i].output, sizeof(float) * l.outputs);
            count += l.outputs;
        }
    }
}

detection *predictions_avg(network *net, int *nboxes)
{
    int i, j;
    int count = 0;

    network_remember(net);

    //Memset of avg
    fill_cpu(output_size, 0, avg, 1);
    //Do average over the prediction buffer entries
    for(j = 0; j < BUFF_SIZE; ++j){
        axpy_cpu(output_size, 1./BUFF_SIZE, predictions[j], 1, avg, 1);
    }
    //Copy back the detect output results into the network layers
    for(i = 0; i < net->n; ++i){
        layer l = net->layers[i];
        if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
            memcpy(l.output, avg + count, sizeof(float) * l.outputs);
            count += l.outputs;
        }
    }
    detection *dets = get_network_boxes(net, buff[0].w, buff[0].h, gThresh, gHier, 0, 1, nboxes);
    return dets;
}

void logResult(float fps,int  frame_count) {
//    printf("\033[2J");
//    printf("\033[1;1H");
//    printf("\nFPS:%.1f\n", fps);
//    printf("Frame_count %d\n", frame_count);
//    printf("Objects:\n\n");
}

void *thread_detect(void *ptr)
{
    float overlappingFactor = .4;
    layer l = net->layers[net->n-1];
    float *frame = buff_letter[(buff_index+2)%3].data;

    network_predict(net, frame);

    detection *dets = 0;
    int nboxes = 0;
    dets = predictions_avg(net, &nboxes);

    if (overlappingFactor > 0)
        process_detections(dets, &nboxes, l.classes, overlappingFactor);

    image display = buff[(buff_index+2) % 3];
    draw_detections(display, dets, nboxes, gThresh, gNames, gAlphabet, drawClasses);
    logResult(fps, frame_count);

#ifdef ARDUINO_CTRL
    if (gTrack)
        track_detections(display, dets, nboxes, gThresh, gClassIdx);
#endif

    free_detections(dets, nboxes);
    detection_index = (detection_index + 1)%BUFF_SIZE;

#ifdef PERF
    if(frame_count > SKIP_FRAMES) {
        acc_fps +=fps;
    }
    if(frame_count == 500) {
        app_done = 1;
        avg_fps = acc_fps / (frame_count - SKIP_FRAMES);
        printf("\navg_fps:%.2f\n", avg_fps);
    }
    frame_count++;
#endif

    return 0;
}

void *thread_fetch(void *ptr)
{
    int status = fill_image_from_stream(cap, buff[buff_index]);
    letterbox_image_into(buff[buff_index], net->w, net->h, buff_letter[buff_index]);
    if(status == 0) app_done = 1;
    return 0;
}

void *thread_display(void *ptr) {
    show_image_cv(buff[(buff_index + 1) % 3], WINDOW_NAME, ipl);
    int c = cvWaitKey(1);
    if (c != -1)
        c = c % 256;
    if (c == 27) { //ESC key
        app_done = 1;
        return 0;
    } else if (c == 'r') {
        gThresh += .02;
    } else if (c == 'e') {
        gThresh -= .02;
        if (gThresh <= .02)
            gThresh = .02;
    } else if (c == 't') {
        gTrack = 1 - gTrack;
    } else if (c == 'c') {
        //Draw all classes or strictly humans
        if (drawClasses == 1) {
            drawClasses = gClasses;
        } else {
            drawClasses = 1;
        }
    } else if (c == 'f') {
        if (!gFullscreen) {
            cvSetWindowProperty(WINDOW_NAME, CV_WND_PROP_FULLSCREEN,
                    CV_WINDOW_FULLSCREEN);
        } else {
            cvMoveWindow(WINDOW_NAME, 0, 0);
            cvResizeWindow(WINDOW_NAME, WINDOW_W, WINDOW_H);
        }
        gFullscreen = 1 - gFullscreen;
    }

    return 0;
}

void gpu_init() {

    gpu_index = 0;
    #ifndef GPU
        gpu_index = -1;
    #else
        cuda_set_device(gpu_index);
    #endif

}

void network_init(char *cfgfile, char *weightfile){

    net = load_network(cfgfile, weightfile, 0);
    set_batch_network(net, 1);

    int i;
    output_size = network_size(net);
    predictions = calloc(BUFF_SIZE, sizeof(float*));
    for (i = 0; i < BUFF_SIZE; ++i) {
        predictions[i] = calloc(output_size, sizeof(float));
    }
    avg = calloc(output_size, sizeof(float));

}

void cam_init(int cam_index, int w, int h) {

    //Init camera
    cap = cvCaptureFromCAM(cam_index);
    if(!cap) error("Couldn't connect to webcam.\n");

    if (w) {
        cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_WIDTH, w);
    }
    if (h) {
        cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_HEIGHT, h);
    }

    //Fill initial buffers
    buff[0] = get_image_from_stream(cap);
    buff[1] = copy_image(buff[0]);
    buff[2] = copy_image(buff[0]);
    buff_letter[0] = letterbox_image(buff[0], net->w, net->h);
    buff_letter[1] = letterbox_image(buff[0], net->w, net->h);
    buff_letter[2] = letterbox_image(buff[0], net->w, net->h);
    ipl = cvCreateImage(cvSize(buff[0].w,buff[0].h), IPL_DEPTH_8U, buff[0].c);

    //Create window
    cvNamedWindow(WINDOW_NAME, CV_WINDOW_NORMAL);
    if (gFullscreen) {
        cvSetWindowProperty(WINDOW_NAME, CV_WND_PROP_FULLSCREEN,
                CV_WINDOW_FULLSCREEN);
    } else {
        cvMoveWindow(WINDOW_NAME, 0, 0);
        cvResizeWindow(WINDOW_NAME, WINDOW_W, WINDOW_H);
    }

}

void track(float thresh, char **names, int classes)
{
    gNames = names;
    drawClasses = gClasses = classes;
    gThresh = thresh;
    gAlphabet = load_alphabet();
    pthread_t detect_thread;
    pthread_t fetch_thread;

#ifdef ARDUINO_CTRL
    ard_serial_init();
#endif

    srand(2222222);

    int count = 0;

    elapsed_time = what_time_is_it_now();

    while(!app_done){
        buff_index = (buff_index + 1) %3;
        if(pthread_create(&fetch_thread, 0, thread_fetch, 0)) error("Thread creation failed");
        if(pthread_create(&detect_thread, 0, thread_detect, 0)) error("Thread creation failed");

        fps = 1./(what_time_is_it_now() - elapsed_time);
        elapsed_time = what_time_is_it_now();
        thread_display(0);

        pthread_join(fetch_thread, 0);
        pthread_join(detect_thread, 0);
        ++count;
    }

#ifdef ARDUINO_CTRL
    ard_serial_end();
#endif

}

int main(int argc, char **argv)
{
    if(argc < 4){
        fprintf(stderr, "usage: %s dataCfg netCfg weights\n", argv[0]);
        return 0;
    }

    //Process command line arguments
    float thresh = find_float_arg(argc, argv, "-thresh", .5);
    int cam_index = find_int_arg(argc, argv, "-cam", 0);
    int width = find_int_arg(argc, argv, "-w", 0);
    int height = find_int_arg(argc, argv, "-h", 0);
    gFullscreen = find_arg(argc, argv, "-fullscreen");
    gClassIdx = find_int_arg(argc, argv, "-class", 0);

    char *datacfg = argv[1];
    char *cfg = argv[2];
    char *weights = argv[3];

    //Fetch class labels
    list *options = read_data_cfg(datacfg);
    int classes = option_find_int(options, "classes", 20);
    char *name_list = option_find_str(options, "names", "data/names.list");
    char **names = get_labels(name_list);

    gpu_init();

    network_init(cfg, weights);

    cam_init(cam_index, width, height);

    track(thresh, names, classes);

    return 0;
}


