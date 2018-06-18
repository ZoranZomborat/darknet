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

#define DEMO 1

#ifndef ARDUINO_CTRL
#define ARDUINO_CTRL
#endif

#define WINDOW_NAME "Tracking_app"

static char **demo_names;
static image **demo_alphabet;
static int demo_classes;

static network *net;
static image buff [3];
static image buff_letter[3];
static int buff_index = 0;
static CvCapture * cap;
static IplImage  * ipl;
static float fps = 0;
static float demo_thresh = .5;
static float demo_hier = .5;
static int running = 0;

static int demo_frame = 3;
static int demo_index = 0;
static float **predictions;
static float *avg;
static int demo_done = 0;
static int demo_total = 0;
double demo_time;

static char ex=0;
static int serial_fd;

#define SKIP_FRAMES 12
static int frame_count=0;

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

void network_remember(network *net)
{
    int i;
    int count = 0;
    for(i = 0; i < net->n; ++i){
        layer l = net->layers[i];
        if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
            memcpy(predictions[demo_index] + count, net->layers[i].output, sizeof(float) * l.outputs);
            count += l.outputs;
        }
    }
}

detection *predictions_avg(network *net, int *nboxes)
{
    int i, j;
    int count = 0;
    fill_cpu(demo_total, 0, avg, 1);
    for(j = 0; j < demo_frame; ++j){
        axpy_cpu(demo_total, 1./demo_frame, predictions[j], 1, avg, 1);
    }
    for(i = 0; i < net->n; ++i){
        layer l = net->layers[i];
        if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
            memcpy(l.output, avg + count, sizeof(float) * l.outputs);
            count += l.outputs;
        }
    }
    detection *dets = get_network_boxes(net, buff[0].w, buff[0].h, demo_thresh, demo_hier, 0, 1, nboxes);
    return dets;
}

void *thread_detect(void *ptr)
{
    running = 1;
    float nms = .4;

    layer l = net->layers[net->n-1];
    float *X = buff_letter[(buff_index+2)%3].data;

    network_predict(net, X);

    network_remember(net);
    detection *dets = 0;
    int nboxes = 0;
    dets = predictions_avg(net, &nboxes);

    if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

    printf("\033[2J");
    printf("\033[1;1H");
    printf("\nFPS:%.1f\n",fps);
    printf("frame_count %d\n",frame_count);
    printf("Objects:\n\n");

    image display = buff[(buff_index+2) % 3];
    draw_detections(display, dets, nboxes, demo_thresh, demo_names, demo_alphabet, demo_classes);
#ifdef ARDUINO_CTRL
    if(frame_count >= SKIP_FRAMES)
        track_detections(display, dets, nboxes, demo_thresh);
    frame_count++;
#endif
    free_detections(dets, nboxes);

    demo_index = (demo_index + 1)%demo_frame;
    running = 0;
    return 0;
}

void *thread_fetch(void *ptr)
{
    int status = fill_image_from_stream(cap, buff[buff_index]);
    letterbox_image_into(buff[buff_index], net->w, net->h, buff_letter[buff_index]);
    if(status == 0) demo_done = 1;
    return 0;
}

void *thread_dispaly(void *ptr)
{
    show_image_cv(buff[(buff_index + 1)%3], WINDOW_NAME, ipl);
    int c = cvWaitKey(1);
    if (c != -1) c = c%256;
    if (c == 27) { //ESC key
        demo_done = 1;
        return 0;
    } else if (c == 'r') {
        demo_thresh += .02;
    } else if (c == 'e') {
        demo_thresh -= .02;
        if(demo_thresh <= .02) demo_thresh = .02;
    } else if (c == 'w') {
        demo_hier += .02;
    } else if (c == 'q') {
        demo_hier -= .02;
        if(demo_hier <= .0) demo_hier = .0;
    } else if (c == 's') {
        //Save current detect frame
        //Get time stamp and concatenate for file name
        save_image(buff[(buff_index + 1)%3], "dummy");
    } else if (c == 't') {
        //Enable disable tracking
    } else if (c == 'c') {
        //Draw all classes or strictly humans
    } else if (c == 'v') {
        //Verbose mode
    } else if (c == 'f') {
        //Enter exit fullscreen mode
    }

    return 0;
}

void track(char *cfgfile, char *weightfile, float thresh, int cam_index, char **names, int classes, float hier, int fullscreen)
{
    image **alphabet = load_alphabet();
    demo_names = names;
    demo_alphabet = alphabet;
    demo_classes = classes;
    demo_thresh = thresh;
    demo_hier = hier;
    net = load_network(cfgfile, weightfile, 0);
    set_batch_network(net, 1);
    pthread_t detect_thread;
    pthread_t fetch_thread;

#ifdef ARDUINO_CTRL
    track_init();
#endif

    srand(2222222);

    int i;
    demo_total = network_size(net);
    predictions = calloc(demo_frame, sizeof(float*));
    for (i = 0; i < demo_frame; ++i){
        predictions[i] = calloc(demo_total, sizeof(float));
    }
    avg = calloc(demo_total, sizeof(float));

    cap = cvCaptureFromCAM(cam_index);

    if(!cap) error("Couldn't connect to webcam.\n");

    buff[0] = get_image_from_stream(cap);
    buff[1] = copy_image(buff[0]);
    buff[2] = copy_image(buff[0]);
    buff_letter[0] = letterbox_image(buff[0], net->w, net->h);
    buff_letter[1] = letterbox_image(buff[0], net->w, net->h);
    buff_letter[2] = letterbox_image(buff[0], net->w, net->h);
    ipl = cvCreateImage(cvSize(buff[0].w,buff[0].h), IPL_DEPTH_8U, buff[0].c);

    int count = 0;

    cvNamedWindow(WINDOW_NAME, CV_WINDOW_NORMAL);

    if(fullscreen){
        cvSetWindowProperty(WINDOW_NAME, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    } else {
        cvMoveWindow(WINDOW_NAME, 0, 0);
        cvResizeWindow(WINDOW_NAME, 1352, 1013);
    }

    demo_time = what_time_is_it_now();

    while(!demo_done){
        buff_index = (buff_index + 1) %3;
        if(pthread_create(&fetch_thread, 0, thread_fetch, 0)) error("Thread creation failed");
        if(pthread_create(&detect_thread, 0, thread_detect, 0)) error("Thread creation failed");

        fps = 1./(what_time_is_it_now() - demo_time);
        demo_time = what_time_is_it_now();
        thread_dispaly(0);

        pthread_join(fetch_thread, 0);
        pthread_join(detect_thread, 0);
        ++count;
    }

#ifdef ARDUINO_CTRL
    track_end();
#endif

}

int main(int argc, char **argv)
{
    if(argc < 4){
        fprintf(stderr, "usage: %s dataCfg netCfg weights\n", argv[0]);
        return 0;
    }
    gpu_index = find_int_arg(argc, argv, "-i", 0);
    if(find_arg(argc, argv, "-nogpu")) {
        gpu_index = -1;
    }

#ifndef GPU
    gpu_index = -1;
#else
    if(gpu_index >= 0){
        cuda_set_device(gpu_index);
    }
#endif

    float thresh = find_float_arg(argc, argv, "-thresh", .5);
    float hier_thresh = find_float_arg(argc, argv, "-hier", .5);
    int cam_index = find_int_arg(argc, argv, "-c", 0);
    int fullscreen = find_arg(argc, argv, "-fullscreen");

    char *datacfg = argv[1];
    char *cfg = argv[2];
    char *weights = argv[3];

    list *options = read_data_cfg(datacfg);
    int classes = option_find_int(options, "classes", 20);
    char *name_list = option_find_str(options, "names", "data/names.list");
    char **names = get_labels(name_list);
    track(cfg, weights, thresh, cam_index, names, classes, hier_thresh, fullscreen);

    return 0;
}


