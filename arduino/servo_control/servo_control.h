#ifndef SRV_CTRL
#define SRV_CTRL

#define BOTTOM    0
#define TOP       180
#define TOLERANCE 10

//#define L_LIMIT (BOTTOM + TOLERANCE)
#define L_LIMIT (14)
#define U_LIMIT (TOP - TOLERANCE)

#define MIDDLE  ((L_LIMIT + U_LIMIT)>>1)

#define PAN_START       MIDDLE
#define TILT_START      140

#define PAN_PIN         9
#define TILT_PIN        10

#define DELAY_TIME      50

#define PAN_COMMAND       48
#define TILT_COMMAND      49
#define WAKE_UP_COMMAND   50
#define SHUT_DOWN_COMMAND 51


#endif
