#define RIGHT_HAND 0
#define LEFT_HAND 1

#define HAND LEFT_HAND
#if HAND==LEFT_HAND
#define DELAY_LOOP 200
#else
#define DELAY_LOOP 900
#endif