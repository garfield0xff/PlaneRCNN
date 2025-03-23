#include <string>

using namespace std;

typedef int32_t result_t;

#define RESULT_OK 0
#define RESULT_TIMEOUT -1
#define RESULT_FAIL -2

#include <sys/time.h>
#include <unistd.h>

static inline void delay(uint32_t ms) {
    while(ms >= 1000) {
        usleep(1000 * 1000);
        ms -= 1000;
    }


    if(ms != 0) {
        usleep(ms * 1000);
    }
}

static inline uint32_t getms() {
    struct timespec t;
    t.tv_sec = t.tv_nsec = 0;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000L + t.tv_nsec / 1000000L;
  }