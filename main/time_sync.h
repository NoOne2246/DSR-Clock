#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdbool.h>

// Function prototypes
void set_time(void);
bool obtain_time_sntp(void);
bool obtain_time_rtc(void);
bool set_time_rtc(void);

#endif // TIME_SYNC_H