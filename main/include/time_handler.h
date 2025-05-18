#ifndef TIME_HANDLER_H
#define TIME_HANDLER_H

#include <stdbool.h>


// Function prototypes
void set_time(void *pvParameter);
bool obtain_time_sntp(void);
bool obtain_time_rtc(void);
bool set_time_rtc(void);

void create_set_time_task();

void obtain_time_manual(const char *datetime_str);
void set_TZ(const char *tz_identifier);
const char *get_posix_from_id(const char *zone_identifier);

char *generateSelectList(void);

#endif // TIME_HANDLER_H