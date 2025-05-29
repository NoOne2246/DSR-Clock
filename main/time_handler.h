#ifndef TIME_HANDLER_H
#define TIME_HANDLER_H

#include <stdbool.h>


// Function prototypes

extern void create_set_time_task();

extern void obtain_time_manual(const char *datetime_str);
extern void set_TZ(const char *tz_identifier);
extern const char *get_posix_from_id(const char *zone_identifier);
extern char *generateSelectList(void);

#endif // TIME_HANDLER_H