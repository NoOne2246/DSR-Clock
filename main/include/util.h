#ifndef UTIL_H
#define UTIL_H

#include <stddef.h>

void trim_whitespace(char *str) ;
void url_decode(char *str);
void normalize_line_endings(char *data);

void normalize_utf8(char *data);
char *replaceSubstring(const char *str, const char *oldWord, const char *newWord);
char *append_to_buffer(char **html, size_t *buffer_size, size_t *position, const char *fmt, ...);

#endif // UTIL_H
