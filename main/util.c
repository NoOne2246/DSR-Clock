#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"  // ESP logging (if using ESP-IDF)
#include "util.h"

void normalize_utf8(char *data) {
    if (!data) return;
    size_t len = strlen(data);
    
    for (size_t i = 0; i < len; i++) {
        if ((unsigned char)data[i] > 127) {
            // Handle non-ASCII characters if needed
        }
    }
}

int is_hex_digit(char c) {
    return ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'));
}
void trim_whitespace(char *str) {
    char *end;
    
    // Remove leading spaces
    while (*str == ' ') str++;
    
    // Find end and remove trailing spaces
    end = str + strlen(str) - 1;
    while (end > str && (*end == ' ' || *end == '\r' || *end == '\n')) {
        end--;
    }

    *(end + 1) = '\0';  // Null-terminate correctly
}


void url_decode(char *str) {
    char *src = str, *dst = str;
    while (*src) {
        if (*src == '%' && is_hex_digit(src[1]) && is_hex_digit(src[2])) {
            char hex[3] = {src[1], src[2], '\0'};
            *dst = (char)strtol(hex, NULL, 16);
            src += 3;
        } else if (*src == '+') {
            *dst = ' ';
            src++;
        } else {
            *dst = *src++;
        }
        dst++;
    }
    *dst = '\0';
}


void normalize_line_endings(char *data) {
    char *pos;
    while ((pos = strstr(data, "\r\n")) != NULL) {
        memmove(pos, pos + 1, strlen(pos));  // Remove '\r'
    }
}

char *replaceSubstring(const char *str, const char *oldWord, const char *newWord) {
    if (!str || !oldWord || !newWord) return NULL; // Ensure valid input

    size_t strLen = strlen(str);
    size_t oldWordLen = strlen(oldWord);
    size_t newWordLen = strlen(newWord);

    // Count occurrences of oldWord in str
    size_t count = 0;
    const char *temp = str;
    while ((temp = strstr(temp, oldWord)) != NULL) {
        count++;
        temp += oldWordLen; // Move past the last occurrence
    }

    // Allocate enough space for the new string
    size_t newSize = strLen + (newWordLen - oldWordLen) * count + 1; // +1 for '\0'
    char *result = malloc(newSize);
    if (!result) return NULL; // Handle memory allocation failure

    char *pos, *dest = result;
    temp = str;

    while ((pos = strstr(temp, oldWord)) != NULL) {
        size_t lenBefore = pos - temp;
        strncpy(dest, temp, lenBefore);
        dest += lenBefore;

        strcpy(dest, newWord);
        dest += newWordLen;

        temp = pos + oldWordLen;
    }

    strcpy(dest, temp); // Copy remaining part of the original string

    return result;
}


char *append_to_buffer(char **html, size_t *buffer_size, size_t *position, const char *fmt, ...) {
    if (!html || !buffer_size || !position || !fmt) {
        return NULL; // Ensure parameters are valid
    }

    va_list args;
    va_start(args, fmt);

    // Calculate the required space
    size_t required = vsnprintf(NULL, 0, fmt, args);
    va_end(args);

    if (*position + required >= *buffer_size) {
        *buffer_size = (required > *buffer_size) ? *buffer_size + required : *buffer_size * 2;

        char *new_html = realloc(*html, *buffer_size);
        if (!new_html) {
            ESP_LOGE("APPEND", "Memory allocation failed.");
            free(*html);
            return NULL;
        }
        *html = new_html;
    }

    // Append formatted text to buffer
    va_start(args, fmt);
    *position += vsnprintf(*html + *position, *buffer_size - *position, fmt, args);
    va_end(args);

    return *html;
}
