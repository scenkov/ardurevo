
#include <AP_HAL.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include "UARTDriver.h"
#include "Util.h"

/*
  implement vsnprintf with support for %S meaning a progmem string
*/
static int libc_vsnprintf(char* str, size_t size, const char *fmt, va_list ap)
{
    int i, ret;
    char *fmt2 = (char *)fmt;
    if (strstr(fmt2, "%S") != NULL) {
        fmt2 = strdup(fmt);
        for (i=0; fmt2[i]; i++) {
            // cope with %S
            if (fmt2[i] == '%' && fmt2[i+1] == 'S') {
                fmt2[i+1] = 's';
            }
        }
    }
    ret = vsnprintf(str, size, fmt2, ap);
    if (fmt2 != fmt) {
        free(fmt2);
    }
    return ret;
}

using namespace VRBRAIN;

int VRBRAINUtil::snprintf(char* str, size_t size, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = libc_vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}

int VRBRAINUtil::snprintf_P(char* str, size_t size, const prog_char_t *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = libc_vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}

int VRBRAINUtil::vsnprintf(char* str, size_t size, const char *format, va_list ap)
{
    return libc_vsnprintf(str, size, format, ap);
}

int VRBRAINUtil::vsnprintf_P(char* str, size_t size, const prog_char_t *format,
            va_list ap)
{
    return libc_vsnprintf(str, size, format, ap);
}
