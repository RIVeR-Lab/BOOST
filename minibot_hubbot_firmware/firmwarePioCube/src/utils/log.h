#ifndef SRC_LOG_H
#define SRC_LOG_H
#include "stdarg.h"
#include "stdint.h"
#include "Arduino.h"
#include "main.h"
#include "configs/config.h"

enum {
  LOGLEVEL_NONE,          // no logging
  LOGLEVEL_FREERTOS,         // lowest normal level
  LOGLEVEL_VERBOSE,
  LOGLEVEL_INFO,
  LOGLEVEL_EVENT,
  LOGLEVEL_DATA,
  LOGLEVEL_ERROR = 0xff,		// highest
  LOGLEVEL_DEBUG = 0x1000  // extra level for debug (programming flow) messages that normally get suppressed
};

bool log_printf_level(bool ros, const char *file, uint32_t line, uint32_t level,
                      bool addNL, const char *Format, ...);
bool log_printf_error(bool ros, const char *file, uint32_t line, const char *Format,
                      ...);

#define __FILENAME__ (strrchr("/" __FILE__, '/') + 1)                      

#define LOGRTOS(...) log_printf_level(false, __FILENAME__, __LINE__, LOGLEVEL_FREERTOS, false, __VA_ARGS__)
// prefixed with small 'i'; lowest normal priority mask to log (LOGLEVEL_VERBOSE)
#define LOGVERBOSE(...) log_printf_level(false, __FILENAME__, __LINE__, LOGLEVEL_VERBOSE, true, __VA_ARGS__)
// prefixed with 'I'; priority LOGLEVEL_INFO
#define LOGINFO(...) log_printf_level(false, __FILENAME__, __LINE__, LOGLEVEL_INFO, true, __VA_ARGS__)
// prefixed with 'D' priority LOGLEVEL_DATA to log; always goes out data port
#define LOGDATA(...) log_printf_level(false, __FILENAME__, __LINE__, LOGLEVEL_DATA, true, __VA_ARGS__)
// prefixed with 'C'; priority LOGLEVEL_ERROR; separated out to prevent recursion
#define LOGCOMMERROR(...) log_printf_level(false, __FILENAME__, __LINE__, LOGLEVEL_ERROR, true, __VA_ARGS__)
// prefixed with 'V'; priority LOGLEVEL_EVENT
#define LOGEVENT(...) log_printf_level(false, __FILENAME__, __LINE__, LOGLEVEL_EVENT, true, __VA_ARGS__)
// prefixed with 'E'; always logs
#define LOGERROR(...) log_printf_error(false, __FILENAME__, __LINE__, __VA_ARGS__)
#define LOGDEBUG(...) log_printf_level(false, __FILENAME__, __LINE__, LOGLEVEL_DEBUG, true, __VA_ARGS__)

#define ROSLOGRTOS(...) log_printf_level(true, __FILENAME__, __LINE__, LOGLEVEL_FREERTOS, false, __VA_ARGS__)
// prefixed with small 'i'; lowest normal priority mask to log (LOGLEVEL_VERBOSE)
#define ROSLOGVERBOSE(...) log_printf_level(true, __FILENAME__, __LINE__, LOGLEVEL_VERBOSE, true, __VA_ARGS__)
// prefixed with 'I'; priority LOGLEVEL_INFO
#define ROSLOGINFO(...) log_printf_level(true, __FILENAME__, __LINE__, LOGLEVEL_INFO, true, __VA_ARGS__)
// prefixed with 'D' priority LOGLEVEL_DATA to log; always goes out data port
#define ROSLOGDATA(...) log_printf_level(true, __FILENAME__, __LINE__, LOGLEVEL_DATA, true, __VA_ARGS__)
// prefixed with 'C'; priority LOGLEVEL_ERROR; separated out to prevent recursion
#define ROSLOGCOMMERROR(...) log_printf_level(true, __FILENAME__, __LINE__, LOGLEVEL_ERROR, true, __VA_ARGS__)
// prefixed with 'V'; priority LOGLEVEL_EVENT
#define ROSLOGEVENT(...) log_printf_level(true, __FILENAME__, __LINE__, LOGLEVEL_EVENT, true, __VA_ARGS__)
// prefixed with 'E'; always logs
#define ROSLOGERROR(...) log_printf_error(true, __FILENAME__, __LINE__, __VA_ARGS__)
#define ROSLOGDEBUG(...) log_printf_level(true, __FILENAME__, __LINE__, LOGLEVEL_DEBUG, true, __VA_ARGS__)



#endif  // SRC_LOG_H