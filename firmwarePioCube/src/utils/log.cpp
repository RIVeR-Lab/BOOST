#include "log.h"

bool log_printf_level(const char *file, uint32_t line, uint32_t level,
                      bool addNL, const char *Format, ...) {
  bool retc = true;
  if(level <= LOG_LEVEL){
    va_list arglist;
    va_start(arglist, Format);
    size_t bufSize = 512;
    char buffer[bufSize]{};
    size_t bufferI = 0;
    const char *str = "%s:%u: ";
    bufferI += snprintf(buffer, bufSize - bufferI, str, file, line);
    vsnprintf(&(buffer[bufferI]), bufSize - bufferI, Format, arglist);
#if LOGGING
    retc = Console.println(buffer);
    Console.flush();
#endif
    va_end(arglist);
  }
  return (retc);
}

bool log_printf_error(const char *file, uint32_t line, const char *Format,
                      ...) {
  bool retc = true;
  va_list arglist;
  va_start(arglist, Format);
  size_t bufSize = 512;
  char buffer[bufSize]{};
  size_t bufferI = 0;
  const char *str = "%s:%u: ";
  bufferI += snprintf(buffer, bufSize - bufferI, str, file, line);
  vsnprintf(&(buffer[bufferI]), bufSize - bufferI, Format, arglist);
#if LOGGING
  retc = Console.println(buffer);
  Console.flush();
#endif
  va_end(arglist);
  return (retc);
}
