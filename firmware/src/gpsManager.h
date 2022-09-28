/**
 * gpsManager.h
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#ifndef SRC_GPSMANAGER_H
#define SRC_GPSMANAGER_H
#include "uartBase.h"
#include "ubxNeo6M.h"
#include <device.h>
#include <zephyr.h>
#include "3rd_party/TinyGPSPlus.h"
#include <stdio.h>

class gpsManager {
public:
  gpsManager(uartBase _uart);
  ~gpsManager();
  void create();
  void start();
  static const uint32_t kStackSize = 1024; // in bytes

private:
  static const uint32_t kThreadPriority = 2;
  static struct k_thread gps_manager_thread_data;
  static k_tid_t kThreadId;
  static const uint32_t loopTimeMs = 1;
  ubxNeo6M gps;

  void loopHook();
  static void entryPoint(void *, void *, void *);

  
static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      printk("*");
    printk(" ");
  }
  else
  {
    printk("%.*f", prec, val);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      printk(" ");
  }
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  printk("%s", sz);
}

static void printDateTime(TinyGPSDate d, TinyGPSTime t)
{
  if (!d.isValid())
  {
    printk("********** ");
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    printk("%s", sz);
  }
  
  if (!t.isValid())
  {
    printk("******** ");
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    printk("%s", sz);
  }
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    if(i<slen){
      printk("%c", str[i]);
    }else{
      printk(" ");
    }
}

};

#endif // SRC_GPSMANAGER_H
