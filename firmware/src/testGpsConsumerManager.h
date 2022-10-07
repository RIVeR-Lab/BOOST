/**
 * testGpsConsumerManager.h
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#ifndef SRC_TEST_GPS_CONSUMER_MANAGER_H
#define SRC_TEST_GPS_CONSUMER_MANAGER_H
#include <device.h>
#include <zephyr.h>
#include "3rd_party/TinyGPSPlus.h"
#include <stdio.h>


class testGpsConsumerManager {
public:
  testGpsConsumerManager();
  ~testGpsConsumerManager();
  void create();
  bool initialize();
  void start();

  static const uint32_t kStackSize = 1024; // in bytes

private:
  static const uint32_t kThreadPriority = 2;
  static struct k_thread testGpsConsumerManagerData;
  static k_tid_t kThreadId;
  static const uint32_t loopTimeMs = 100;

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
    // char sz[32] = "*****************";
    // sprintf(sz, "%f", val);
    printk("%.5f", val);
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

static void printStr(bool valid, const char *str)
{
  if(valid){
    printk("%s", str);
  }else{
    printk("***");
  }
  // int slen = strlen(str);
  // for (int i=0; i<len; ++i)
  //   if(i<slen){
  //     printk("%c", str[i]);
  //   }else{
  //     printk(" ");
  //   }
}

};

#endif // SRC_TEST_GPS_CONSUMER_MANAGER_H
