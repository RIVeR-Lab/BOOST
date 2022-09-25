/**
 * uartBase.cpp
 *
 * Author: David Antaki
 * Date: 9/24/2022
 */

#include "uartBase.h"
uartBase::uartBase(const struct device &_uartDev) : uartDev(_uartDev) {}
uartBase::~uartBase() {}

// Just a wrapper for the time being
int uartBase::poll_read(unsigned char &p_char){
    return uart_poll_in(&uartDev, &p_char);
}