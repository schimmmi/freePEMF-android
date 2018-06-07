//
// Created by blos on 2017-10-14.
//

#include <USBAPI.h>
#include "FPTimer.h"

FPTimer fpTimer;

FPTimer::FPTimer() {
    resetTimer();
}

void FPTimer::resetTimer() {
    startIntervalA = millis();
}


bool FPTimer::checkPauseTimeout() {
    return checkTimeout(pauseTimeOut);
}


bool FPTimer::checkTimeout(unsigned long timeout) {
    return (millis() - startIntervalA) >= timeout;
}

bool FPTimer::checkButtonTimeout() {
    return isTicking(btnTimeOut);
}

unsigned long FPTimer::getTimerValue() {
    return startIntervalA;
}


bool FPTimer::isTicking(unsigned long time) {
    return  (millis() - startIntervalA <= time);
}

unsigned long FPTimer::getElapsedTime() {
  double seconds = (millis() - startIntervalA) /1000; 
  return  (unsigned long) seconds;
}

void FPTimer::addPause(unsigned long pauseTime) {
   startIntervalA += pauseTime;
}


