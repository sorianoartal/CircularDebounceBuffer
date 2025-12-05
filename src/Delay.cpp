#include "Delay.h"

#include "Arduino.h"
#include "HardwareSerial.h"


/**Constructor: we pass the  target time delay*/
Delay::Delay(unsigned long delayTime) : _delayTime(delayTime), _previousTime(0){}

/** set the target time delay and start counting */
void Delay::init(){
  this->_delayTime = _delayTime;
  this->_previousTime = micros();
}

void Delay::init(unsigned long delayTime){
  this->_delayTime = delayTime;
  _previousTime = micros();
}


/** Calculate if the delay time has elapsed*/
bool Delay::isDelayTimeElapsed(){ 
  unsigned long now = micros();
  if(now - _previousTime >= _delayTime) {
    restartTimer() ; 
    return true;
  } else {
    return false;
  }
 
}

/** when the time delay has elapse we update the time counter*/
void Delay::restartTimer(){
  this->_previousTime = micros();
}

/** Set new Delay Value for the Class*/
void Delay::updateDelayTime(unsigned long newDelayTime){
  this->_delayTime = newDelayTime;
}