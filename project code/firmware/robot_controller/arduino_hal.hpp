#pragma once
#include <Arduino.h>
#include <pwm.h>
#include "pins.hpp"
#include "src/controller.hpp"
#if !defined(ARDUINO_UNOR4_WIFI)
#error "This firmware targets Arduino Uno R4 WiFi only."
#endif
namespace robot {
class ArduinoHal:public Hal {
public:
  void beginPins() {
    digitalWrite(PIN_PAN,LOW);pinMode(PIN_PAN,OUTPUT);
    digitalWrite(PIN_TILT,LOW);pinMode(PIN_TILT,OUTPUT);
  }
  uint32_t nowMs() const override{return millis();}
  bool usbConnected() const override{return bool(Serial);}
  void safeOff() override {
    if(panOn_) {pan_.end();panOn_=false;}
    if(tiltOn_) {tilt_.end();tiltOn_=false;}
    pinMode(PIN_PAN,OUTPUT);digitalWrite(PIN_PAN,LOW);
    pinMode(PIN_TILT,OUTPUT);digitalWrite(PIN_TILT,LOW);
    // Servo supply remains powered until the physical switch is opened.
  }
  bool setServos(uint16_t p,uint16_t t) override {
    if(!panOn_) {panOn_=pan_.begin(uint32_t(20000),uint32_t(0));if(!panOn_)return false;}
    if(!tiltOn_) {tiltOn_=tilt_.begin(uint32_t(20000),uint32_t(0));if(!tiltOn_)return false;}
    return pan_.pulseWidth_us(p) && tilt_.pulseWidth_us(t);
  }
private:
  PwmOut pan_{PIN_PAN},tilt_{PIN_TILT};
  bool panOn_=false,tiltOn_=false;
};
}
