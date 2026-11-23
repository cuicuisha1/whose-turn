#pragma once
#include "controller.hpp"
// Simulation values only; never copied into productionConfig().
inline robot::Config verifiedFakeConfig() {
  robot::Config c;c.powerWiringVerified=c.servoLimitsVerified=true;return c;
}
struct FakeHal:robot::Hal {
  uint32_t now=0,servoDelay=0;bool usb=true,servo=false,servoOk=true;
  uint16_t pan=0,tilt=0;
  uint32_t nowMs() const override{return now;}
  bool usbConnected() const override{return usb;}
  void safeOff() override{servo=false;}
  bool setServos(uint16_t p,uint16_t t) override {
    now+=servoDelay;pan=p;tilt=t;servo=servoOk;return servoOk;
  }
  void advance(uint32_t ms) {now+=ms;}
};
