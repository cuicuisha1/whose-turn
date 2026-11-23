#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace robot {
constexpr uint32_t LINK_TIMEOUT_MS=200, MAX_LOOP_MS=60;
enum class State:uint8_t { Fault, Disarmed, Armed };
enum class Mode:uint8_t { Bench, Head };
enum class Type:uint8_t { Hello, Status, ClearFault, Prepare, Heartbeat, Servo, Stop, WatchdogTest };
// Preserve existing bit values for remaining faults in historical telemetry.
enum Fault:uint32_t {
  BootReset=1u<<0, LinkTimeout=1u<<2, InvalidCommand=1u<<5,
  ServoFailure=1u<<8, WatchdogUnavailable=1u<<9, UsbLost=1u<<10, LoopOverrun=1u<<11
};
enum class Error:uint8_t { None, BadFrame, NoSession, Replay, WrongState,
  InputsUnsafe, HardwareUnverified, ServoUnverified, Range, WrongEpoch,
  HardwareFailure, WatchdogUnavailable, SessionWhileActive };
struct Config {
  bool powerWiringVerified=false, servoLimitsVerified=false;
  uint16_t panMinUs=1400,panMaxUs=1600,tiltMinUs=1400,tiltMaxUs=1600;
};
struct Command {
  Type type=Type::Status;
  Mode mode=Mode::Bench;
  uint32_t sequence=0,session=0,armEpoch=0;
  uint16_t panUs=1500,tiltUs=1500;
  bool confirmOutputsOff=false;
};
struct Reply { bool ok; Error error; uint32_t sequence; };
class Hal {
public:
  virtual ~Hal() {}
  virtual uint32_t nowMs() const=0;
  virtual bool usbConnected() const=0;
  // Disable both PWM outputs; the servo supply remains powered.
  virtual void safeOff()=0;
  virtual bool setServos(uint16_t panUs,uint16_t tiltUs)=0;
};
inline const char *stateName(State s) {
  switch(s) {
    case State::Fault:return "FAULT";
    case State::Disarmed:return "DISARMED";
    case State::Armed:return "ARMED";
  } return "INVALID";
}
inline const char *modeName(Mode m) {return m==Mode::Head?"head":"bench";}
inline const char *errorName(Error e) {
  switch(e) {
    case Error::None:return "ok"; case Error::BadFrame:return "invalid_frame";
    case Error::NoSession:return "wrong_or_missing_session"; case Error::Replay:return "stale_sequence";
    case Error::WrongState:return "wrong_state"; case Error::InputsUnsafe:return "inputs_unsafe";
    case Error::HardwareUnverified:return "hardware_not_verified";
    case Error::ServoUnverified:return "servo_limits_not_verified";
    case Error::Range:return "out_of_range"; case Error::WrongEpoch:return "wrong_arm_epoch";
    case Error::HardwareFailure:return "hardware_io_failed";
    case Error::WatchdogUnavailable:return "watchdog_unavailable";
    case Error::SessionWhileActive:return "hello_while_active";
  } return "unknown";
}
inline bool validServoLimits(const Config &c) {
  return c.servoLimitsVerified && c.panMinUs>=500 && c.panMaxUs<=2500 &&
         c.tiltMinUs>=500 && c.tiltMaxUs<=2500 && c.panMinUs<=c.panMaxUs && c.tiltMinUs<=c.tiltMaxUs;
}
class Controller {
public:
  Controller(const Config &cfg,Hal &hal):cfg_(cfg),hal_(hal){}
  void begin(bool watchdogReady) {
    hal_.safeOff();servoActive_=false;watchdogReady_=watchdogReady;
    state_=State::Fault;faults_=BootReset|(watchdogReady?0u:uint32_t(WatchdogUnavailable));
  }
  bool active() const {return state_==State::Armed;}
  void trip(Fault reason) {
    hal_.safeOff();servoActive_=false;faults_|=uint32_t(reason);state_=State::Fault;
  }
  Reply malformed(uint32_t sequence=0) {
    if(active())trip(InvalidCommand);
    return {false,Error::BadFrame,sequence};
  }
  void checkSafety() {
    if(!active())return;
    if(!hal_.usbConnected()) {trip(UsbLost);return;}
    if(hal_.nowMs()-lastLive_>LINK_TIMEOUT_MS)trip(LinkTimeout);
  }
  void tick() {checkSafety();}
  Reply command(const Command &c) {
    checkSafety();
    if(c.type==Type::Status)return {true,Error::None,c.sequence};
    if(c.type==Type::Stop) {
      hal_.safeOff();servoActive_=false;
      if(state_!=State::Fault)state_=State::Disarmed;
      return {true,Error::None,c.sequence};
    }
    if(c.type==Type::Hello) {
      if(active())return reject(c,Error::SessionWhileActive);
      if(!c.session || !c.sequence)return reject(c,Error::NoSession);
      session_=c.session;lastSequence_=c.sequence;
      return {true,Error::None,c.sequence};
    }
    if(!session_ || c.session!=session_)return reject(c,Error::NoSession);
    if(!c.sequence || c.sequence<=lastSequence_)return reject(c,Error::Replay);
    lastSequence_=c.sequence; // Rejected packets do not renew the link.
    if(c.type==Type::ClearFault) {
      if(active())return reject(c,Error::WrongState);
      if(!watchdogReady_)return reject(c,Error::WatchdogUnavailable);
      faults_=0;state_=State::Disarmed;
      return {true,Error::None,c.sequence};
    }
    if(c.type==Type::WatchdogTest) {
      if(active() || !c.confirmOutputsOff)return reject(c,Error::InputsUnsafe);
      if(!watchdogReady_)return reject(c,Error::WatchdogUnavailable);
      hal_.safeOff();watchdogTestRequested_=true;
      return {true,Error::None,c.sequence};
    }
    if(c.type==Type::Prepare) {
      if(state_!=State::Disarmed || faults_ || watchdogTestRequested_)return reject(c,Error::WrongState);
      if(!watchdogReady_)return reject(c,Error::WatchdogUnavailable);
      if(!hal_.usbConnected())return reject(c,Error::InputsUnsafe);
      if(!cfg_.powerWiringVerified)return reject(c,Error::HardwareUnverified);
      if(c.mode==Mode::Head && !validServoLimits(cfg_))return reject(c,Error::ServoUnverified);
      hal_.safeOff();servoActive_=false;mode_=c.mode;
      state_=State::Armed;lastLive_=hal_.nowMs();
      if(!++armEpoch_)++armEpoch_;
      // Prepare never starts PWM or centres the head.
      return {true,Error::None,c.sequence};
    }
    if(c.type==Type::Heartbeat) {
      if(!active())return reject(c,Error::WrongState);
      lastLive_=hal_.nowMs();return {true,Error::None,c.sequence};
    }
    if(!active())return reject(c,Error::WrongState);
    if(c.armEpoch!=armEpoch_)return reject(c,Error::WrongEpoch);
    if(c.type==Type::Servo) {
      if(mode_!=Mode::Head || !validServoLimits(cfg_))return reject(c,Error::ServoUnverified);
      if(c.panUs<cfg_.panMinUs || c.panUs>cfg_.panMaxUs ||
         c.tiltUs<cfg_.tiltMinUs || c.tiltUs>cfg_.tiltMaxUs)return reject(c,Error::Range);
      if(!hal_.setServos(c.panUs,c.tiltUs)) {trip(ServoFailure);return {false,Error::HardwareFailure,c.sequence};}
      checkSafety();if(!active())return {false,Error::HardwareFailure,c.sequence};
      panUs_=c.panUs;tiltUs_=c.tiltUs;servoActive_=true;lastLive_=hal_.nowMs();
      return {true,Error::None,c.sequence};
    }
    return reject(c,Error::BadFrame);
  }
  State state() const{return state_;}
  Mode mode() const{return mode_;}
  uint32_t faults() const{return faults_;}
  uint32_t session() const{return session_;}
  uint32_t armEpoch() const{return armEpoch_;}
  uint32_t lastSequence() const{return lastSequence_;}
  const Config &config() const{return cfg_;}
  bool servoActive() const{return servoActive_;}
  uint16_t panUs() const{return panUs_;}
  uint16_t tiltUs() const{return tiltUs_;}
  bool watchdogTestRequested() const{return watchdogTestRequested_;}
  bool watchdogReady() const{return watchdogReady_;}
  uint32_t linkAge() const{return hal_.nowMs()-lastLive_;}
private:
  Config cfg_;Hal &hal_;
  State state_=State::Fault;Mode mode_=Mode::Bench;uint32_t faults_=BootReset;
  uint32_t session_=0,lastSequence_=0,armEpoch_=0,lastLive_=0;
  bool watchdogReady_=false,servoActive_=false,watchdogTestRequested_=false;
  uint16_t panUs_=1500,tiltUs_=1500;
  Reply reject(const Command &c,Error error) {
    if(active())trip(InvalidCommand);
    return {false,error,c.sequence};
  }
};
}
