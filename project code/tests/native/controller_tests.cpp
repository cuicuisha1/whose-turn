#include <iostream>
#include <stdexcept>
#include <functional>
#include "fake_hal.hpp"
#include "protocol.hpp"
#include "../../firmware/robot_controller/config.hpp"
#include "../../firmware/robot_controller/pins.hpp"
using namespace robot;
#define CHECK(expr) do {if(!(expr))throw std::runtime_error(std::string(__FILE__)+":"+std::to_string(__LINE__)+" " #expr);}while(0)

struct Rig {
  FakeHal h;Controller c;uint32_t seq=0;
  explicit Rig(Config cfg=verifiedFakeConfig()):c(cfg,h){c.begin(true);}
  Reply send(Type type) {
    Command cmd;cmd.type=type;cmd.session=42;cmd.sequence=++seq;cmd.armEpoch=c.armEpoch();return c.command(cmd);
  }
  void handshake(){CHECK(send(Type::Hello).ok);CHECK(send(Type::ClearFault).ok);}
  Reply prepare(Mode mode=Mode::Head) {
    Command p;p.type=Type::Prepare;p.mode=mode;p.session=42;p.sequence=++seq;return c.command(p);
  }
  void arm(Mode mode=Mode::Head){handshake();CHECK(prepare(mode).ok);CHECK(c.state()==State::Armed);CHECK(!h.servo);}
  Reply servo(uint16_t pan=1500,uint16_t tilt=1550,uint32_t epoch=0) {
    Command s;s.type=Type::Servo;s.session=42;s.sequence=++seq;s.armEpoch=epoch?epoch:c.armEpoch();
    s.panUs=pan;s.tiltUs=tilt;return c.command(s);
  }
  void step(uint32_t ms,bool heartbeat=false) {
    for(uint32_t i=0;i<ms;i++){h.advance(1);c.tick();if(heartbeat && i%50==0)send(Type::Heartbeat);}
  }
};
void boot_and_guards() {
  Rig r(productionConfig());CHECK(r.c.faults()==BootReset);r.handshake();
  CHECK(r.prepare().error==Error::HardwareUnverified);CHECK(!r.h.servo);
  Config cfg=verifiedFakeConfig();cfg.servoLimitsVerified=false;Rig s(cfg);s.handshake();
  CHECK(s.prepare().error==Error::ServoUnverified);
  CHECK(PIN_PAN==10 && PIN_TILT==11);
}
void explicit_prepare_and_stop() {
  Rig r;r.step(3000);CHECK(r.c.state()==State::Fault && !r.h.servo);
  r.arm();CHECK(r.c.armEpoch()==1);CHECK(r.servo().ok);CHECK(r.h.pan==1500 && r.h.tilt==1550);
  Command stop;stop.type=Type::Stop;CHECK(r.c.command(stop).ok);
  CHECK(r.c.state()==State::Disarmed && !r.h.servo);
  CHECK(r.prepare().ok);CHECK(r.c.armEpoch()==2 && !r.h.servo);
}
void servo_limits_and_bench() {
  Rig r;r.arm();CHECK(r.servo(1600,1400).ok);
  CHECK(r.servo(1601).error==Error::Range);CHECK(!r.h.servo && r.c.faults()&InvalidCommand);
  Rig b;b.arm(Mode::Bench);CHECK(b.servo().error==Error::ServoUnverified);CHECK(!b.h.servo);
  Config cfg=verifiedFakeConfig();cfg.panMinUs=1700;cfg.panMaxUs=1600;
  Rig bad(cfg);bad.handshake();CHECK(bad.prepare().error==Error::ServoUnverified);
}
void link_usb_and_wrap() {
  Rig r;r.arm();CHECK(r.servo().ok);r.step(200);CHECK(r.c.active());r.step(1);
  CHECK(r.c.faults()&LinkTimeout);CHECK(!r.h.servo);
  Rig s;s.arm();s.h.usb=false;s.c.checkSafety();CHECK(s.c.faults()&UsbLost);
  Rig w;w.h.now=0xfffffff0u;w.arm();w.step(201);CHECK(w.c.faults()&LinkTimeout);
  Rig hold;hold.arm();CHECK(hold.servo().ok);hold.step(1000,true);CHECK(hold.c.active() && hold.h.servo);
}
void replay_epochs_and_sessions() {
  Rig r;r.arm();Command b;b.type=Type::Heartbeat;b.session=42;b.sequence=r.seq;
  CHECK(r.c.command(b).error==Error::Replay);CHECK(!r.h.servo);
  Rig s;s.arm();CHECK(s.servo(1500,1500,s.c.armEpoch()+1).error==Error::WrongEpoch);
  Rig q;q.arm();Command hello;hello.type=Type::Hello;hello.session=7;hello.sequence=1;
  CHECK(q.c.command(hello).error==Error::SessionWhileActive);CHECK(!q.c.active());
  Rig z;z.arm();Command wrong;wrong.type=Type::Heartbeat;wrong.session=41;wrong.sequence=++z.seq;
  CHECK(z.c.command(wrong).error==Error::NoSession);CHECK(z.c.faults()&InvalidCommand);
}
void servo_failure_and_latch() {
  Rig r;r.arm();r.h.servoOk=false;
  CHECK(r.servo().error==Error::HardwareFailure);CHECK(r.c.faults()&ServoFailure);CHECK(!r.h.servo);
  r.h.servoOk=true;r.step(100);CHECK(r.c.state()==State::Fault && !r.h.servo);
  CHECK(r.send(Type::Stop).ok);CHECK(r.c.state()==State::Fault);
  Rig slow;slow.arm();slow.h.servoDelay=201;
  CHECK(!slow.servo().ok);CHECK(slow.c.faults()&LinkTimeout);CHECK(!slow.h.servo);
}
void watchdog_guards() {
  FakeHal h;Controller c(verifiedFakeConfig(),h);c.begin(false);CHECK(c.faults()&WatchdogUnavailable);
  Command hello;hello.type=Type::Hello;hello.session=1;hello.sequence=1;CHECK(c.command(hello).ok);
  hello.type=Type::ClearFault;hello.sequence=2;CHECK(c.command(hello).error==Error::WatchdogUnavailable);
  Rig r;r.handshake();Command w;w.type=Type::WatchdogTest;w.session=42;w.sequence=++r.seq;
  CHECK(r.c.command(w).error==Error::InputsUnsafe);w.confirmOutputsOff=true;w.sequence=++r.seq;
  CHECK(r.c.command(w).ok && r.c.watchdogTestRequested());CHECK(!r.prepare().ok);
  Rig active;active.arm();w.sequence=++active.seq;CHECK(!active.c.command(w).ok);
}
void strict_protocol() {
  Command c;
  const char *valid=R"({"type":"servo","session":1,"sequence":15,"arm_epoch":2,"pan_us":1500,"tilt_us":1550})";
  CHECK(parseCommand(valid,strlen(valid),c));CHECK(c.panUs==1500 && c.sequence==15);
  const char *bad[]={
    R"({"type":"status","type":"stop"})",R"({type:"status"})",R"({'type':'status'})",
    R"({"type":"status",})",R"({"type":"status"}{})",R"({"type":"status","junk":1})",
    R"({"type":"heartbeat","session":true,"sequence":1})",
    R"({"type":"heartbeat","session":1,"sequence":1.5})",
    R"({"type":"heartbeat","session":1,"sequence":-1})",
    R"({"type":"heartbeat","session":01,"sequence":1})",
    R"({"type":"heartbeat","session":+1,"sequence":1})",
    R"({"type":"heartbeat","session":1,"sequence":1.})",
    R"({"type":"servo","session":1,"sequence":1,"arm_epoch":1,"pan_us":1500})",
    R"({"type":"servo","session":1,"sequence":1,"arm_epoch":1,"pan_us":true,"tilt_us":1500})",
    R"({"type":"servo","session":1,"sequence":1,"pan_us":1500,"tilt_us":1500})",
    R"({"type":"status","session":{"a":1}})",
    R"({"type":"motion","session":1,"sequence":1,"arm_epoch":1,"wheels_rpm":[0,0,0,0]})",
    R"({"type":"encoder_raw","session":1,"sequence":1})",
    R"({"type":"prepare","session":1,"sequence":1,"mode":"drive"})",
    R"({"type":"prepare","session":1,"sequence":1,"mode":"head","motors_unplugged":true})"
  };
  for(const char *p:bad)CHECK(!parseCommand(p,strlen(p),c));
  Rig active;active.arm();active.c.malformed();CHECK(active.c.faults()&InvalidCommand);
}
void status_contract() {
  char out[1024];Rig r;CHECK(writeStatus(out,sizeof(out),r.c,9,250,0)>0);
  StaticJsonDocument<2048> d;CHECK(!deserializeJson(d,out));
  CHECK(d["reply_to"]==9 && d["protocol_version"]=="head-1" && d["state"]=="FAULT");
  CHECK(d["power_cutoff_available"]==false && d["pwm_output_disabled"]==true);
  CHECK(!d.containsKey("requested_rpm") && !d.containsKey("targets_counts") && !d.containsKey("calibration_ready"));
  CHECK(writeStatus(out,10,r.c,9,250,0)==0);
}
int main() {
  const std::pair<const char*,std::function<void()>> tests[]={
    {"boot and commissioning gates",boot_and_guards},{"explicit prepare and stop",explicit_prepare_and_stop},
    {"servo limits and bench",servo_limits_and_bench},{"link USB and clock wrap",link_usb_and_wrap},
    {"replay epochs and sessions",replay_epochs_and_sessions},{"servo failure and latch",servo_failure_and_latch},
    {"watchdog guards",watchdog_guards},{"strict protocol and removed commands",strict_protocol},
    {"head-only status contract",status_contract}};
  size_t passed=0;
  for(const auto &t:tests){try{t.second();passed++;std::cout<<"PASS "<<t.first<<"\n";}
    catch(const std::exception &e){std::cerr<<"FAIL "<<t.first<<": "<<e.what()<<"\n";return 1;}}
  std::cout<<passed<<" native test groups passed\n";return 0;
}
