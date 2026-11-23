#include <WDT.h>
#include "arduino_hal.hpp"
#include "config.hpp"
#include "src/protocol.hpp"

robot::ArduinoHal io;
robot::Controller controller(robot::productionConfig(),io);
uint32_t watchdogMs=0,txDropped=0,lastStatus=0;

// A whole JSON line is enqueued or dropped. USB writes are bounded and never
// wait for a host to consume a complete line.
class TxRing {
public:
  void put(const char *p,size_t n) {
    if(!n || n+1>sizeof(data_)-used_) {txDropped++;return;}
    for(size_t i=0;i<n;i++)push(p[i]);push('\n');
  }
  void pump() {
    if(!Serial) {head_=tail_=used_=0;return;}
    int room = Serial.availableForWrite();
   // UNO R4 WiFi 的 UART availableForWrite() 可能始终返回 0。
   // 临时采用每次最多发送 16 字节的受限发送方式。
    size_t budget = room > 0 ? size_t(room) : 16;

    if (budget > 64) budget = 64;
    while(used_ && budget) {
      size_t n=sizeof(data_)-tail_;if(n>used_)n=used_;if(n>budget)n=budget;
      size_t sent=Serial.write(reinterpret_cast<const uint8_t*>(data_+tail_),n);
      if(!sent)break;tail_=(tail_+sent)%sizeof(data_);used_-=sent;budget-=sent;
    }
  }
private:
  char data_[2048];size_t head_=0,tail_=0,used_=0;
  void push(char ch) {data_[head_]=ch;head_=(head_+1)%sizeof(data_);used_++;}
} tx;

void sendStatus(uint32_t replyTo=0) {
  char out[1024];tx.put(out,robot::writeStatus(out,sizeof(out),controller,replyTo,watchdogMs,txDropped));
}
void processFrame(const char *line,size_t n) {
  robot::Command c;
  robot::Reply r=robot::parseCommand(line,n,c)?controller.command(c):controller.malformed();
  char out[768];
  if(r.ok && c.type==robot::Type::Status)sendStatus(c.sequence);
  else tx.put(out,robot::writeAck(out,sizeof(out),r,controller));
}
void receiveBounded() {
  static char line[robot::MAX_FRAME+1];static size_t length=0;
  static bool discarding=false;static uint32_t started=0;
  if(length && millis()-started>100) {
    controller.malformed();length=0;discarding=true;
  }
  for(size_t budget=0;budget<128 && Serial.available();budget++) {
    const int value=Serial.read();if(value<0)break;const char ch=char(value);
    controller.checkSafety();
    if(ch=='\n') {
      if(!discarding && length) {line[length]=0;processFrame(line,length);}
      length=0;discarding=false;
    } else if(!discarding) {
      if(length==0)started=millis();
      if(length>=robot::MAX_FRAME || ch==0) {controller.malformed();length=0;discarding=true;}
      else line[length++]=ch;
    }
  }
}
void setup() {
  io.beginPins();Serial.begin(115200); // Never wait for a USB host.
  const bool ready=WDT.begin(250)==1;
  if(ready)watchdogMs=WDT.getTimeout();
  controller.begin(ready);
}
void loop() {
  const uint32_t start=millis();
  controller.tick();receiveBounded();controller.checkSafety();
  if(millis()-lastStatus>=250) {lastStatus=millis();if(Serial)sendStatus();}
  tx.pump();
  if(controller.watchdogTestRequested()) {
    io.safeOff(); // Intentional reset test, with outputs already disabled.
    while(true) {tx.pump();} // Deliberately no WDT.refresh().
  }
  if(millis()-start>robot::MAX_LOOP_MS)controller.trip(robot::LoopOverrun);
  // A blocked driver cannot reach here; the internal watchdog then resets MCU.
  if(controller.watchdogReady())WDT.refresh();
}
