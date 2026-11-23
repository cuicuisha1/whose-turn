#include <iostream>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <chrono>
#include "fake_hal.hpp"
#include "protocol.hpp"
using namespace robot;
int main() {
  std::cerr<<"SIMULATION ONLY: head-only fake HAL, no GPIO/USB hardware is controlled.\n";
  FakeHal hal;Controller c(verifiedFakeConfig(),hal);c.begin(true);
  std::mutex lock;std::queue<std::string> lines;std::atomic<bool> ended{false};
  std::thread reader([&] {std::string line;while(std::getline(std::cin,line)) {std::lock_guard<std::mutex> guard(lock);lines.push(line);}ended=true;});
  const auto start=std::chrono::steady_clock::now();uint32_t nextStatus=0;
  int result=0;
  while(!ended) {
    const uint32_t now=uint32_t(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count());
    hal.advance(now-hal.now);
    c.tick();std::string line;
    {std::lock_guard<std::mutex> guard(lock);if(!lines.empty()) {line=lines.front();lines.pop();}}
    if(!line.empty()) {
      StaticJsonDocument<512> sim;
      const bool parsed=!deserializeJson(sim,line);
      const char *type=parsed?(sim["type"]|""):"";
      if(!strncmp(type,"sim_",4)) {
        bool ok=true;
        if(!strcmp(type,"sim_servo_fail"))hal.servoOk=false;
        else if(!strcmp(type,"sim_servo_restore"))hal.servoOk=true;
        else ok=false;
        sim.clear();sim["type"]="ack";sim["sequence"]=0;sim["ok"]=ok;sim["simulation"]=true;
        serializeJson(sim,std::cout);std::cout<<std::endl;
      } else {
        Command cmd;Reply r=parseCommand(line.data(),line.size(),cmd)?c.command(cmd):c.malformed();char out[1024];
        if(r.ok && cmd.type==Type::Status)writeStatus(out,sizeof(out),c,cmd.sequence,250,0);
        else writeAck(out,sizeof(out),r,c);
        std::cout<<out<<std::endl;
      }
    }
    if(now-nextStatus>=100) {char out[1024];writeStatus(out,sizeof(out),c,0,250,0);std::cout<<out<<std::endl;nextStatus=now;}
    if(c.watchdogTestRequested()) {std::cerr<<"Simulated watchdog test: process exits; actual WDT not tested.\n";result=78;break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  c.trip(UsbLost);
  if(ended)reader.join();else reader.detach();
  // On intentional WDT simulation, exit immediately so detached stdin thread
  // cannot access destroyed synchronization objects.
  if(result)std::_Exit(result);
  return result;
}
