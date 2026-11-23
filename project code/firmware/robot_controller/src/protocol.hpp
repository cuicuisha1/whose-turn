#pragma once
#include <ArduinoJson.h>
#include "controller.hpp"

namespace robot {
constexpr size_t MAX_FRAME=384;

inline bool jsonNumber(const char *s,size_t n) {
  size_t i=0;if(i<n && s[i]=='-')i++;
  if(i==n)return false;
  if(s[i]=='0')i++;
  else {if(s[i]<'1' || s[i]>'9')return false;while(i<n && s[i]>='0' && s[i]<='9')i++;}
  if(i<n && s[i]=='.') {i++;size_t start=i;while(i<n && s[i]>='0' && s[i]<='9')i++;if(i==start)return false;}
  if(i<n && (s[i]=='e' || s[i]=='E')) {
    i++;if(i<n && (s[i]=='+' || s[i]=='-'))i++;
    size_t start=i;while(i<n && s[i]>='0' && s[i]<='9')i++;if(i==start)return false;
  }
  return i==n;
}

// One flat JSON object, no escaped strings, duplicate keys, nested objects,
// embedded NUL or trailing JSON. The public vocabulary needs no escapes.
inline bool frameShape(const char *s,size_t n) {
  if(!n || n>MAX_FRAME)return false;
  char keys[12][32]={{0}};size_t keyCount=0;
  bool quoted=false,opened=false,closed=false;int arrays=0;
  for(size_t i=0;i<n;i++) {
    const unsigned char ch=s[i];
    if(ch==0 || ch=='\\' || ch=='\'' || (ch<32 && ch!='\t' && ch!='\r' && ch!='\n'))return false;
    if(closed) {if(ch!=' ' && ch!='\t' && ch!='\r' && ch!='\n')return false;continue;}
    if(ch=='"') {
      if(!opened)return false;
      if(!quoted) {
        size_t end=i+1;while(end<n && s[end]!='"')end++;
        if(end==n)return false;
        size_t next=end+1;while(next<n && (s[next]==' ' || s[next]=='\t' || s[next]=='\r' || s[next]=='\n'))next++;
        if(next<n && s[next]==':') {
          if(arrays || keyCount>=12 || end-i-1>=32 || end==i+1)return false;
          const size_t len=end-i-1;
          for(size_t k=0;k<keyCount;k++)if(strlen(keys[k])==len && !memcmp(keys[k],s+i+1,len))return false;
          memcpy(keys[keyCount++],s+i+1,len);
        }
      }
      quoted=!quoted;continue;
    }
    if(quoted) {if(ch<32)return false;continue;}
    if(!opened) {
      if(ch==' ' || ch=='\t' || ch=='\r' || ch=='\n')continue;
      if(ch!='{')return false;opened=true;continue;
    }
    if(ch=='{')return false;
    if((ch>='0' && ch<='9') || ch=='-' || ch=='+' || ch=='.') {
      size_t end=i;
      while(end<n && s[end]!=',' && s[end]!=']' && s[end]!='}' && s[end]!=' ' && s[end]!='\t' && s[end]!='\r' && s[end]!='\n')end++;
      if(!jsonNumber(s+i,end-i))return false;
      i=end-1;continue;
    }
    if(ch==':' || ch=='}' || ch==']') {
      size_t prev=i;while(prev && (s[prev-1]==' ' || s[prev-1]=='\t' || s[prev-1]=='\r' || s[prev-1]=='\n'))prev--;
      if(!prev || (ch==':' && s[prev-1]!='"') || s[prev-1]==',' || s[prev-1]==':')return false;
    }
    if(ch=='[') {if(++arrays>1)return false;}
    if(ch==']') {if(--arrays<0)return false;}
    if(ch=='}') {if(arrays)return false;closed=true;}
  }
  return opened && closed && !quoted && arrays==0;
}
inline bool fieldAllowed(const char *key,const char *extra1=nullptr,const char *extra2=nullptr) {
  return !strcmp(key,"type") || !strcmp(key,"session") || !strcmp(key,"sequence") ||
         (extra1 && !strcmp(key,extra1)) || (extra2 && !strcmp(key,extra2));
}
inline bool unsignedField(JsonObjectConst o,const char *key,uint32_t &v,bool required=true) {
  if(!o.containsKey(key))return !required;
  if(!o[key].is<uint32_t>())return false;
  v=o[key].as<uint32_t>();return true;
}
inline bool parseCommand(const char *line,size_t n,Command &c) {
  c=Command();
  if(!frameShape(line,n))return false;
  StaticJsonDocument<1024> doc;
  if(deserializeJson(doc,line,n) || !doc.is<JsonObject>())return false;
  JsonObjectConst o=doc.as<JsonObjectConst>();
  if(!o["type"].is<const char*>())return false;
  const char *t=o["type"];
  const char *x=nullptr,*y=nullptr;
  if(!strcmp(t,"hello"))c.type=Type::Hello;
  else if(!strcmp(t,"status"))c.type=Type::Status;
  else if(!strcmp(t,"stop"))c.type=Type::Stop;
  else if(!strcmp(t,"clear_fault"))c.type=Type::ClearFault;
  else if(!strcmp(t,"heartbeat"))c.type=Type::Heartbeat;
  else if(!strcmp(t,"prepare")) {c.type=Type::Prepare;x="mode";}
  else if(!strcmp(t,"servo")) {c.type=Type::Servo;x="pan_us";y="tilt_us";}
  else if(!strcmp(t,"watchdog_test")) {c.type=Type::WatchdogTest;x="confirm";}
  else return false;
  for(JsonPairConst kv:o) {
    const char *k=kv.key().c_str();
    if(!fieldAllowed(k,x,y) && !(c.type==Type::Servo && !strcmp(k,"arm_epoch")))return false;
  }
  const bool identity=c.type!=Type::Status && c.type!=Type::Stop;
  if(!unsignedField(o,"session",c.session,identity) || !unsignedField(o,"sequence",c.sequence,identity))return false;
  if(identity && (!c.session || !c.sequence))return false;
  if(c.type==Type::Prepare) {
    if(!o["mode"].is<const char*>())return false;
    const char *m=o["mode"];
    if(!strcmp(m,"head"))c.mode=Mode::Head;
    else if(!strcmp(m,"bench"))c.mode=Mode::Bench;
    else return false;

  }
  if(c.type==Type::Servo) {
    if(!unsignedField(o,"arm_epoch",c.armEpoch) || !c.armEpoch)return false;
  }
  if(c.type==Type::Servo) {
    uint32_t p=0,q=0;
    if(!unsignedField(o,"pan_us",p) || !unsignedField(o,"tilt_us",q) || p>65535 || q>65535)return false;
    c.panUs=uint16_t(p);c.tiltUs=uint16_t(q);
  }
  if(c.type==Type::WatchdogTest) {
    if(!o["confirm"].is<const char*>() || strcmp(o["confirm"],"outputs_off"))return false;
    c.confirmOutputsOff=true;
  }
  return true;
}

inline size_t writeAck(char *out,size_t cap,const Reply &r,const Controller &c) {
  StaticJsonDocument<768> d;
  d["type"]="ack";d["sequence"]=r.sequence;d["ok"]=r.ok;d["error"]=errorName(r.error);
  d["session"]=c.session();d["state"]=stateName(c.state());d["arm_epoch"]=c.armEpoch();d["faults"]=c.faults();
  if(d.overflowed() || measureJson(d)+1>cap)return 0;
  return serializeJson(d,out,cap);
}
inline size_t writeStatus(char *out,size_t cap,const Controller &c,uint32_t replyTo,uint32_t watchdogMs,uint32_t dropped) {
  StaticJsonDocument<1792> d;
  d["type"]="status";d["reply_to"]=replyTo;d["state"]=stateName(c.state());d["mode"]=modeName(c.mode());
  d["session"]=c.session();d["arm_epoch"]=c.armEpoch();d["faults"]=c.faults();d["last_sequence"]=c.lastSequence();
  d["servo_active"]=c.servoActive();d["pan_us"]=c.panUs();d["tilt_us"]=c.tiltUs();
  d["hardware_verified"]=c.config().powerWiringVerified;
  d["protocol_version"]="head-1";d["power_cutoff_available"]=false;
  d["servo_verified"]=validServoLimits(c.config());
  d["pwm_output_disabled"]=!c.servoActive();
  d["watchdog_ms"]=watchdogMs;d["link_age_ms"]=c.linkAge();d["tx_dropped"]=dropped;
  if(d.overflowed() || measureJson(d)+1>cap)return 0;
  return serializeJson(d,out,cap);
}
}
