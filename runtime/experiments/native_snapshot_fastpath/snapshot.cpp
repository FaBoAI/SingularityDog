// File-only fixed Record ABI decoder. No FD, serial, CAN, IMU or motor API.
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

extern "C" {
struct Record {
  uint64_t start_ns, finish_ns, read_start_ns, received_ns, deadline_ns;
  uint8_t tx[17], rx[17];
  uint32_t written, received;
};
struct Motor {
  uint32_t motor_id, parameter; // 0=position, 1=velocity
  double value;
  uint64_t request_ns, received_ns, age_upper_bound_ns;
};
struct Result {
  Motor motors[24];
  uint32_t count, composite;
  uint64_t oldest, latest, earliest_receive;
};
}
static_assert(sizeof(Record)==88 && offsetof(Record,written)==76 && offsetof(Record,received)==80);
static_assert(sizeof(Motor)==40 && sizeof(Result)==992);
static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559);
static_assert(std::endian::native==std::endian::little);

namespace {
struct Frame { uint32_t can_id; uint8_t flags; const uint8_t* data; };
enum Error { OK=0, NONCAUSAL=1, MALFORMED=2, CROSS_BUS=3, STOP_INVALID=4,
             TYPE17_REQUEST=5, REPLY_MISMATCH=6, TYPE17_VALUE=7,
             NOT_CYCLE=8, DUPLICATE=9, MISSING=10 };

uint32_t be32(const uint8_t* p) {
  return (uint32_t(p[0])<<24)|(uint32_t(p[1])<<16)|(uint32_t(p[2])<<8)|p[3];
}
uint16_t be16(const uint8_t* p) { return (uint16_t(p[0])<<8)|p[1]; }
uint32_t le32(const uint8_t* p) {
  return uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);
}
bool parse(const uint8_t* p, Frame& f) {
  if (p[0]!='A'||p[1]!='T'||p[6]!=8||p[15]!='\r'||p[16]!='\n') return false;
  const auto encoded=be32(p+2);
  f={encoded>>3,uint8_t(encoded&7),p+7};
  return true;
}
uint32_t kind(const Frame& f) { return (f.can_id>>24)&31; }
uint32_t source(const Frame& f) { return (f.can_id>>8)&255; }
uint32_t destination(const Frame& f) { return f.can_id&255; }
bool canonical_request(const uint8_t* wire, uint32_t mid, uint32_t type, uint32_t parameter) {
  if (wire[0]!='A'||wire[1]!='T'||wire[6]!=8||wire[15]!='\r'||wire[16]!='\n') return false;
  const uint32_t id=(type<<24)|(0xfd<<8)|mid;
  if (be32(wire+2)!=((id<<3)|4)) return false;
  if (type==17 && (wire[7]!=(parameter==0?0x19:0x1b)||wire[8]!=0x70)) return false;
  if (type==4 && (wire[7]!=0||wire[8]!=0)) return false;
  for(int i=9;i<15;++i) if(wire[i]!=0) return false;
  return true;
}
bool append(Result& out, uint32_t& seen, uint32_t mid, uint32_t parameter,
            double value, const Record& record, uint64_t tick_ns) {
  const uint32_t bit=1u<<((mid-1)*2+parameter);
  if (seen&bit || out.count>=24) return false;
  seen|=bit;
  out.motors[out.count++]={mid,parameter,value,record.start_ns,record.received_ns,
                            tick_ns-record.start_ns};
  if (out.count==1) {
    out.oldest=record.start_ns; out.latest=record.received_ns;
    out.earliest_receive=record.received_ns;
  } else {
    if(record.start_ns<out.oldest) out.oldest=record.start_ns;
    if(record.received_ns>out.latest) out.latest=record.received_ns;
    if(record.received_ns<out.earliest_receive) out.earliest_receive=record.received_ns;
  }
  return true;
}
int batch(const Record* records, uint32_t count, uint32_t scope, uint64_t tick_ns,
          Result& out, uint32_t& seen) {
  for(uint32_t i=0;i<count;++i) {
    const auto& r=records[i];
    if (!(r.written==17 && r.received==17 && r.start_ns>0 &&
          r.start_ns<=r.finish_ns && r.finish_ns<=r.received_ns &&
          r.received_ns<r.deadline_ns && r.received_ns<=tick_ns)) return NONCAUSAL;
    Frame tx{},rx{};
    if(!parse(r.tx,tx)||!parse(r.rx,rx)) return MALFORMED;
    const auto mid=destination(tx);
    if(mid<(scope==0?1u:7u)||mid>(scope==0?6u:12u)) return CROSS_BUS;
    if(kind(tx)==4) {
      out.composite=1;
      if(!canonical_request(r.tx,mid,4,0)||rx.flags!=4||
         rx.can_id!=((2u<<24)|(mid<<8)|0xfdu)||
         (rx.data[0]==0 && rx.data[1]==0xc4 && rx.data[2]==0x56)) return STOP_INVALID;
      const auto p=be16(rx.data),v=be16(rx.data+2);
      const double position=p*(2.*12.57)/65535.-12.57;
      const double velocity=v*100./65535.-50.;
      if(!append(out,seen,mid,0,position,r,tick_ns)||
         !append(out,seen,mid,1,velocity,r,tick_ns)) return DUPLICATE;
    } else if(kind(tx)==17) {
      const uint32_t parameter=(tx.data[0]==0x19 && tx.data[1]==0x70)?0:1;
      if(!canonical_request(r.tx,mid,17,parameter)) return TYPE17_REQUEST;
      if(rx.flags!=4||source(rx)!=mid||kind(rx)!=17||
         destination(rx)!=0xfd||rx.data[0]!=(parameter==0?0x19:0x1b)||
         rx.data[1]!=0x70) return REPLY_MISMATCH;
      const auto status=(rx.can_id>>16)&255;
      if(rx.data[2]!=0||rx.data[3]!=0||status!=0) return TYPE17_VALUE;
      const float decoded=std::bit_cast<float>(le32(rx.data+4));
      if(!std::isfinite(decoded)) return TYPE17_VALUE;
      if(!append(out,seen,mid,parameter,double(decoded),r,tick_ns)) return DUPLICATE;
    } else return NOT_CYCLE;
  }
  return OK;
}
}

extern "C" uint32_t sd_snapshot_abi() { return 1; }
extern "C" int sd_snapshot_parse(const Record* first, uint32_t first_count, uint32_t first_scope,
    const Record* second, uint32_t second_count, uint32_t second_scope,
    uint64_t tick_ns, Result* out) {
  if(!out || (first_count && !first) || (second_count && !second) ||
     first_scope>1 || second_scope>1) return NONCAUSAL;
  *out=Result{};
  uint32_t seen=0;
  int status=batch(first,first_count,first_scope,tick_ns,*out,seen);
  if(status) return status;
  status=batch(second,second_count,second_scope,tick_ns,*out,seen);
  if(status) return status;
  return seen==0x00ffffffu?OK:MISSING;
}
