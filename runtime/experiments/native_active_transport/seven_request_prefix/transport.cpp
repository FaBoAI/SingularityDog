// Separate active RS05 transport. The diagnostic allowlist is not modified.
// No device opening/configuration, shell, network, allocation in the RX loop, or retry.
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <condition_variable>
#include <chrono>
#include <fcntl.h>
#include <mutex>
#include <new>
#include <set>
#include <thread>
#ifndef __APPLE__
#include <sched.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#endif
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

extern "C" {
struct SDRecord {
    uint64_t start_ns, finish_ns, read_start_ns, received_ns, deadline_ns;
    unsigned char tx[17], rx[17];
    uint32_t written, received;
};
struct SDStats {
    uint64_t begin_ns, end_ns, waits, reads, bytes, writes;
    unsigned char rejected[4096];
    uint32_t rejected_size;
    uint64_t rejected_total;
};
struct SDLimits { double lower[6], upper[6], kp[6], kd[6]; };
struct SDStopResult { uint32_t attempted_mask, confirmed_mask, ambiguous_mask, fault[6]; };
struct SDPairStats {
    uint64_t generation, submitted_ns, validated_ns, released_ns;
    uint64_t owner_started_ns[2], owner_finished_ns[2], cancel_requested_ns;
    int32_t owner_status[2];
};
struct SDPairOwnerSettings {
    uint64_t native_tid, cpu_mask, timer_slack_ns, original_cpu_mask, original_timer_slack_ns;
    int32_t status, configured, restored;
};
// Separate optional offline codec ABI; no transport/session/descriptor access.
struct SDFeedbackDecoded {
    uint32_t motor_id, mode_state, fault_bits, position_u16;
    double protocol_position_rad, velocity_rad_s, torque_nm, temperature_c;
    uint64_t request_started_ns, received_ns;
};
uint32_t sda_abi() { return 1; }
}
namespace {
uint64_t now() {
    timespec t{};
#ifdef __APPLE__
    if (clock_gettime(CLOCK_UPTIME_RAW, &t)) return 0;
#else
    if (clock_gettime(CLOCK_MONOTONIC, &t)) return 0;
#endif
    return uint64_t(t.tv_sec)*1000000000ULL+uint64_t(t.tv_nsec);
}
uint32_t id(const unsigned char *w) {
    return (uint32_t(w[2])<<24|uint32_t(w[3])<<16|uint32_t(w[4])<<8|w[5])>>3;
}
uint16_t be16(const unsigned char *p) { return uint16_t(p[0])<<8|p[1]; }
bool framing(const unsigned char *w) {
    return w[0]=='A'&&w[1]=='T'&&(w[5]&7)==4&&w[6]==8&&w[15]==13&&w[16]==10;
}
bool zero(const unsigned char *p, int n) {
    for(int i=0;i<n;++i) if(p[i]) return false;
    return true;
}
bool version_request(const unsigned char *w) {
    return (id(w)>>24)==4 && w[7]==0 && w[8]==0xc4 && zero(w+9,6);
}
void retain(SDStats *s, const unsigned char *p, size_t n) {
    s->rejected_total+=n;
    const size_t copy=std::min(n,sizeof(s->rejected)-s->rejected_size);
    std::memcpy(s->rejected+s->rejected_size,p,copy); s->rejected_size+=uint32_t(copy);
}
std::mutex owners_mutex;
std::set<std::array<uint64_t,3>> owners;
struct Session {
    int fd, cancel_fd, boot_fd, first;
    char boot[37]; SDLimits limits;
    uint64_t gap, last_finish=0;
    uint32_t window, ambiguous=0;
    bool poisoned=false;
    void *pair_owner=nullptr;
    std::atomic<bool> paired_phase{false};
    std::array<uint64_t,3> binding;
    std::mutex mutex;
};
bool binding(int fd, std::array<uint64_t,3> &out) {
    struct stat st{};
    if(fstat(fd,&st)) return false;
    out={uint64_t(st.st_dev),uint64_t(st.st_ino),uint64_t(st.st_rdev)};
    return true;
}
bool fd_ok(Session *s) {
    std::array<uint64_t,3> actual{};
    const int flags=fcntl(s->fd,F_GETFL);
    return binding(s->fd,actual)&&actual==s->binding&&flags>=0&&(flags&O_NONBLOCK);
}
bool boot_ok(Session *s) {
    char b[80];const ssize_t n=pread(s->boot_fd,b,sizeof(b),0);
    return n>=36&&n<=38&&std::memcmp(b,s->boot,36)==0&&
        (n==36||b[36]=='\n')&&(n<=37||b[37]=='\n');
}
bool cancelled(Session *s, int sibling_cancel=-1) {
    fd_set f;FD_ZERO(&f);FD_SET(s->cancel_fd,&f);
    if(sibling_cancel>=0)FD_SET(sibling_cancel,&f);
    timespec t{};
    const int r=pselect(std::max(s->cancel_fd,sibling_cancel)+1,&f,nullptr,nullptr,&t,nullptr);
    return r<0||FD_ISSET(s->cancel_fd,&f)||(sibling_cancel>=0&&FD_ISSET(sibling_cancel,&f));
}
bool valid_request(const unsigned char *w, Session *s) {
    if(!framing(w)) return false;
    const uint32_t c=id(w), k=c>>24, mid=c&255;
    if(mid<uint32_t(s->first)||mid>=uint32_t(s->first+6)) return false;
    const int i=int(mid)-s->first;
    if(k==1) {
        // The torque occupies the middle 16 bits, not the host address.
        if(((c>>8)&65535)!=32767||be16(w+9)!=32767) return false;
        const double q=double(be16(w+7))*25.14/65535.-12.57;
        const double kp=double(be16(w+11))*500./65535.;
        const double kd=double(be16(w+13))*5./65535.;
        return q>=s->limits.lower[i]&&q<=s->limits.upper[i]&&
            kp<=s->limits.kp[i]&&kd<=s->limits.kd[i];
    }
    if(((c>>8)&65535)!=0xfd) return false;
    if(k==18) {
        static const unsigned char watchdog[8]={0x28,0x70,0,0,0xa0,0x0f,0,0};
        return std::memcmp(w+7,watchdog,8)==0;
    }
    if(k==0||k==3||k==4) return zero(w+7,8)||version_request(w);
    const uint16_t index=w[7]|uint16_t(w[8])<<8;
    return k==17&&(index==0x7019||index==0x701b||index==0x701c||
                       index==0x7028||index==0x7005)&&zero(w+9,6);
}

// PRIVATE diagnostic-only seven-request experiment. ABI1 paths call with no
// prefix owner and remain byte-for-byte on their original scheduling path.
struct SDSplitMeta {
    uint64_t generation, snapshot_ns, final_native_ns;
    uint32_t received_mask, scope; // scope1: partial combined seven-request phase
};
struct SplitPipeBinding {
    std::array<uint64_t,3> identity{}; mode_t mode=0; int flags=0;
};
struct SplitState {
    Session *session=nullptr;
    int hint_read=-1,hint_write=-1;
    SplitPipeBinding pipe[2];
    uint64_t generation=0;
    std::mutex lifecycle,publication;
    bool started=false,ready=false,terminal=false;
    bool live_hold=false;
    std::array<unsigned char,102> held_wires{};
    std::array<double,6> live_lower{},live_upper{};
    int final_status=0;
    std::array<SDRecord,6> prefix{}; SDStats stats{}; SDSplitMeta meta{};
    char final_error[256]{};
};
bool split_pipe_binding(int fd,int access,SplitPipeBinding &out) {
    struct stat st{};const int flags=fcntl(fd,F_GETFL);
    if(fd<0||fd>=FD_SETSIZE||flags<0||fstat(fd,&st)||!S_ISFIFO(st.st_mode)||
       !(flags&O_NONBLOCK)||(flags&O_ACCMODE)!=access)return false;
    out.identity={uint64_t(st.st_dev),uint64_t(st.st_ino),uint64_t(st.st_rdev)};
    out.mode=st.st_mode&S_IFMT;out.flags=flags&(O_ACCMODE|O_NONBLOCK);return true;
}
bool split_pipe_ok(SplitState *p) {
    for(int i=0;i<2;++i) {
        SplitPipeBinding actual{};
        if(!split_pipe_binding(i?p->hint_write:p->hint_read,i?O_WRONLY:O_RDONLY,actual)||
           actual.identity!=p->pipe[i].identity||actual.mode!=p->pipe[i].mode||
           actual.flags!=p->pipe[i].flags)return false;
    }
    return true;
}
bool publish_split_prefix(SplitState *p,SDRecord *records,SDStats *stats,uint64_t deadline,
                          const char *&failure) {
    if(!p)return true;
    // Readability is never readiness. Test ALL SIX ORIGINAL slots rather than
    // a reply counter: the seventh voltage reply may have arrived earlier.
    for(unsigned i=0;i<6;++i)if(records[i].received!=17)return true;
    std::lock_guard<std::mutex> lock(p->publication);
    if(p->ready)return true;
    if(p->terminal||!split_pipe_ok(p)){failure="Split prefix binding/state changed";return false;}
    for(unsigned i=0;i<6;++i) {
        const auto &r=records[i];
        const uint32_t c=id(r.rx);
        if(r.written!=17||!framing(r.rx)||(c>>24)!=2||((c>>22)&3)!=(p->live_hold?2U:0U)||
           ((c>>16)&63)!=0||!r.start_ns||r.start_ns>r.finish_ns||
           r.finish_ns>r.received_ns||r.received_ns>=deadline) {
            failure=p->live_hold?"LIVE held Type1 prefix fault/mode/causality rejected":"Split prefix STOP fault/mode/causality rejected";return false;
        }
    }
    if(cancelled(p->session)){failure="Cancelled before split prefix publication";return false;}
    const uint64_t actual=now();
    if(!actual||actual>=deadline){failure="Split prefix publication deadline exceeded";return false;}
    std::memcpy(p->prefix.data(),records,sizeof(SDRecord)*6);
    std::memcpy(&p->stats,stats,sizeof(*stats));
    // This clock bounds the scoped snapshot; it is NOT an exchange-completed
    // clock. Native final stats/end and all seven records remain separate.
    p->stats.end_ns=actual;p->meta.generation=p->generation;p->meta.snapshot_ns=actual;
    p->meta.scope=1;p->meta.received_mask=0;
    for(unsigned i=0;i<7;++i)if(records[i].received==17)p->meta.received_mask|=1U<<i;
    const unsigned char hint=1;
    if(write(p->hint_write,&hint,1)!=1){failure="Split prefix notification write failed";return false;}
    if(!split_pipe_ok(p)){failure="Split prefix notification binding changed";return false;}
    if(cancelled(p->session)){failure="Cancelled after split prefix publication";return false;}
    if(now()>=deadline){failure="Split prefix notification deadline exceeded";return false;}
    p->ready=true;return true;
}

bool matches(const unsigned char *tx,const unsigned char *rx) {
    const uint32_t a=id(tx),b=id(rx),k=a>>24;
    if(((b>>8)&255)!=(a&255)) return false;
    if(k==0) return b==((a&255)<<8|0xfe);
    if(k==1||k==3||k==4||k==18) {
        if((b>>24)!=2||(b&255)!=0xfd) return false;
        const uint32_t mode=(b>>22)&3,fault=(b>>16)&63;
        const bool version_reply=rx[7]==0&&rx[8]==0xc4&&rx[9]==0x56;
        // The stopped-only firmware reply is NOT position/velocity telemetry.
        // Neither direction may substitute for ordinary STOP/enable/motion ACKs.
        if(version_request(tx)) return version_reply&&mode==0&&fault==0;
        if(version_reply) return false;
        if(k==4) return mode==0; // STOP reports faults; it never clears them.
        if(k==18) return mode==0&&fault==0; // Volatile watchdog setup ACK.
        return fault==0&&(k==1?mode==2:(mode==0||mode==2));
    }
    if(b!=((17U<<24)|((a&255)<<8)|0xfd)||tx[7]!=rx[7]||tx[8]!=rx[8]||!zero(rx+9,2)) return false;
    const uint16_t index=tx[7]|uint16_t(tx[8])<<8;
    if(index==0x7028) return true; // uint32 watchdog ticks; caller checks exact 4000.
    if(index==0x7005) return rx[11]<=3&&zero(rx+12,3); // uint8 run mode.
    const uint32_t bits=rx[11]|uint32_t(rx[12])<<8|uint32_t(rx[13])<<16|uint32_t(rx[14])<<24;
    float value;std::memcpy(&value,&bits,4);return std::isfinite(value);
}
void stop_wire(int mid,unsigned char *w) {
    const uint32_t c=(((4U<<24)|(0xfd<<8)|uint32_t(mid))<<3)|4;
    std::memset(w,0,17);w[0]='A';w[1]='T';w[2]=c>>24;w[3]=c>>16;w[4]=c>>8;w[5]=c;
    w[6]=8;w[15]=13;w[16]=10;
}
}
extern "C" uint64_t sda_now_ns() {return now();}
extern "C" uint32_t sda_feedback_decode_abi() {
    // An unusual compiler packing/alignment option must not claim ABI 1 and
    // reinterpret the caller's genuine ctypes arrays at different offsets.
    const bool record_layout=sizeof(SDRecord)==88&&
        offsetof(SDRecord,start_ns)==0&&offsetof(SDRecord,finish_ns)==8&&
        offsetof(SDRecord,read_start_ns)==16&&offsetof(SDRecord,received_ns)==24&&
        offsetof(SDRecord,deadline_ns)==32&&offsetof(SDRecord,tx)==40&&
        offsetof(SDRecord,rx)==57&&offsetof(SDRecord,written)==76&&
        offsetof(SDRecord,received)==80;
    const bool decoded_layout=sizeof(SDFeedbackDecoded)==64&&
        offsetof(SDFeedbackDecoded,motor_id)==0&&offsetof(SDFeedbackDecoded,mode_state)==4&&
        offsetof(SDFeedbackDecoded,fault_bits)==8&&offsetof(SDFeedbackDecoded,position_u16)==12&&
        offsetof(SDFeedbackDecoded,protocol_position_rad)==16&&
        offsetof(SDFeedbackDecoded,velocity_rad_s)==24&&offsetof(SDFeedbackDecoded,torque_nm)==32&&
        offsetof(SDFeedbackDecoded,temperature_c)==40&&
        offsetof(SDFeedbackDecoded,request_started_ns)==48&&offsetof(SDFeedbackDecoded,received_ns)==56;
    return record_layout&&decoded_layout?1:0;
}
// Return 1 for an unsupported batch, -1 for invalid bytes/timestamps, and 0
// only for six complete Type2 feedback records on one explicitly selected bus.
// The caller falls back to the authoritative Python codec for either nonzero
// result, retaining its exact error messages. Raw records are never changed.
extern "C" int sda_feedback_decode_batch(const SDRecord *records,uint32_t count,
        int first_id,SDFeedbackDecoded *decoded,char *error,uint32_t size) {
    if(!records||!decoded||!error||!size)return -1;
    error[0]=0;
    auto fail=[&](const char *message,int status) {
        std::snprintf(error,size,"%s",message);return status;
    };
    if(count!=6||(first_id!=1&&first_id!=7))
        return fail("Unsupported six-axis feedback batch",1);
    std::array<SDFeedbackDecoded,6> rows{};uint32_t seen=0;
    for(uint32_t i=0;i<6;++i) {
        const SDRecord &record=records[i];
        if(record.written!=17||record.received!=17||!record.start_ns||
           record.start_ns>record.finish_ns||record.finish_ns>record.received_ns||
           record.received_ns>=record.deadline_ns)
            return fail("Incomplete or noncausal motor transaction",-1);
        if(!framing(record.tx)||!framing(record.rx))return fail("Invalid native frame",-1);
        const uint32_t tx=id(record.tx),rx=id(record.rx),kind=tx>>24,mid=tx&255;
        if((kind!=1&&kind!=3&&kind!=4&&kind!=18)||version_request(record.tx))
            return fail("Unsupported mixed or firmware feedback batch",1);
        if(mid<uint32_t(first_id)||mid>=uint32_t(first_id+6))
            return fail("Feedback record outside selected bus",1);
        if((rx>>24)!=2||((rx>>8)&255)!=mid||(rx&255)!=0xfd)
            return fail("Reply type/source/destination does not match selected motor / host FD",-1);
        if(record.rx[7]==0&&record.rx[8]==0xc4&&record.rx[9]==0x56)
            return fail("Version-shaped Type 2 reply is not position feedback",-1);
        const uint32_t mode=(rx>>22)&3;
        if(mode==3)return fail("Reserved Type 2 mode state",-1);
        const uint32_t bit=uint32_t(1)<<(mid-uint32_t(first_id));
        if(seen&bit)return fail("Duplicate transaction",-1);
        seen|=bit;
        auto &row=rows[i];row.motor_id=mid;row.mode_state=mode;row.fault_bits=(rx>>16)&63;
        row.position_u16=be16(record.rx+7);
        const uint16_t velocity=be16(record.rx+9),torque=be16(record.rx+11),temp=be16(record.rx+13);
        // CPython rounds each multiply/divide/add to double. Volatile staging
        // prevents fused or reassociated expressions even on ARM toolchains.
        volatile double position_product=double(row.position_u16)*25.14;
        volatile double position_quotient=position_product/65535.0;
        row.protocol_position_rad=position_quotient+(-12.57);
        volatile double velocity_product=double(velocity)*100.0;
        volatile double velocity_quotient=velocity_product/65535.0;
        row.velocity_rad_s=velocity_quotient-50.0;
        volatile double torque_product=double(torque)*11.0;
        volatile double torque_quotient=torque_product/65535.0;
        row.torque_nm=torque_quotient-5.5;
        row.temperature_c=double(temp)/10.0;
        row.request_started_ns=record.start_ns;row.received_ns=record.received_ns;
    }
    std::memcpy(decoded,rows.data(),sizeof(rows));return 0;
}
// Optional bounded release wait. It never receives a motor/serial descriptor or
// sends a command. The cancellation fd is checked both before and after the
// sleep/spin, and a missed deadline returns the actual monotonic wake time.
extern "C" int sda_wait_until(int cancel_fd,uint64_t deadline_ns,uint32_t spin_us,
        uint64_t *woke_ns,char *error,uint32_t error_size) {
    if(woke_ns)*woke_ns=0;
    if(!woke_ns||!error||!error_size)return -1;
    auto fail=[&](const char *message) {
        std::snprintf(error,error_size,"%s",message);return -1;
    };
    const uint64_t start=now();
    if(!start||!deadline_ns||cancel_fd<0||cancel_fd>=FD_SETSIZE||
       fcntl(cancel_fd,F_GETFL)<0||(spin_us!=200&&spin_us!=500)||
       (deadline_ns>start&&deadline_ns-start>1000000000ULL)||
       (start>deadline_ns&&start-deadline_ns>1000000000ULL))
        return fail("Invalid bounded active release wait arguments");
    auto check_cancel=[&](uint64_t wait_ns) {
        fd_set readable;FD_ZERO(&readable);FD_SET(cancel_fd,&readable);
        timespec timeout{time_t(wait_ns/1000000000ULL),long(wait_ns%1000000000ULL)};
        const int ready=pselect(cancel_fd+1,&readable,nullptr,nullptr,&timeout,nullptr);
        if(ready<0)return errno==EINTR?2:-1;
        return FD_ISSET(cancel_fd,&readable)?1:0;
    };
    uint32_t interrupts=0;
    const uint64_t spin_ns=uint64_t(spin_us)*1000ULL;
    while(true) {
        const int before=check_cancel(0);
        if(before==1)return fail("Cancelled before active release");
        if(before==-1)return fail("Active release cancellation check failed");
        if(before==2) {
            if(++interrupts>32)return fail("Active release interrupted too often");
            continue;
        }
        const uint64_t current=now();
        if(!current)return fail("Active release monotonic clock failed");
        if(current>=deadline_ns)break;
        const uint64_t remaining=deadline_ns-current;
        if(remaining>spin_ns) {
            const int ready=check_cancel(remaining-spin_ns);
            if(ready==1)return fail("Cancelled during active release");
            if(ready==-1)return fail("Active release wait failed");
            if(ready==2&&++interrupts>32)
                return fail("Active release interrupted too often");
            continue;
        }
        while(true) {
            const uint64_t spinning_now=now();
            if(!spinning_now)return fail("Active release monotonic clock failed");
            if(spinning_now>=deadline_ns)break;
        }
        break;
    }
    const int after=check_cancel(0);
    if(after==1)return fail("Cancelled after active release");
    if(after==-1||after==2)return fail("Active release final cancellation check failed");
    const uint64_t actual=now();
    if(!actual||actual<deadline_ns)return fail("Active release returned before deadline");
    *woke_ns=actual;return 0;
}
// Optional Future publication hint wait. Neither descriptor is a motor/serial
// reader and no byte is consumed. The Python owner drains its private pipe and
// rechecks the original Futures; readability alone never certifies completion.
extern "C" uint32_t sda_future_readiness_abi() {return 1;}
extern "C" int sda_wait_future_ready(int cancel_fd,int hint_read_fd,uint64_t deadline_ns,
        uint64_t *woke_ns,char *error,uint32_t error_size) {
    if(woke_ns)*woke_ns=0;
    if(!woke_ns||!error||!error_size)return -1;
    error[0]=0;
    auto fail=[&](const char *message) {
        std::snprintf(error,error_size,"%s",message);return -1;
    };
    const uint64_t begin=now();
    if(!begin||!deadline_ns||cancel_fd<0||hint_read_fd<0||cancel_fd==hint_read_fd||
       cancel_fd>=FD_SETSIZE||hint_read_fd>=FD_SETSIZE||
       (deadline_ns>begin&&deadline_ns-begin>1000000000ULL)||
       (begin>deadline_ns&&begin-deadline_ns>1000000000ULL))
        return fail("Invalid bounded Future readiness wait arguments");
    struct Descriptor {
        std::array<uint64_t,3> identity{};
        int flags=0;
        mode_t mode=0;
    } expected[2];
    const int descriptors[2]={cancel_fd,hint_read_fd};
    auto inspect=[&](unsigned index,Descriptor &out) {
        struct stat st{};const int flags=fcntl(descriptors[index],F_GETFL);
        if(flags<0||fstat(descriptors[index],&st)||(flags&O_ACCMODE)==O_WRONLY)return false;
        if(index==1&&(!S_ISFIFO(st.st_mode)||!(flags&O_NONBLOCK)||
                      (flags&O_ACCMODE)!=O_RDONLY))return false;
        out.identity={uint64_t(st.st_dev),uint64_t(st.st_ino),uint64_t(st.st_rdev)};
        out.flags=flags&(O_ACCMODE|O_NONBLOCK);out.mode=st.st_mode&S_IFMT;return true;
    };
    for(unsigned i=0;i<2;++i)
        if(!inspect(i,expected[i]))return fail("Invalid Future readiness descriptor binding/mode");
    auto unchanged=[&]() {
        for(unsigned i=0;i<2;++i) {
            Descriptor current{};
            if(!inspect(i,current)||current.identity!=expected[i].identity||
               current.flags!=expected[i].flags||current.mode!=expected[i].mode)return false;
        }
        return true;
    };
    uint32_t interrupts=0;
    auto select_both=[&](uint64_t wait_ns,fd_set &readable) {
        FD_ZERO(&readable);FD_SET(cancel_fd,&readable);FD_SET(hint_read_fd,&readable);
        timespec timeout{time_t(wait_ns/1000000000ULL),long(wait_ns%1000000000ULL)};
        return pselect(std::max(cancel_fd,hint_read_fd)+1,&readable,nullptr,nullptr,&timeout,nullptr);
    };
    while(true) {
        if(!unchanged())return fail("Future readiness descriptor binding/mode changed");
        // This zero-time check runs before each sleep and again after it. A
        // simultaneous cancellation always wins over a hint or deadline.
        fd_set immediate;const int before=select_both(0,immediate);
        if(before<0) {
            const int failure=errno;
            if(!unchanged())return fail("Future readiness descriptor binding/mode changed");
            if(failure==EINTR&&++interrupts<=32)continue;
            return fail("Future readiness cancellation check interrupted/failed");
        }
        if(!unchanged())return fail("Future readiness descriptor binding/mode changed");
        if(FD_ISSET(cancel_fd,&immediate))return fail("Cancelled Future readiness wait");
        const uint64_t stamp=now();
        if(!stamp)return fail("Future readiness monotonic clock failed");
        if(stamp>=deadline_ns) {*woke_ns=stamp;return 1;}
        if(FD_ISSET(hint_read_fd,&immediate)) {
            // Readability also includes EOF. Inspect the queued byte count
            // without consuming the owner's hint or accepting EOF as a wake.
            int available=0;const int inspected=ioctl(hint_read_fd,FIONREAD,&available);
            fd_set final_readable;const int final_ready=select_both(0,final_readable);
            if(final_ready<0) {
                const int failure=errno;
                if(!unchanged())return fail("Future readiness descriptor binding/mode changed");
                if(failure==EINTR&&++interrupts<=32)continue;
                return fail("Future readiness final cancellation check interrupted/failed");
            }
            if(!unchanged())return fail("Future readiness descriptor binding/mode changed");
            if(FD_ISSET(cancel_fd,&final_readable))return fail("Cancelled Future readiness wait");
            const uint64_t actual=now();
            if(!actual)return fail("Future readiness monotonic clock failed");
            if(actual>=deadline_ns) {*woke_ns=actual;return 1;}
            if(inspected<0)return fail("Future readiness pipe inspection failed");
            if(!available||!FD_ISSET(hint_read_fd,&final_readable))
                return fail("Future readiness pipe reached EOF or lost its hint");
            *woke_ns=actual;return 0;
        }
        fd_set readable;const int ready=select_both(deadline_ns-stamp,readable);
        if(ready<0) {
            const int failure=errno;
            if(!unchanged())return fail("Future readiness descriptor binding/mode changed");
            if(failure==EINTR&&++interrupts<=32)continue;
            return fail("Future readiness wait interrupted/failed");
        }
        if(!unchanged())return fail("Future readiness descriptor binding/mode changed");
        if(FD_ISSET(cancel_fd,&readable))return fail("Cancelled Future readiness wait");
        // Returning through the immediate check rechecks cancellation, the
        // current descriptor bindings and the actual clock after pselect.
    }
}
extern "C" void *sda_create(int fd,int cancel_fd,int boot_fd,const char *boot,
        int first,const SDLimits *limits,uint64_t gap,uint32_t window,char *error,uint32_t size) {
    auto bad=[&](const char *why)->void*{if(error&&size)std::snprintf(error,size,"%s",why);return nullptr;};
    if(!error||!size||!limits||!boot||std::strlen(boot)!=36||fd<0||fd>=FD_SETSIZE||
       cancel_fd<0||cancel_fd>=FD_SETSIZE||fd==cancel_fd||boot_fd<0||boot_fd==fd||boot_fd==cancel_fd||
       (first!=1&&first!=7)||gap<600000||gap>5000000||window<1||window>3)
        return bad("Invalid explicit active session configuration");
    for(int i=0;i<6;++i) {
        const double lo=limits->lower[i],hi=limits->upper[i],kp=limits->kp[i],kd=limits->kd[i];
        if(!std::isfinite(lo)||!std::isfinite(hi)||!std::isfinite(kp)||!std::isfinite(kd)||
           lo < -12.57||hi>12.57||lo>=hi||kp<0||kp>36||kd<0||kd>1)
            return bad("Invalid per-axis position/gain caps");
    }
    Session *s=new(std::nothrow) Session{};if(!s)return bad("Session allocation failed");
    s->fd=fd;s->cancel_fd=cancel_fd;s->boot_fd=boot_fd;s->first=first;s->limits=*limits;
    s->gap=gap;s->window=window;std::memcpy(s->boot,boot,37);
    if(!binding(fd,s->binding)||!fd_ok(s)||fcntl(cancel_fd,F_GETFL)<0||!boot_ok(s)) {
        delete s;return bad("Invalid FD binding/nonblocking mode/boot identity");
    }
    std::lock_guard<std::mutex> lock(owners_mutex);
    if(!owners.insert(s->binding).second){delete s;return bad("FD already owned by an active session");}
    return s;
}
extern "C" void sda_destroy(void *handle) {
    auto *s=static_cast<Session*>(handle);if(!s)return;
    // Borrowing pair threads must be destroyed before their sessions. Python
    // rejects this misuse explicitly; the C ABI also avoids a dangling handle.
    {std::lock_guard<std::mutex> lock(s->mutex);if(s->pair_owner)return;}
    {std::lock_guard<std::mutex> lock(owners_mutex);owners.erase(s->binding);}delete s;
}
static int exchange_owned(void *handle,const unsigned char *wires,uint32_t count,int send_only,
        uint64_t deadline,SDRecord *records,SDStats *stats,char *error,uint32_t size,
        void *pair_owner=nullptr,int sibling_cancel=-1,SplitState *prefix=nullptr) {
    auto *s=static_cast<Session*>(handle);
    if(!s||!records||!stats||!error||!size)return -1;
    std::memset(stats,0,sizeof(*stats));stats->begin_ns=now();
    std::unique_lock<std::mutex> lock(s->mutex,std::try_to_lock);
    if(!lock.owns_lock()){std::snprintf(error,size,"Concurrent native active session use");return -1;}
    if(s->paired_phase.load()&&s->pair_owner!=pair_owner) {
        stats->end_ns=now();std::snprintf(error,size,"Session borrowed by native pair phase");return -1;
    }
    unsigned char buffer[4096];size_t used=0;uint32_t sent=0;
    auto fail=[&](const char *why) {
        s->poisoned=true;
        for(uint32_t i=0;i<sent;++i)if(!records[i].received) {
            const uint32_t c=id(records[i].tx),k=c>>24;
            // A pending Type1 may reply mode0 after watchdog/fault disable.
            // Type2 has no sequence number: a later STOP cannot disambiguate it.
            if(k==1||k==3||k==4||k==18)s->ambiguous|=1U<<((c&255)-s->first);
        }
        if(used)retain(stats,buffer,used);
        stats->end_ns=now();std::snprintf(error,size,"%s",why);return -1;
    };
    if(s->poisoned)return fail("Session poisoned; active retry prohibited");
    // Keep the ABI field, but no active command may skip its acknowledgement.
    if(!wires||count<1||count>12||send_only!=0||!fd_ok(s)||
       !stats->begin_ns||deadline<=stats->begin_ns||deadline-stats->begin_ns>250000000)
        return fail("Invalid active exchange arguments/FD binding");
    std::memset(records,0,sizeof(*records)*count);
    uint32_t effective_window=s->window;
    for(uint32_t i=0;i<count;++i) {
        const auto *w=wires+i*17;
        if(!valid_request(w,s))return fail("Disallowed/noncanonical/out-of-bounds active command");
        // UID, watchdog setup and canonical firmware probes are startup work. Wait for each
        // reply before another write; motion/parameter batches keep the fast window.
        if((id(w)>>24)==0||(id(w)>>24)==18||version_request(w))effective_window=1;
        for(uint32_t j=0;j<i;++j) {
            const uint32_t a=id(w),b=id(wires+j*17);
            const uint32_t ak=a>>24,bk=b>>24;
            const bool type2a=ak==1||ak==3||ak==4||ak==18,type2b=bk==1||bk==3||bk==4||bk==18;
            if((a&255)==(b&255)&&((type2a&&type2b)||(ak==bk&&
               (ak!=17||std::memcmp(w+7,wires+j*17+7,2)==0))))
                return fail("Duplicate active request key");
        }
        std::memcpy(records[i].tx,w,17);records[i].deadline_ns=deadline;
    }
    if(cancelled(s,sibling_cancel))return fail("Cancelled before active exchange");
    if(!boot_ok(s))return fail("Boot identity changed/read failed");
    fd_set initial;FD_ZERO(&initial);FD_SET(s->fd,&initial);timespec immediate{};
    const int initial_ready=pselect(s->fd+1,&initial,nullptr,nullptr,&immediate,nullptr);
    if(initial_ready<0)return fail("Initial readiness check failed");
    if(FD_ISSET(s->fd,&initial)) {
        const ssize_t n=read(s->fd,buffer,sizeof(buffer));
        if(n>0){used=size_t(n);stats->bytes+=uint64_t(n);++stats->reads;}
        return fail("Input backlog/EOF before active exchange");
    }
    uint32_t done=0,loops=0,transient=0;
    uint64_t next=std::max(stats->begin_ns,s->last_finish+s->gap);
    while(done<count) {
        uint64_t t=now();
        if(!t||t>=deadline)return fail("Active exchange deadline exceeded; no retry");
        if(++loops>20000)return fail("Active bounded iteration limit");
        if(!boot_ok(s))return fail("Boot identity changed/read failed");
        fd_set readable;FD_ZERO(&readable);FD_SET(s->fd,&readable);FD_SET(s->cancel_fd,&readable);
        if(sibling_cancel>=0)FD_SET(sibling_cancel,&readable);
        const bool writable=sent<count&&sent-done<effective_window;
        const uint64_t wake=writable?std::min(deadline,next):deadline;
        // Boot checking may consume time after loop-entry t.
        // Keep the original absolute wake/deadline; never extend either one.
        const uint64_t before_wait=now();
        if(!before_wait||before_wait>=deadline)return fail("Active exchange deadline exceeded; no retry");
        const uint64_t wait=wake>before_wait?wake-before_wait:0;
        timespec timeout{time_t(wait/1000000000),long(wait%1000000000)};
        ++stats->waits;
        const int ready=pselect(std::max({s->fd,s->cancel_fd,sibling_cancel})+1,&readable,nullptr,nullptr,&timeout,nullptr);
        if(ready<0){if(errno==EINTR&&++transient<=32)continue;return fail("Active pselect failed");}
        if(FD_ISSET(s->cancel_fd,&readable)||
           (sibling_cancel>=0&&FD_ISSET(sibling_cancel,&readable)))return fail("Cancelled active exchange");
        if(FD_ISSET(s->fd,&readable)) {
            const uint64_t read_start=now();++stats->reads;
            if(used==sizeof(buffer))return fail("Active receive buffer overflow");
            const ssize_t n=read(s->fd,buffer+used,sizeof(buffer)-used);const uint64_t received=now();
            if(n<0){if((errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR)&&++transient<=32)continue;return fail("Active read failed");}
            if(!n)return fail("Active serial EOF");
            stats->bytes+=uint64_t(n);used+=size_t(n);
            if(received>=deadline||!received)return fail("Late active receive rejected");
            while(used>=17) {
                if(!framing(buffer))return fail("Malformed active AT frame; no resynchronization");
                uint32_t target=count;
                for(uint32_t i=0;i<sent;++i)if(!records[i].received&&matches(records[i].tx,buffer)){target=i;break;}
                if(target==count)return fail("Unmatched/duplicate/fault/mode/status reply");
                auto &r=records[target];if(received<r.finish_ns)return fail("Noncausal active reply");
                std::memcpy(r.rx,buffer,17);r.read_start_ns=read_start;r.received_ns=received;r.received=17;++done;
                if((id(r.tx)>>24)==4&&((id(buffer)>>16)&63))s->poisoned=true;
                used-=17;std::memmove(buffer,buffer+17,used);
            }
            // No buffered prefix can belong to an unsent request. Otherwise a
            // later LF could make pre-request bytes appear to be a causal ACK.
            if(used&&done==sent)return fail("Trailing partial active frame");
        }
        const char *prefix_failure=nullptr;
        if(prefix&&!publish_split_prefix(prefix,records,stats,deadline,prefix_failure))
            return fail(prefix_failure);
        t=now();
        if(!t||t>=deadline)
            return fail(sent==count ? "Active reply deadline exceeded; all requests already written"
                                    : "Active deadline reached before remaining writes");
        if(sent<count&&sent-done<effective_window&&t>=next) {
            // Check immediately before EVERY individual write, not merely at phase entry.
            if(cancelled(s,sibling_cancel))return fail("Cancelled before active write");
            if(!boot_ok(s))return fail("Boot identity changed before active write");
            if(!fd_ok(s))return fail("Active FD binding changed before write");
            if(s->poisoned)return fail("STOP reported a fault; further active commands prohibited");
            auto &r=records[sent];r.start_ns=now();
            if(r.start_ns>=deadline)return fail("Active deadline reached at write boundary");
            const ssize_t n=write(s->fd,r.tx,17);r.finish_ns=now();s->last_finish=r.finish_ns;++stats->writes;
            r.written=n>0?uint32_t(n):0;++sent;
            if(n!=17)return fail("Partial/failed active write; no retransmission");
            if(!r.finish_ns||r.finish_ns>=deadline)return fail("Late active write");
            next=r.finish_ns+s->gap;
        }
    }
    stats->end_ns=now();return 0;
}

extern "C" int sda_exchange(void *handle,const unsigned char *wires,uint32_t count,int send_only,
        uint64_t deadline,SDRecord *records,SDStats *stats,char *error,uint32_t size) {
    return exchange_owned(handle,wires,count,send_only,deadline,records,stats,error,size);
}

extern "C" int sda_emergency_stop(void *handle,uint64_t deadline,SDRecord *records,
        SDStats *stats,SDStopResult *result,char *error,uint32_t size) {
    auto *s=static_cast<Session*>(handle);if(!s||!records||!stats||!result||!error||!size)return -1;
    std::memset(records,0,sizeof(*records)*6);std::memset(stats,0,sizeof(*stats));
    std::memset(result,0,sizeof(*result));stats->begin_ns=now();
    std::unique_lock<std::mutex> lock(s->mutex,std::try_to_lock);
    auto finish=[&](int rc,const char *why){stats->end_ns=now();std::snprintf(error,size,"%s",why);return rc;};
    if(!lock.owns_lock())return finish(-1,"Concurrent emergency request: cancel active call and join owner first");
    if(s->paired_phase.load())return finish(-1,"Cancel and join native pair before emergency STOP");
    s->poisoned=true;result->ambiguous_mask=s->ambiguous;
    if(!fd_ok(s)||deadline<=stats->begin_ns||deadline-stats->begin_ns<20000000||
       deadline-stats->begin_ns>500000000)return finish(-1,"Invalid emergency FD/budget");
    // Boot/cancel do NOT block STOP. Preserve bounded raw backlog; never use it as an ACK.
    // Each actuator gets its own slice even if the preceding actuator times out.
    // The caller's default aggregate budget is 250ms (about 41.7ms per ID),
    // not an active-control deadline. A late earlier ID remains rejected: a
    // larger explicit budget cannot retroactively validate an expired slice.
    unsigned char buffer[4096];size_t used=0;
    const uint64_t budget=deadline-stats->begin_ns;
    for(int axis=0;axis<6;++axis) {
        auto &r=records[axis];stop_wire(s->first+axis,r.tx);
        const uint64_t axis_deadline=stats->begin_ns+budget*uint64_t(axis+1)/6;
        r.deadline_ns=axis_deadline;
        if(used){retain(stats,buffer,used);used=0;}
        // At most one nonblocking pre-write read: continuous telemetry cannot prevent STOP.
        if(fd_ok(s)) {
            const ssize_t n=read(s->fd,buffer,sizeof(buffer));++stats->reads;
            if(n>0){stats->bytes+=uint64_t(n);retain(stats,buffer,size_t(n));}
        }
        uint64_t t=now(),next=s->last_finish+s->gap;
        if(t<next&&next<axis_deadline) {
            const uint64_t wait=next-t;timespec ts{time_t(wait/1000000000),long(wait%1000000000)};
            ++stats->waits;pselect(0,nullptr,nullptr,nullptr,&ts,nullptr);
        }
        t=now();
        if(!fd_ok(s)||!t||t>=axis_deadline||t<s->last_finish+s->gap)continue;
        r.start_ns=t;result->attempted_mask|=1U<<axis;
        const ssize_t n=write(s->fd,r.tx,17);r.finish_ns=now();s->last_finish=r.finish_ns;++stats->writes;
        r.written=n>0?uint32_t(n):0;
        if(n!=17||r.finish_ns>=axis_deadline)continue; // still attempt the other IDs.
        uint32_t loops=0;
        while(now()<axis_deadline&&++loops<=1000) {
            t=now();if(t>=axis_deadline)break;
            const uint64_t wait=axis_deadline-t;timespec ts{time_t(wait/1000000000),long(wait%1000000000)};
            fd_set f;FD_ZERO(&f);FD_SET(s->fd,&f);++stats->waits;
            const int ready=pselect(s->fd+1,&f,nullptr,nullptr,&ts,nullptr);
            if(ready<0){if(errno==EINTR)continue;break;}if(!ready)break;
            if(!fd_ok(s))break;
            const uint64_t read_start=now();++stats->reads;
            const ssize_t got=read(s->fd,buffer+used,sizeof(buffer)-used);const uint64_t received=now();
            if(got<=0)break;
            stats->bytes+=uint64_t(got);used+=size_t(got);
            if(received>=axis_deadline){retain(stats,buffer,used);used=0;break;}
            // STOP-only recovery can scan malformed backlog, but retains every skipped byte.
            while(used>=17) {
                if(framing(buffer)) {
                    if(!r.received&&matches(r.tx,buffer)&&received>=r.finish_ns) {
                        std::memcpy(r.rx,buffer,17);r.received=17;r.read_start_ns=read_start;r.received_ns=received;
                        result->fault[axis]=(id(buffer)>>16)&63;
                        if(!(result->ambiguous_mask&(1U<<axis)))result->confirmed_mask|=1U<<axis;
                    } else retain(stats,buffer,17);
                    used-=17;std::memmove(buffer,buffer+17,used);
                } else {retain(stats,buffer,1);--used;std::memmove(buffer,buffer+1,used);}
            }
            if(r.received)break;
        }
    }
    if(used)retain(stats,buffer,used);
    // A later emergency call is allowed to attempt STOP again, but Type2 has
    // no transaction number. Preserve any reply still outstanding from this
    // cleanup so the next call cannot mistake it for the new STOP's reply.
    // Include partial writes: their downstream interpretation is not proven.
    for(int axis=0;axis<6;++axis)
        if(records[axis].written&&!records[axis].received)s->ambiguous|=1U<<axis;
    const bool complete=result->attempted_mask==63&&result->confirmed_mask==63;
    return finish(complete?0:1,complete?"All six STOP mode-zero replies observed":"STOP best effort incomplete/ambiguous; physical cutoff may be required");
}

namespace {
// Real, caller-owned serial sessions are borrowed, never duplicated/opened.
// Each phase uses fixed slots. Only the coordinator allocates at construction;
// the ordinary individual-frame sda_exchange parser owns every transaction.
struct PairJob {
    std::array<unsigned char,17*12> wires{};
    uint32_t count=0;
    uint64_t deadline=0;
    std::array<SDRecord,12> records{};
    SDStats stats{};
    std::array<char,256> error{};
    int status=-2;
};
struct PhasePair {
    Session *sessions[2];
    int cancellation[2]{-1,-1};
    // Optional caller-owned private pipe. It carries completion hints, never
    // readiness or telemetry; caller retains both ends until waiters and the
    // coordinator have joined. ABI 1 result layouts/exchange stay unchanged.
    int notification[2]{-1,-1};
    std::array<uint64_t,3> notification_binding[2]{};
    std::thread owners[2];
    std::mutex operation, mutex;
    std::condition_variable release, completion;
    std::atomic<bool> cancelled{false};
    std::atomic<uint64_t> cancel_requested_ns{0};
    bool closing=false;
    uint64_t generation=0;
    uint32_t finished=0;
    PairJob jobs[2];
    SDPairStats phase{};
    SDPairOwnerSettings settings[2]{};
    uint64_t requested_cpu_mask=0, requested_timer_slack_ns=0;
    bool settings_job=false, restore_settings=false;
    explicit PhasePair(Session *front,Session *rear):sessions{front,rear}{}
    bool notification_ok() const {
        for(unsigned i=0;i<2;++i) {
            std::array<uint64_t,3> actual{};
            const int flags=fcntl(notification[i],F_GETFL);
            if(notification[i]<0||notification[i]>=FD_SETSIZE||
               flags<0||!(flags&O_NONBLOCK)||!binding(notification[i],actual)||
               actual!=notification_binding[i])return false;
        }
        return true;
    }
    void notify_completion() const {
        // A full nonblocking pipe already holds a hint. Never block serial
        // owners or change their original status/deadlines for this hint.
        if(!notification_ok())return;
        const unsigned char byte=1;
        const ssize_t ignored=write(notification[1],&byte,1);(void)ignored;
    }
    void owner_settings(unsigned index) {
        auto &row=settings[index];row.status=-1;
#ifdef __APPLE__
        // macOS socket regressions can exercise serial ownership; Linux-only
        // placement controls must fail closed instead of pretending to apply.
        (void)requested_cpu_mask;(void)requested_timer_slack_ns;
#else
        cpu_set_t original;CPU_ZERO(&original);
        if(sched_getaffinity(0,sizeof(original),&original))return;
        uint64_t original_mask=0;
        for(int cpu=0;cpu<CPU_SETSIZE;++cpu)if(CPU_ISSET(cpu,&original)) {
            if(cpu>=64)return;
            original_mask|=uint64_t(1)<<cpu;
        }
        const long original_slack=prctl(PR_GET_TIMERSLACK);
        if(original_slack<=0)return;
        row.native_tid=uint64_t(syscall(SYS_gettid));
        if(!row.configured) {
            row.original_cpu_mask=original_mask;row.original_timer_slack_ns=uint64_t(original_slack);
        }
        const uint64_t target_mask=restore_settings?row.original_cpu_mask:requested_cpu_mask;
        const uint64_t target_slack=restore_settings?row.original_timer_slack_ns:requested_timer_slack_ns;
        if(!target_mask||!target_slack)return;
        cpu_set_t target;CPU_ZERO(&target);
        for(int cpu=0;cpu<64;++cpu)if(target_mask&(uint64_t(1)<<cpu))CPU_SET(cpu,&target);
        // Mark a partial application for restoration even if the second syscall
        // or its readback fails. Never treat a successful call as readback proof.
        row.configured=1;row.restored=0;
        if(sched_setaffinity(0,sizeof(target),&target)||
           prctl(PR_SET_TIMERSLACK,static_cast<unsigned long>(target_slack)))return;
        cpu_set_t actual;CPU_ZERO(&actual);
        if(sched_getaffinity(0,sizeof(actual),&actual))return;
        row.cpu_mask=0;
        for(int cpu=0;cpu<CPU_SETSIZE;++cpu)if(CPU_ISSET(cpu,&actual)) {
            if(cpu>=64)return;
            row.cpu_mask|=uint64_t(1)<<cpu;
        }
        const long slack=prctl(PR_GET_TIMERSLACK);
        if(slack<=0)return;
        row.timer_slack_ns=uint64_t(slack);
        if(row.cpu_mask!=target_mask||row.timer_slack_ns!=target_slack)return;
        row.status=0;row.restored=restore_settings?1:0;
#endif
    }
    void cancel() {
        if(!cancelled.exchange(true)) {
            cancel_requested_ns.store(now());
            const unsigned char byte=1;
            // Nonblocking private pipe: one sticky byte cancels both owners.
            // A failed notification remains observable through the atomic flag
            // at phase boundaries; their original hard deadlines still apply.
            const ssize_t ignored=write(cancellation[1],&byte,1);(void)ignored;
        }
        release.notify_all();completion.notify_all();
    }
    void owner(unsigned index) {
        uint64_t seen=0;
        for(;;) {
            std::unique_lock<std::mutex> lock(mutex);
            release.wait(lock,[&]{return closing||generation!=seen;});
            if(closing)return;
            seen=generation;
            PairJob &job=jobs[index];phase.owner_started_ns[index]=now();
            lock.unlock();
            if(settings_job) {
                owner_settings(index);job.status=settings[index].status;
            } else {
                job.status=exchange_owned(sessions[index],job.wires.data(),job.count,0,
                    job.deadline,job.records.data(),&job.stats,job.error.data(),
                    uint32_t(job.error.size()),this,cancellation[0]);
                if(job.status||sessions[index]->poisoned)cancel();
            }
            lock.lock();
            phase.owner_finished_ns[index]=now();phase.owner_status[index]=job.status;
            ++finished;
            // A normal first owner cannot satisfy the coordinator's two-owner
            // predicate. Preserve all settings/error/cancel/closing wakeups.
            if(finished==2||settings_job||job.status||cancelled.load()||closing)
                completion.notify_all();
        }
    }
};
const char *pair_batch_problem(Session *session,const unsigned char *wires,
        uint32_t count,uint64_t deadline,uint64_t stamp) {
    if(session->poisoned)return "Session poisoned; paired active retry prohibited";
    if(!wires||count<1||count>12||!stamp||deadline<=stamp||deadline-stamp>250000000)
        return "Invalid paired active batch/absolute deadline";
    if(!fd_ok(session))return "Paired active FD binding/nonblocking mode changed";
    if(!boot_ok(session))return "Paired active boot identity changed/read failed";
    if(cancelled(session))return "Cancelled before paired active publication";
    for(uint32_t i=0;i<count;++i) {
        const unsigned char *wire=wires+i*17;
        if(!valid_request(wire,session))return "Disallowed/noncanonical/out-of-bounds paired command";
        for(uint32_t j=0;j<i;++j) {
            const uint32_t a=id(wire),b=id(wires+j*17),ak=a>>24,bk=b>>24;
            const bool type2a=ak==1||ak==3||ak==4||ak==18;
            const bool type2b=bk==1||bk==3||bk==4||bk==18;
            if((a&255)==(b&255)&&((type2a&&type2b)||(ak==bk&&
               (ak!=17||std::memcmp(wire+7,wires+j*17+7,2)==0))))
                return "Duplicate paired active request key";
        }
    }
    return nullptr;
}
}

extern "C" uint32_t sda_pair_abi() {return 1;}
extern "C" uint32_t sda_pair_notification_abi() {return 1;}
extern "C" void *sda_pair_create(void *front_handle,void *rear_handle,
        char *error,uint32_t size) {
    auto bad=[&](const char *why)->void* {
        if(error&&size)std::snprintf(error,size,"%s",why);
        return nullptr;
    };
    auto *front=static_cast<Session*>(front_handle),*rear=static_cast<Session*>(rear_handle);
    if(!error||!size||!front||!rear||front==rear||front->first!=1||rear->first!=7)
        return bad("Distinct front/rear sessions required for native pair");
    std::scoped_lock<std::mutex,std::mutex> held(front->mutex,rear->mutex);
    if(front->pair_owner||rear->pair_owner||front->poisoned||rear->poisoned||
       !fd_ok(front)||!fd_ok(rear)||front->binding==rear->binding||
       !boot_ok(front)||!boot_ok(rear)||std::memcmp(front->boot,rear->boot,36)||
       cancelled(front)||cancelled(rear))return bad("Invalid/owned/poisoned paired session binding");
    PhasePair *pair=new(std::nothrow) PhasePair(front,rear);
    if(!pair)return bad("Native pair allocation failed");
    if(pipe(pair->cancellation)||pair->cancellation[0]>=FD_SETSIZE||pair->cancellation[1]>=FD_SETSIZE) {
        for(int fd:pair->cancellation)if(fd>=0)close(fd);
        delete pair;
        return bad("Native pair cancellation pipe failed");
    }
    for(int fd:pair->cancellation) {
        const int flags=fcntl(fd,F_GETFL);
        if(flags<0||fcntl(fd,F_SETFL,flags|O_NONBLOCK)<0||fcntl(fd,F_SETFD,FD_CLOEXEC)<0) {
            close(pair->cancellation[0]);close(pair->cancellation[1]);delete pair;
            return bad("Native pair cancellation pipe flags failed");
        }
    }
    front->pair_owner=pair;rear->pair_owner=pair;
    try {
        pair->owners[0]=std::thread(&PhasePair::owner,pair,0);
        pair->owners[1]=std::thread(&PhasePair::owner,pair,1);
    } catch(...) {
        {std::lock_guard<std::mutex> lock(pair->mutex);pair->closing=true;}
        pair->release.notify_all();
        for(auto &thread:pair->owners)if(thread.joinable())thread.join();
        front->pair_owner=nullptr;rear->pair_owner=nullptr;
        close(pair->cancellation[0]);close(pair->cancellation[1]);delete pair;
        return bad("Native pair owner creation failed");
    }
    return pair;
}
extern "C" void sda_pair_cancel(void *handle) {
    auto *pair=static_cast<PhasePair*>(handle);if(pair)pair->cancel();
}
extern "C" int sda_pair_set_notification(void *handle,int read_fd,int write_fd,
        char *error,uint32_t size) {
    auto *pair=static_cast<PhasePair*>(handle);
    if(!pair||!error||!size)return -1;
    error[0]=0;
    auto fail=[&](const char *message) {
        std::snprintf(error,size,"%s",message);return -1;
    };
    std::unique_lock<std::mutex> operation(pair->operation,std::try_to_lock);
    if(!operation.owns_lock())return fail("Native pair busy during notification binding");
    if(pair->generation||pair->notification[0]>=0||pair->cancelled.load()||
       read_fd<0||write_fd<0||read_fd==write_fd||read_fd>=FD_SETSIZE||write_fd>=FD_SETSIZE)
        return fail("Fresh distinct private notification pipe required");
    for(int fd:{read_fd,write_fd}) {
        for(auto *session:pair->sessions)
            if(fd==session->fd||fd==session->cancel_fd||fd==session->boot_fd)
                return fail("Notification pipe aliases a session descriptor");
        if(fd==pair->cancellation[0]||fd==pair->cancellation[1])
            return fail("Notification pipe aliases pair cancellation");
    }
    struct stat reader{},writer{};
    const int read_flags=fcntl(read_fd,F_GETFL),write_flags=fcntl(write_fd,F_GETFL);
    if(read_flags<0||write_flags<0||!(read_flags&O_NONBLOCK)||!(write_flags&O_NONBLOCK)||
       (read_flags&O_ACCMODE)!=O_RDONLY||(write_flags&O_ACCMODE)!=O_WRONLY||
       fstat(read_fd,&reader)||fstat(write_fd,&writer)||!S_ISFIFO(reader.st_mode)||
       !S_ISFIFO(writer.st_mode))
        return fail("Bound nonblocking private notification pipe required");
    // macOS gives the two ends distinct inode values. Confirm an empty,
    // connected pipe with a private setup byte rather than assuming Linux's
    // same-inode representation. This runs before any phase is released.
    unsigned char probe=0;
    if(read(read_fd,&probe,1)!=-1||(errno!=EAGAIN&&errno!=EWOULDBLOCK))
        return fail("Fresh empty private notification pipe required");
    const unsigned char marker=0x73;
    if(write(write_fd,&marker,1)!=1||read(read_fd,&probe,1)!=1||probe!=marker)
        return fail("Connected private notification pipe required");
    pair->notification[0]=read_fd;pair->notification[1]=write_fd;
    if(!binding(read_fd,pair->notification_binding[0])||
       !binding(write_fd,pair->notification_binding[1])) {
        pair->notification[0]=-1;pair->notification[1]=-1;
        return fail("Notification pipe binding failed");
    }
    return 0;
}
// Optional hint wait: 0 is a bounded tick, 1 is a pipe notification. A
// notification never certifies Future publication. Cancellation has priority,
// even when both descriptors were already readable before this call.
extern "C" int sda_pair_wait_completion(void *handle,uint64_t tick_ns,
        uint64_t deadline_ns,uint64_t *actual_ns,char *error,uint32_t size) {
    auto *pair=static_cast<PhasePair*>(handle);
    if(actual_ns)*actual_ns=0;
    if(!pair||!actual_ns||!error||!size)return -1;
    error[0]=0;
    auto fail=[&](const char *message) {
        std::snprintf(error,size,"%s",message);return -1;
    };
    const uint64_t begin=now();
    if(!begin||!tick_ns||!deadline_ns||tick_ns>deadline_ns||
       deadline_ns<=begin||deadline_ns-begin>1000000000ULL||
       !pair->notification_ok())return fail("Invalid bounded native completion wait");
    const int cancel_fds[3]={pair->cancellation[0],pair->sessions[0]->cancel_fd,
                             pair->sessions[1]->cancel_fd};
    for(int fd:cancel_fds)
        if(fd<0||fd>=FD_SETSIZE||fcntl(fd,F_GETFL)<0)
            return fail("Invalid native completion cancellation descriptor");
    auto finish=[&](int kind) {
        fd_set readable;FD_ZERO(&readable);int highest=-1;
        for(int fd:cancel_fds){FD_SET(fd,&readable);highest=std::max(highest,fd);}
        timespec zero_timeout{};
        const int ready=pselect(highest+1,&readable,nullptr,nullptr,&zero_timeout,nullptr);
        if(ready<0)return fail("Native completion final cancellation check failed");
        if(pair->cancelled.load())return fail("Cancelled native completion wait");
        for(int fd:cancel_fds)
            if(FD_ISSET(fd,&readable))return fail("Cancelled native completion wait");
        const uint64_t actual=now();
        if(!actual||actual>=deadline_ns)return fail("Native completion hard deadline expired");
        *actual_ns=actual;return kind;
    };
    uint32_t interrupts=0;
    while(true) {
        if(pair->cancelled.load())return fail("Cancelled native completion wait");
        if(!pair->notification_ok())return fail("Native completion pipe binding changed");
        fd_set readable;FD_ZERO(&readable);
        int highest=pair->notification[0];FD_SET(pair->notification[0],&readable);
        for(int fd:cancel_fds){FD_SET(fd,&readable);highest=std::max(highest,fd);}
        const uint64_t stamp=now();
        if(!stamp||stamp>=deadline_ns)return fail("Native completion hard deadline expired");
        const uint64_t remaining=stamp<tick_ns?tick_ns-stamp:0;
        timespec timeout{time_t(remaining/1000000000ULL),long(remaining%1000000000ULL)};
        const int ready=pselect(highest+1,&readable,nullptr,nullptr,&timeout,nullptr);
        if(ready<0) {
            if(errno==EINTR&&++interrupts<=32)continue;
            return fail("Native completion wait interrupted/failed");
        }
        if(pair->cancelled.load())return fail("Cancelled native completion wait");
        for(int fd:cancel_fds)
            if(FD_ISSET(fd,&readable))return fail("Cancelled native completion wait");
        const uint64_t actual=now();
        if(!actual||actual>=deadline_ns)return fail("Native completion hard deadline expired");
        if(!pair->notification_ok())return fail("Native completion pipe binding changed");
        if(FD_ISSET(pair->notification[0],&readable)) {
            unsigned char hints[256];
            const ssize_t count=read(pair->notification[0],hints,sizeof(hints));
            if(count==0)return fail("Native completion pipe reached EOF");
            if(count<0) {
                if(errno==EINTR&&++interrupts<=32)continue;
                if(errno==EAGAIN||errno==EWOULDBLOCK)continue;
                return fail("Native completion pipe read failed");
            }
            return finish(1);
        }
        if(actual>=tick_ns)return finish(0);
    }
}
extern "C" int sda_pair_exchange(void *handle,
        const unsigned char *front_wires,uint32_t front_count,uint64_t front_deadline,
        const unsigned char *rear_wires,uint32_t rear_count,uint64_t rear_deadline,
        SDRecord *front_records,SDStats *front_stats,char *front_error,
        SDRecord *rear_records,SDStats *rear_stats,char *rear_error,
        uint32_t error_size,SDPairStats *phase) {
    auto *pair=static_cast<PhasePair*>(handle);
    if(!pair||!front_records||!rear_records||!front_stats||!rear_stats||
       !front_error||!rear_error||!error_size||!phase)return -1;
    std::unique_lock<std::mutex> operation(pair->operation,std::try_to_lock);
    if(!operation.owns_lock())return -1;
    *front_stats={};*rear_stats={};*phase={};front_error[0]=0;rear_error[0]=0;
    const unsigned char *wires[2]={front_wires,rear_wires};
    const uint32_t counts[2]={front_count,rear_count};
    const uint64_t deadlines[2]={front_deadline,rear_deadline};
    char *errors[2]={front_error,rear_error};
    SDRecord *records[2]={front_records,rear_records};
    SDStats *stats[2]={front_stats,rear_stats};
    const uint64_t submitted=now();
    {
        std::scoped_lock<std::mutex,std::mutex> sessions(pair->sessions[0]->mutex,pair->sessions[1]->mutex);
        const char *problem[2]={nullptr,nullptr};
        for(unsigned i=0;i<2;++i)problem[i]=pair_batch_problem(pair->sessions[i],wires[i],counts[i],deadlines[i],submitted);
        if(pair->cancelled.load()||problem[0]||problem[1]) {
            phase->submitted_ns=submitted;phase->validated_ns=now();
            for(unsigned i=0;i<2;++i) {
                pair->sessions[i]->poisoned=true;
                stats[i]->begin_ns=submitted;stats[i]->end_ns=now();
                phase->owner_status[i]=-1;
                std::snprintf(errors[i],error_size,"%s",problem[i]?problem[i]:
                    "Native pair not released because the phase was cancelled/invalid");
            }
            pair->cancel();phase->cancel_requested_ns=pair->cancel_requested_ns.load();return -1;
        }
        for(auto *session:pair->sessions)session->paired_phase.store(true);
    }
    std::unique_lock<std::mutex> lock(pair->mutex);
    pair->phase={};pair->phase.submitted_ns=submitted;pair->phase.validated_ns=now();
    pair->settings_job=false;
    pair->finished=0;
    for(unsigned i=0;i<2;++i) {
        pair->jobs[i]=PairJob{};pair->jobs[i].count=counts[i];pair->jobs[i].deadline=deadlines[i];
        std::memcpy(pair->jobs[i].wires.data(),wires[i],17*counts[i]);
        pair->phase.owner_status[i]=-2;
    }
    pair->phase.generation=++pair->generation;pair->phase.released_ns=now();
    pair->release.notify_all();
    // Individual exchanges keep their absolute <=250 ms deadlines. On the
    // first error, the private pipe wakes the other pselect without waiting
    // for Python to collect its Future. No motor command is retried here.
    const uint64_t maximum=std::max(front_deadline,rear_deadline);
    while(pair->finished!=2) {
        const uint64_t stamp=now();
        if(!stamp||stamp>=maximum)pair->cancel();
        const uint64_t remaining=stamp&&stamp<maximum?maximum-stamp:1000000;
        pair->completion.wait_for(lock,std::chrono::nanoseconds(remaining));
    }
    int status=0;
    for(unsigned i=0;i<2;++i) {
        auto &job=pair->jobs[i];std::memcpy(records[i],job.records.data(),counts[i]*sizeof(SDRecord));
        *stats[i]=job.stats;std::snprintf(errors[i],error_size,"%s",job.error.data());
        if(job.status)status=-1;
        pair->sessions[i]->paired_phase.store(false);
    }
    pair->phase.cancel_requested_ns=pair->cancel_requested_ns.load();*phase=pair->phase;
    pair->notify_completion();
    return status;
}
extern "C" int sda_pair_owner_settings(void *handle,uint64_t cpu_mask,
        uint64_t timer_slack_ns,int restore,SDPairOwnerSettings *settings,
        char *error,uint32_t size) {
    auto *pair=static_cast<PhasePair*>(handle);
    if(!pair||!settings||!error||!size||(restore!=0&&restore!=1)||
       (!restore&&(!cpu_mask||timer_slack_ns!=1000)))return -1;
    std::unique_lock<std::mutex> operation(pair->operation);
    std::unique_lock<std::mutex> lock(pair->mutex);
    pair->settings_job=true;pair->restore_settings=restore!=0;
    pair->requested_cpu_mask=cpu_mask;pair->requested_timer_slack_ns=timer_slack_ns;
    pair->finished=0;pair->phase={};++pair->generation;pair->release.notify_all();
    while(pair->finished!=2)pair->completion.wait(lock);
    settings[0]=pair->settings[0];settings[1]=pair->settings[1];
    const bool success=!settings[0].status&&!settings[1].status;
    std::snprintf(error,size,"%s",success?"":"Native pair owner placement/slack unsupported or not restored by readback");
    return success?0:-1;
}
extern "C" void sda_pair_destroy(void *handle) {
    auto *pair=static_cast<PhasePair*>(handle);if(!pair)return;
    pair->cancel();
    std::unique_lock<std::mutex> operation(pair->operation);
    {std::lock_guard<std::mutex> lock(pair->mutex);pair->closing=true;}
    pair->release.notify_all();
    for(auto &thread:pair->owners)if(thread.joinable())thread.join();
    {std::scoped_lock<std::mutex,std::mutex> held(pair->sessions[0]->mutex,pair->sessions[1]->mutex);
        for(auto *session:pair->sessions){session->paired_phase.store(false);session->pair_owner=nullptr;}}
    close(pair->cancellation[0]);close(pair->cancellation[1]);operation.unlock();delete pair;
}

// Optional PRIVATE candidate ABI, never automatically selected by production.
extern "C" uint32_t sda_split_feedback_voltage_abi() {
    return sizeof(SDRecord)==88&&sizeof(SDSplitMeta)==32&&
           offsetof(SDSplitMeta,received_mask)==24?2:0;
}
extern "C" void *sda_split_create(void *handle,uint64_t generation,int expected_cancel,int hint_read,int hint_write,
                                  char *error,uint32_t size) {
    auto bad=[&](const char *why)->void* {if(error&&size)std::snprintf(error,size,"%s",why);return nullptr;};
    auto *s=static_cast<Session*>(handle);
    if(!s||!generation||!error||!size||hint_read==hint_write)return bad("Invalid split ownership");
    if(expected_cancel!=s->cancel_fd)return bad("Split cancellation FD differs from original owner");
    std::unique_lock<std::mutex> lock(s->mutex,std::try_to_lock);
    if(!lock.owns_lock()||s->pair_owner||s->paired_phase.load()||s->poisoned)
        return bad("Split session already borrowed/busy/poisoned");
    if(s->gap!=900000||s->window!=3||!fd_ok(s)||!boot_ok(s))
        return bad("Split candidate requires canonical gap900/window3/current binding");
    SplitPipeBinding pipes[2];
    if(!split_pipe_binding(hint_read,O_RDONLY,pipes[0])||
       !split_pipe_binding(hint_write,O_WRONLY,pipes[1])||
       hint_read==s->fd||hint_write==s->fd||hint_read==s->cancel_fd||hint_write==s->cancel_fd)
        return bad("Invalid split nonblocking private pipe endpoints");
    auto *p=new(std::nothrow) SplitState{};
    if(!p)return bad("Split state allocation failed");
    p->session=s;p->generation=generation;p->hint_read=hint_read;p->hint_write=hint_write;
    p->pipe[0]=pipes[0];p->pipe[1]=pipes[1];s->pair_owner=p;s->paired_phase.store(true);
    return p;
}
static int split_exchange_configured(void *handle,const unsigned char *wires,uint32_t count,
        uint64_t deadline,SDRecord *records,SDStats *stats,char *error,uint32_t size,bool live_expected) {
    auto *p=static_cast<SplitState*>(handle);
    if(!p||!wires||!records||!stats||!error||!size)return -1;
    if(count!=7){std::snprintf(error,size,"Split output extent requires count7");return -1;}
    if(p->live_hold!=live_expected){std::snprintf(error,size,"Wrong private STOP/LIVE split capability");return -1;}
    std::unique_lock<std::mutex> lock(p->lifecycle,std::try_to_lock);
    if(!lock.owns_lock()){std::snprintf(error,size,"Concurrent split owner use");return -1;}
    {std::lock_guard<std::mutex> publication(p->publication);
     if(p->started){std::snprintf(error,size,"Split generation already used");return -1;}
     p->started=true;}
    std::memset(records,0,sizeof(*records)*7);std::memset(stats,0,sizeof(*stats));
    auto finish=[&](int status,const char *failure) {
        std::lock_guard<std::mutex> publication(p->publication);
        p->terminal=true;p->final_status=status;p->meta.final_native_ns=now();
        if(failure)std::snprintf(error,size,"%s",failure);
        if(status)std::snprintf(p->final_error,sizeof(p->final_error),"%s",error);
        return status;
    };
    const uint64_t before=now();
    if(count!=7||!before||deadline<=before||deadline-before>20000000||!split_pipe_ok(p))
        return finish(-1,"Split requires seven requests / original at-most-20ms deadline");
    for(unsigned i=0;i<6;++i) {
        if(p->live_hold) {
            const auto *w=wires+i*17;
            const double q=double(be16(w+7))*25.14/65535.-12.57;
            if(std::memcmp(w,p->held_wires.data()+i*17,17)||!valid_request(w,p->session)||
               (id(w)>>24)!=1||q<p->live_lower[i]||q>p->live_upper[i])
                return finish(-1,"LIVE split requires exact bounded previous held Type1 bytes");
        } else {
            unsigned char expected[17];stop_wire(p->session->first+int(i),expected);
            if(std::memcmp(wires+i*17,expected,17))return finish(-1,"Split prefix requires six ordered canonical STOP requests");
        }
    }
    const unsigned char *v=wires+6*17;const uint32_t can=id(v);
    if(!valid_request(v,p->session)||(can>>24)!=17||v[7]!=0x1c||v[8]!=0x70)
        return finish(-1,"Split seventh request must be this bus rotating voltage 0x701c");
    const int status=exchange_owned(p->session,wires,count,0,deadline,records,stats,error,size,p,-1,p);
    if(!status&&p->session->poisoned)return finish(-1,"Split STOP fault poisoned session");
    if(!status&&cancelled(p->session))return finish(-1,"Cancelled at split full completion");
    if(!status&&(!fd_ok(p->session)||!boot_ok(p->session)||!split_pipe_ok(p)))
        return finish(-1,"Split full completion binding/boot changed");
    if(!status&&now()>=deadline)return finish(-1,"Split full completion original deadline exceeded");
    return finish(status,nullptr);
}
extern "C" int sda_split_exchange(void *handle,const unsigned char *wires,uint32_t count,
        uint64_t deadline,SDRecord *records,SDStats *stats,char *error,uint32_t size) {
    return split_exchange_configured(handle,wires,count,deadline,records,stats,error,size,false);
}

extern "C" int sda_split_take_prefix(void *handle,uint64_t generation,
        SDRecord *records,SDStats *stats,SDSplitMeta *meta,char *error,uint32_t size) {
    auto *p=static_cast<SplitState*>(handle);
    if(!p||!records||!stats||!meta||!error||!size)return -1;
    std::lock_guard<std::mutex> lock(p->publication);
    if(generation!=p->generation||!split_pipe_ok(p)) {
        std::snprintf(error,size,"Split prefix generation/pipe binding changed");return -1;
    }
    if(p->ready) {
        std::memcpy(records,p->prefix.data(),sizeof(SDRecord)*6);
        std::memcpy(stats,&p->stats,sizeof(*stats));std::memcpy(meta,&p->meta,sizeof(*meta));
    }
    if(p->terminal&&p->final_status) {
        std::snprintf(error,size,"%s",p->final_error);return -1;
    }
    return p->ready?1:0;
}
extern "C" int sda_split_destroy(void *handle,char *error,uint32_t size) {
    auto *p=static_cast<SplitState*>(handle);if(!p||!error||!size)return -1;
    std::unique_lock<std::mutex> life(p->lifecycle,std::try_to_lock);
    if(!life.owns_lock()){std::snprintf(error,size,"Join split owner before destroy");return -1;}
    {std::lock_guard<std::mutex> publication(p->publication);
     std::lock_guard<std::mutex> session(p->session->mutex);
     if(p->session->pair_owner!=p){std::snprintf(error,size,"Split session owner changed");return -1;}
     p->session->paired_phase.store(false);p->session->pair_owner=nullptr;}
    life.unlock();delete p;return 0;
}


// Separately selected PRIVATE LIVE transport prototype. This capability is NOT
// a profile approval, mechanical qualification, or permission to send targets.
// It only repeats the exact caller-bound previously validated held Type1 bytes.
struct SDLiveHoldLimits { double lower[6],upper[6]; };
extern "C" uint32_t sda_live_split_feedback_voltage_abi() {
    return sizeof(SDLiveHoldLimits)==96&&offsetof(SDLiveHoldLimits,upper)==48&&
           sizeof(SDRecord)==88&&sizeof(SDSplitMeta)==32?1:0;
}
extern "C" void *sda_live_split_create(void *handle,uint64_t generation,int expected_cancel,
        int hint_read,int hint_write,const unsigned char *held,uint32_t count,
        const SDLiveHoldLimits *limits,char *error,uint32_t size) {
    auto bad=[&](const char *why)->void* {if(error&&size)std::snprintf(error,size,"%s",why);return nullptr;};
    auto *s=static_cast<Session*>(handle);
    if(!s||!held||count!=6||!limits||!error||!size)return bad("LIVE split requires exactly six bound held Type1 wires and target limits");
    // These narrow numeric caps add no runtime authorization. Future caller
    // admission must bind the same original profile/current branch/PD checks.
    constexpr double max_span=2.*3.14159265358979323846/180.;
    for(unsigned i=0;i<6;++i) {
        const auto *w=held+i*17;const double lo=limits->lower[i],hi=limits->upper[i];
        if(!std::isfinite(lo)||!std::isfinite(hi)||lo>=hi||hi-lo>max_span||
           lo<s->limits.lower[i]||hi>s->limits.upper[i]||
           !valid_request(w,s)||(id(w)>>24)!=1||(id(w)&255)!=uint32_t(s->first)+i)
            return bad("LIVE held Type1 framing/order/current target bounds rejected");
        const double q=double(be16(w+7))*25.14/65535.-12.57;
        const double kp=double(be16(w+11))*500./65535.;
        const double kd=double(be16(w+13))*5./65535.;
        if(q<lo||q>hi||kp>3.||kd>.15)
            return bad("LIVE held Type1 target/gain outside bounded caps");
    }
    auto *p=static_cast<SplitState*>(sda_split_create(handle,generation,expected_cancel,
                                                    hint_read,hint_write,error,size));
    if(!p)return nullptr;
    p->live_hold=true;std::memcpy(p->held_wires.data(),held,102);
    for(unsigned i=0;i<6;++i){p->live_lower[i]=limits->lower[i];p->live_upper[i]=limits->upper[i];}
    return p;
}
extern "C" int sda_live_split_exchange(void *handle,const unsigned char *wires,uint32_t count,
        uint64_t deadline,SDRecord *records,SDStats *stats,char *error,uint32_t size) {
    return split_exchange_configured(handle,wires,count,deadline,records,stats,error,size,true);
}
extern "C" int sda_live_split_take_prefix(void *handle,uint64_t generation,
        SDRecord *records,SDStats *stats,SDSplitMeta *meta,char *error,uint32_t size) {
    auto *p=static_cast<SplitState*>(handle);
    if(!p||!p->live_hold){if(error&&size)std::snprintf(error,size,"LIVE prefix requires LIVE capability");return -1;}
    return sda_split_take_prefix(handle,generation,records,stats,meta,error,size);
}
extern "C" int sda_live_split_destroy(void *handle,char *error,uint32_t size) {
    auto *p=static_cast<SplitState*>(handle);
    if(!p||!p->live_hold){if(error&&size)std::snprintf(error,size,"LIVE destroy requires LIVE capability");return -1;}
    return sda_split_destroy(handle,error,size);
}
