// Separate active RS05 transport. The diagnostic allowlist is not modified.
// No device opening/configuration, shell, network, allocation in the RX loop, or retry.
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <new>
#include <set>
#include <sys/select.h>
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
bool cancelled(Session *s) {
    fd_set f;FD_ZERO(&f);FD_SET(s->cancel_fd,&f);timespec t{};
    const int r=pselect(s->cancel_fd+1,&f,nullptr,nullptr,&t,nullptr);
    return r<0||FD_ISSET(s->cancel_fd,&f);
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
    {std::lock_guard<std::mutex> lock(owners_mutex);owners.erase(s->binding);}delete s;
}
extern "C" int sda_exchange(void *handle,const unsigned char *wires,uint32_t count,int send_only,
        uint64_t deadline,SDRecord *records,SDStats *stats,char *error,uint32_t size) {
    auto *s=static_cast<Session*>(handle);
    if(!s||!records||!stats||!error||!size)return -1;
    std::memset(stats,0,sizeof(*stats));stats->begin_ns=now();
    std::unique_lock<std::mutex> lock(s->mutex,std::try_to_lock);
    if(!lock.owns_lock()){std::snprintf(error,size,"Concurrent native active session use");return -1;}
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
    if(cancelled(s))return fail("Cancelled before active exchange");
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
        const bool writable=sent<count&&sent-done<effective_window;
        const uint64_t wake=writable?std::min(deadline,next):deadline;
        const uint64_t wait=wake>t?wake-t:0;
        timespec timeout{time_t(wait/1000000000),long(wait%1000000000)};
        ++stats->waits;
        const int ready=pselect(std::max(s->fd,s->cancel_fd)+1,&readable,nullptr,nullptr,&timeout,nullptr);
        if(ready<0){if(errno==EINTR&&++transient<=32)continue;return fail("Active pselect failed");}
        if(FD_ISSET(s->cancel_fd,&readable))return fail("Cancelled active exchange");
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
        t=now();if(!t||t>=deadline)return fail("Active deadline reached before write");
        if(sent<count&&sent-done<effective_window&&t>=next) {
            // Check immediately before EVERY individual write, not merely at phase entry.
            if(cancelled(s))return fail("Cancelled before active write");
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

extern "C" int sda_emergency_stop(void *handle,uint64_t deadline,SDRecord *records,
        SDStats *stats,SDStopResult *result,char *error,uint32_t size) {
    auto *s=static_cast<Session*>(handle);if(!s||!records||!stats||!result||!error||!size)return -1;
    std::memset(records,0,sizeof(*records)*6);std::memset(stats,0,sizeof(*stats));
    std::memset(result,0,sizeof(*result));stats->begin_ns=now();
    std::unique_lock<std::mutex> lock(s->mutex,std::try_to_lock);
    auto finish=[&](int rc,const char *why){stats->end_ns=now();std::snprintf(error,size,"%s",why);return rc;};
    if(!lock.owns_lock())return finish(-1,"Concurrent emergency request: cancel active call and join owner first");
    s->poisoned=true;result->ambiguous_mask=s->ambiguous;
    if(!fd_ok(s)||deadline<=stats->begin_ns||deadline-stats->begin_ns<20000000||
       deadline-stats->begin_ns>250000000)return finish(-1,"Invalid emergency FD/budget");
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
