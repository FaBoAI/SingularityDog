// Bounded diagnostic transport. No enable, configuration, or motion opcode.
// One caller owns the fd; ctypes releases the GIL for this entire transaction.
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/select.h>
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
};
uint32_t sd_abi() { return 1; }
}
namespace {
uint64_t now() {
    timespec t{};
#ifdef __APPLE__
    // CPython uses mach_absolute_time on macOS; CLOCK_MONOTONIC includes sleep.
    if (clock_gettime(CLOCK_UPTIME_RAW, &t)) return 0;
#else
    if (clock_gettime(CLOCK_MONOTONIC, &t)) return 0;
#endif
    return uint64_t(t.tv_sec)*1000000000ULL + uint64_t(t.tv_nsec);
}
uint32_t id(const unsigned char *w) {
    return (uint32_t(w[2])<<24 | uint32_t(w[3])<<16 |
            uint32_t(w[4])<<8 | uint32_t(w[5])) >> 3;
}
bool framing(const unsigned char *w) {
    return w[0]=='A' && w[1]=='T' && (w[5]&7)==4 && w[6]==8 && w[15]==13 && w[16]==10;
}
bool zero(const unsigned char *p, int n) {
    for (int i=0;i<n;++i) if(p[i]) return false;
    return true;
}
bool request(const unsigned char *w, int first, bool stop) {
    if(!framing(w)) return false;
    const uint32_t c=id(w), kind=c>>24, mid=c&255;
    if(mid<uint32_t(first)||mid>=uint32_t(first+6)||((c>>8)&65535)!=0xfd) return false;
    if(kind==0 || (kind==4 && stop)) return zero(w+7,8);
    const uint32_t index=w[7] | uint32_t(w[8])<<8;
    // Voltage is read-only and admitted only for the explicitly disabled
    // STOP-proxy cadence comparison. No other parameter or active command is
    // added to this diagnostic transport.
    return kind==17 && (index==0x7019 || index==0x701b ||
                        (stop && index==0x701c)) && zero(w+9,6);
}
bool match(const unsigned char *tx, const unsigned char *rx) {
    const uint32_t a=id(tx), b=id(rx), k=a>>24;
    if(((b>>8)&255)!=(a&255)) return false;
    if(k==0) return b==(uint32_t((a&255)<<8)|0xfe);
    if(k==4) return b==((2U<<24)|((a&255)<<8)|0xfd) &&
        !(rx[7]==0 && rx[8]==0xc4 && rx[9]==0x56); // reject version-shaped Type2
    if(b!=((17U<<24)|((a&255)<<8)|0xfd) || tx[7]!=rx[7] || tx[8]!=rx[8] || !zero(rx+9,2)) return false;
    const uint32_t bits=rx[11]|uint32_t(rx[12])<<8|uint32_t(rx[13])<<16|uint32_t(rx[14])<<24;
    float value; std::memcpy(&value,&bits,4);
    return std::isfinite(value);
}
}

extern "C" uint64_t sd_now_ns() { return now(); }

// Pure scheduling diagnostic: no serial/device fd, command, or motor state.
// ctypes.CDLL releases the GIL while this bounded wait runs. The caller's
// monotonic deadline is never replaced by a planned/backdated timestamp.
extern "C" int sd_wait_until(int cancel_fd, uint64_t deadline_ns, uint32_t spin_us,
        uint64_t *woke_ns, char *error, uint32_t error_size) {
    if(woke_ns) *woke_ns=0;
    if(!woke_ns||!error||error_size==0) return -1;
    auto fail=[&](const char *message) {
        std::snprintf(error,error_size,"%s",message);return -1;
    };
    const uint64_t start=now();
    if(!start||!deadline_ns||cancel_fd<0||cancel_fd>=FD_SETSIZE||
       fcntl(cancel_fd,F_GETFL)<0||(spin_us!=200&&spin_us!=500)||
       (deadline_ns>start&&deadline_ns-start>1000000000ULL)||
       (start>deadline_ns&&start-deadline_ns>1000000000ULL))
        return fail("Invalid bounded diagnostic wait arguments");
    auto check_cancel=[&](uint64_t wait_ns) {
        fd_set readable;FD_ZERO(&readable);FD_SET(cancel_fd,&readable);
        timespec timeout{time_t(wait_ns/1000000000ULL),long(wait_ns%1000000000ULL)};
        const int ready=pselect(cancel_fd+1,&readable,nullptr,nullptr,&timeout,nullptr);
        if(ready<0) return errno==EINTR?2:-1;
        return FD_ISSET(cancel_fd,&readable)?1:0;
    };
    uint32_t interrupts=0;
    const uint64_t spin_ns=uint64_t(spin_us)*1000ULL;
    while(true) {
        // Also covers an already-past deadline: cancellation wins over success.
        const int before=check_cancel(0);
        if(before==1) return fail("Cancelled before diagnostic wait completion");
        if(before==-1) return fail("Diagnostic wait cancellation check failed");
        if(before==2) {
            if(++interrupts>32) return fail("Diagnostic wait interrupted too often");
            continue;
        }
        const uint64_t current=now();
        if(!current) return fail("Diagnostic wait monotonic clock failed");
        if(current>=deadline_ns) break;
        const uint64_t remaining=deadline_ns-current;
        if(remaining>spin_ns) {
            // pselect listens for cancellation throughout the sleeping phase.
            const int ready=check_cancel(remaining-spin_ns);
            if(ready==1) return fail("Cancelled during diagnostic wait");
            if(ready==-1) return fail("Diagnostic wait pselect failed");
            if(ready==2 && ++interrupts>32)
                return fail("Diagnostic wait interrupted too often");
            continue;
        }
        // Only the final caller-selected 200/500 us can spin.
        while(true) {
            const uint64_t spinning_now=now();
            if(!spinning_now) return fail("Diagnostic wait monotonic clock failed");
            if(spinning_now>=deadline_ns) break;
        }
        break;
    }
    // Check again after the bounded spin (and after an already-past target).
    const int after=check_cancel(0);
    if(after==1) return fail("Cancelled after diagnostic wait");
    if(after==-1||after==2) return fail("Diagnostic wait final cancellation check failed");
    const uint64_t actual=now();
    if(!actual||actual<deadline_ns) return fail("Diagnostic wait returned before deadline");
    *woke_ns=actual;
    return 0;
}

extern "C" int sd_exchange(int fd, int cancel_fd, int boot_fd, const char *boot,
        const unsigned char *wires, uint32_t count, int first, int allow_stop,
        uint64_t gap_ns, uint32_t window, uint64_t deadline, uint64_t last_finish,
        SDRecord *records, SDStats *stats, char *error, uint32_t error_size) {
    if(!records||!stats||!error||!error_size) return -1;
    std::memset(stats,0,sizeof(*stats));
    unsigned char buffer[4096]; size_t used=0;
    auto fail=[&](const char *message) {
        if(used) {stats->rejected_size=uint32_t(used);std::memcpy(stats->rejected,buffer,used);}
        stats->end_ns=now(); std::snprintf(error,error_size,"%s",message); return -1;
    };
    stats->begin_ns=now();
    if(fd<0||fd>=FD_SETSIZE||cancel_fd<0||cancel_fd>=FD_SETSIZE||fd==cancel_fd||
       !wires||count==0||count>12||(first!=1&&first!=7)||window==0||window>3||
       (allow_stop!=0&&allow_stop!=1)||gap_ns<600000||gap_ns>5000000||
       !stats->begin_ns||last_finish>stats->begin_ns||deadline<=stats->begin_ns||deadline-stats->begin_ns>250000000||
       !(fcntl(fd,F_GETFL)&O_NONBLOCK)||fcntl(fd,F_GETFL)<0||fcntl(cancel_fd,F_GETFL)<0)
        return fail("Invalid bounded diagnostic exchange arguments");
    if(boot_fd>=0 && (!boot||std::strlen(boot)!=36)) return fail("Invalid boot binding");
    std::memset(records,0,sizeof(*records)*count);
    bool identity_only=true;
    for(uint32_t i=0;i<count;++i) {
        const auto *w=wires+17*i;
        if(!request(w,first,allow_stop)) return fail("Disallowed/noncanonical diagnostic command");
        identity_only=identity_only && (id(w)>>24)==0;
        for(uint32_t j=0;j<i;++j) if(std::memcmp(w,wires+17*j,17)==0) return fail("Duplicate request key");
        std::memcpy(records[i].tx,w,17);
        records[i].deadline_ns=deadline;
    }
    // UID preflight keeps one request in flight; periodic telemetry keeps its configured window.
    const uint32_t effective_window=identity_only?1:window;
    // No flush: pre-existing bytes make response attribution ambiguous.
    fd_set initial; FD_ZERO(&initial); FD_SET(fd,&initial); FD_SET(cancel_fd,&initial);
    timespec immediate{};
    const int initial_ready=pselect(std::max(fd,cancel_fd)+1,&initial,nullptr,nullptr,&immediate,nullptr);
    if(initial_ready<0) return fail("Initial readiness check failed");
    if(FD_ISSET(cancel_fd,&initial)) return fail("Cancelled before exchange");
    ssize_t n=-1;
    if(FD_ISSET(fd,&initial)) {
        n=read(fd,buffer,sizeof(buffer));
        if(n>0) used=size_t(n);
        return fail("Input backlog/EOF before exchange");
    }
    uint32_t sent=0, done=0, transient=0, loops=0;
    auto deadline_fail=[&]() {
        char message[256];
        if(sent==count && done<count) {
            char pending[64]{}; size_t length=0;
            for(uint32_t i=0;i<sent;++i) if(!records[i].received)
                length+=size_t(std::snprintf(pending+length,sizeof(pending)-length,
                    "%s%u",length?",":"",id(records[i].tx)&255));
            std::snprintf(message,sizeof(message),
                "Response deadline exceeded after all writes (%u/%u replies); missing motor IDs: %s; no retry",
                done,count,pending);
        } else if(sent<count) {
            std::snprintf(message,sizeof(message),
                "Diagnostic deadline exceeded before all writes (%u/%u written, %u/%u replies); no retry",
                sent,count,done,count);
        } else {
            std::snprintf(message,sizeof(message),"Diagnostic exchange deadline exceeded after all replies; no retry");
        }
        return fail(message);
    };
    uint64_t next_send=std::max(stats->begin_ns,last_finish+gap_ns);
    while(done<count) {
        uint64_t t=now();
        if(!t||t>=deadline) return deadline_fail();
        if(++loops>20000) return fail("Bounded iteration limit");
        if(boot_fd>=0) {
            char b[80]; const ssize_t got=pread(boot_fd,b,sizeof(b),0);
            if(got<36||got>38||std::memcmp(b,boot,36)!=0||
               (got>36&&b[36]!='\n')||(got>37&&b[37]!='\n')) return fail("Boot identity changed/read failed");
        }
        fd_set readable; FD_ZERO(&readable);FD_SET(fd,&readable);FD_SET(cancel_fd,&readable);
        // pselect keeps sub-millisecond deadlines; poll(int milliseconds) would round them.
        const bool writable=sent<count && sent-done<effective_window;
        uint64_t wake=writable?std::min(deadline,next_send):deadline;
        uint64_t wait=wake>t?wake-t:0;
        timespec timeout{time_t(wait/1000000000),long(wait%1000000000)};
        ++stats->waits;
        const int ready=pselect(std::max(fd,cancel_fd)+1,&readable,nullptr,nullptr,&timeout,nullptr);
        if(ready<0) {if(errno==EINTR&&++transient<=32) continue;return fail("pselect failed");}
        if(FD_ISSET(cancel_fd,&readable)) return fail("Cancelled");
        if(FD_ISSET(fd,&readable)) {
            const uint64_t read_start=now();
            if(used==sizeof(buffer)) return fail("Receive buffer overflow");
            ++stats->reads;
            n=read(fd,buffer+used,sizeof(buffer)-used);
            const uint64_t received=now();
            if(n<0) {if((errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR)&&++transient<=32) continue;return fail("read failed");}
            if(n==0) return fail("Serial EOF");
            stats->bytes+=uint64_t(n);used+=size_t(n);
            auto bad_rx=[&](const char *why) {
                stats->rejected_size=uint32_t(used);std::memcpy(stats->rejected,buffer,used);return fail(why);
            };
            if(received>=deadline||!received) return bad_rx("Late receive rejected");
            while(used>=17) {
                if(!framing(buffer)) return bad_rx("Malformed AT frame; no resynchronization");
                uint32_t target=count;
                for(uint32_t i=0;i<sent;++i) if(!records[i].received&&match(records[i].tx,buffer)) {target=i;break;}
                if(target==count) return bad_rx("Unmatched/duplicate/fault/mode/status reply");
                auto &r=records[target];
                if(received<r.finish_ns) return bad_rx("Noncausal reply");
                std::memcpy(r.rx,buffer,17);r.read_start_ns=read_start;r.received_ns=received;r.received=17;++done;
                used-=17;std::memmove(buffer,buffer+17,used);
            }
            if(used && done==count) return bad_rx("Trailing partial frame");
        }
        t=now();
        if(t>=deadline||!t) return deadline_fail();
        if(sent<count&&sent-done<effective_window&&t>=next_send) {
            auto &r=records[sent];r.start_ns=now();
            n=write(fd,r.tx,17);r.finish_ns=now();++stats->writes;
            r.written=n>0?uint32_t(n):0;
            if(n!=17) return fail("Partial/failed write; no retransmission");
            if(r.finish_ns>=deadline||!r.finish_ns) return fail("Late write");
            next_send=r.finish_ns+gap_ns;++sent;
        }
    }
    stats->end_ns=now();
    return 0;
}
