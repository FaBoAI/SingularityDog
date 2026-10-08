// Mask-bound four-bus active exchange. The ordinary ABI/path and the exact-three
// STOP extension are included unchanged; this file adds new symbols only.
#ifdef FOUR_BUS_SUBSET_STOP_SOURCE
#include FOUR_BUS_SUBSET_STOP_SOURCE
#else
#include "../four_bus_diagnostic/subset_stop.cpp"
#endif

namespace {
// Static checks only: no I/O, clock, cancel or boot access. Caller holds s->mutex.
const char *subset_problem(Session *s,uint32_t group_mask,const unsigned char *wires,uint32_t count) {
    if(group_mask!=0x07U&&group_mask!=0x38U)return "Explicit three-axis half-envelope mask required";
    if(!wires||count<1||count>6)return "Subset active batch must hold 1..6 wires";
    uint32_t type1=0;
    for(uint32_t i=0;i<count;++i) {
        const unsigned char *w=wires+i*17;
        if(!framing(w))return "Noncanonical subset active frame";
        const uint32_t c=id(w),k=c>>24;
        const int slot=int(c&255)-s->first;
        if(slot<0||slot>=6||!(group_mask&(1U<<slot)))return "Motor ID outside this port's three-axis mask";
        if(k!=0&&k!=1&&k!=3&&k!=4&&k!=17&&k!=18)return "Disallowed subset active kind";
        if(k==1)++type1;
        if(!valid_request(w,s))return "Disallowed/noncanonical/out-of-bounds subset active command";
        for(uint32_t j=0;j<i;++j) {
            const uint32_t b=id(wires+j*17),bk=b>>24;
            const bool type2a=k==1||k==3||k==4||k==18,type2b=bk==1||bk==3||bk==4||bk==18;
            if((c&255)==(b&255)&&((type2a&&type2b)||(k==bk&&
               (k!=17||std::memcmp(w+7,wires+j*17+7,2)==0))))
                return "Duplicate subset active request key";
        }
    }
    if(type1) {
        if(type1!=count)return "Type1 batch must contain only Type1";
        if(count!=1&&count!=3)return "Type1 batch must be one axis or the exact masked three";
        if(count==3) {
            uint32_t next=0;
            for(int slot=0;slot<6;++slot) {
                if(!(group_mask&(1U<<slot)))continue;
                if((id(wires+next*17)&255)!=uint32_t(s->first+slot))
                    return "Type1 triple must cover the mask in ascending ID order";
                ++next;
            }
        }
    }
    return nullptr;
}
}

extern "C" uint32_t sda_subset_active_abi() {
    return sizeof(SDRecord)==88&&sizeof(SDStopResult)==36 ? 1U : 0U;
}

// 0 valid; -1 rejected. A batch violation poisons the session under its mutex.
extern "C" int sda_subset_validate(void *handle,uint32_t group_mask,const unsigned char *wires,
        uint32_t count,char *error,uint32_t size) {
    auto *s=static_cast<Session*>(handle);if(!s||!error||!size)return -1;
    error[0]=0;
    std::unique_lock<std::mutex> lock(s->mutex,std::try_to_lock);
    if(!lock.owns_lock()){std::snprintf(error,size,"Concurrent native active session use");return -1;}
    if(s->paired_phase.load()){std::snprintf(error,size,"Session borrowed by native pair phase");return -1;}
    if(s->poisoned){std::snprintf(error,size,"Session poisoned; active retry prohibited");return -1;}
    if(const char *why=subset_problem(s,group_mask,wires,count)) {
        s->poisoned=true;std::snprintf(error,size,"%s",why);return -1;
    }
    return 0;
}

// Same trailing parameters/semantics as sda_exchange. Validation runs first on a
// private copy; nothing is written on failure. The validated copy, not the
// caller's buffer, is handed to the ordinary exchange_owned (which re-takes the
// mutex and re-checks poison/cancel/boot/FD/caps/deadline).
extern "C" int sda_subset_exchange(void *handle,uint32_t group_mask,const unsigned char *wires,
        uint32_t count,int send_only,uint64_t deadline,SDRecord *records,SDStats *stats,
        char *error,uint32_t size) {
    auto *s=static_cast<Session*>(handle);
    if(!s||!records||!stats||!error||!size)return -1;
    unsigned char copy[17*6];
    {
        std::memset(stats,0,sizeof(*stats));stats->begin_ns=now();
        auto reject=[&](const char *why,bool poison) {
            if(poison)s->poisoned=true;
            stats->end_ns=now();std::snprintf(error,size,"%s",why);return -1;
        };
        std::unique_lock<std::mutex> lock(s->mutex,std::try_to_lock);
        if(!lock.owns_lock()){stats->end_ns=now();std::snprintf(error,size,"Concurrent native active session use");return -1;}
        if(s->paired_phase.load())return reject("Session borrowed by native pair phase",false);
        if(s->poisoned)return reject("Session poisoned; active retry prohibited",false);
        if(!wires||count<1||count>6)return reject("Subset active batch must hold 1..6 wires",true);
        std::memcpy(copy,wires,size_t(count)*17);
        if(const char *why=subset_problem(s,group_mask,copy,count))return reject(why,true);
    }
    return exchange_owned(handle,copy,count,send_only,deadline,records,stats,error,size);
}

extern "C" uint32_t sda_subset_exchange_at_abi() {
    return sizeof(SDRecord)==88&&sizeof(SDStopResult)==36 ? 1U : 0U;
}

// Pre-armed variant of sda_subset_exchange: identical validation first (nothing
// written; violation poisons), then now<not_before<deadline with not_before at
// most 5 ms ahead, then a native wait (session cancel fd watched, short final
// spin) under the session mutex, then the unchanged exchange_owned path with the
// original deadline. Every failure after validation also poisons the session
// and writes nothing; concurrent/borrowed/already-poisoned calls only refuse.
extern "C" int sda_subset_exchange_at(void *handle,uint32_t group_mask,const unsigned char *wires,
        uint32_t count,int send_only,uint64_t not_before,uint64_t deadline,SDRecord *records,
        SDStats *stats,char *error,uint32_t size) {
    auto *s=static_cast<Session*>(handle);
    if(!s||!records||!stats||!error||!size)return -1;
    unsigned char copy[17*6];
    {
        std::memset(stats,0,sizeof(*stats));stats->begin_ns=now();
        auto reject=[&](const char *why,bool poison) {
            if(poison)s->poisoned=true;
            stats->end_ns=now();std::snprintf(error,size,"%s",why);return -1;
        };
        std::unique_lock<std::mutex> lock(s->mutex,std::try_to_lock);
        if(!lock.owns_lock()){stats->end_ns=now();std::snprintf(error,size,"Concurrent native active session use");return -1;}
        if(s->paired_phase.load())return reject("Session borrowed by native pair phase",false);
        if(s->poisoned)return reject("Session poisoned; active retry prohibited",false);
        if(!wires||count<1||count>6)return reject("Subset active batch must hold 1..6 wires",true);
        std::memcpy(copy,wires,size_t(count)*17);
        if(const char *why=subset_problem(s,group_mask,copy,count))return reject(why,true);
        const uint64_t start=now();
        if(!start||send_only!=0||not_before<=start||not_before>=deadline||
           not_before-start>5000000ULL||deadline-start>250000000ULL)
            return reject("Pre-armed exchange requires now<not_before<deadline, not_before<=now+5ms, deadline<=now+250ms",true);
        if(!fd_ok(s))return reject("Active FD binding changed before pre-armed wait",true);
        if(!boot_ok(s))return reject("Boot identity changed before pre-armed wait",true);
        auto check_cancel=[&](uint64_t wait_ns) {
            fd_set readable;FD_ZERO(&readable);FD_SET(s->cancel_fd,&readable);
            timespec timeout{time_t(wait_ns/1000000000ULL),long(wait_ns%1000000000ULL)};
            const int ready=pselect(s->cancel_fd+1,&readable,nullptr,nullptr,&timeout,nullptr);
            if(ready<0)return errno==EINTR?2:-1;
            return FD_ISSET(s->cancel_fd,&readable)?1:0;
        };
        const uint64_t spin_ns=200000ULL;
        uint32_t interrupts=0;
        while(true) {
            const int before=check_cancel(0);
            if(before==1)return reject("Cancelled before pre-armed release",true);
            if(before==-1)return reject("Pre-armed cancellation check failed",true);
            if(before==2){if(++interrupts>32)return reject("Pre-armed wait interrupted too often",true);continue;}
            const uint64_t current=now();
            if(!current)return reject("Pre-armed monotonic clock failed",true);
            if(current>=not_before)break;
            const uint64_t remaining=not_before-current;
            if(remaining>spin_ns) {
                const int ready=check_cancel(remaining-spin_ns);
                if(ready==1)return reject("Cancelled during pre-armed wait",true);
                if(ready==-1)return reject("Pre-armed wait failed",true);
                if(ready==2&&++interrupts>32)return reject("Pre-armed wait interrupted too often",true);
                continue;
            }
            while(true) {
                const uint64_t spinning=now();
                if(!spinning)return reject("Pre-armed monotonic clock failed",true);
                if(spinning>=not_before)break;
            }
            break;
        }
        const int after=check_cancel(0);
        if(after==1)return reject("Cancelled after pre-armed release",true);
        if(after==-1||after==2)return reject("Pre-armed final cancellation check failed",true);
        const uint64_t released=now();
        if(!released||released<not_before)return reject("Pre-armed wait returned before not_before",true);
        if(released>=deadline)return reject("Pre-armed release reached the exchange deadline",true);
        if(!fd_ok(s))return reject("Active FD binding changed during pre-armed wait",true);
        if(!boot_ok(s))return reject("Boot identity changed during pre-armed wait",true);
    }
    return exchange_owned(handle,copy,count,send_only,deadline,records,stats,error,size);
}
