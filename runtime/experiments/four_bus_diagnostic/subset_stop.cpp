// Independent optional exact-three STOP cleanup. The ordinary ABI/path is included unchanged.
#ifdef FOUR_BUS_ORDINARY_SOURCE
#include FOUR_BUS_ORDINARY_SOURCE
#else
#include "../native_active_transport/transport.cpp"
#endif

extern "C" uint32_t sda_emergency_stop_subset_abi() {
    return sizeof(SDRecord)==88 && sizeof(SDStopResult)==36 ? 1U : 0U;
}

extern "C" int sda_emergency_stop_subset(void *handle,uint32_t group_mask,uint64_t deadline,SDRecord *records,
        SDStats *stats,SDStopResult *result,char *error,uint32_t size) {
    auto *s=static_cast<Session*>(handle);if(!s||!records||!stats||!result||!error||!size)return -1;
    if(group_mask!=0x07U&&group_mask!=0x38U) {
        std::snprintf(error,size,"Explicit three-axis half-envelope mask required");return -1;
    }
    std::memset(records,0,sizeof(*records)*6);std::memset(stats,0,sizeof(*stats));
    std::memset(result,0,sizeof(*result));stats->begin_ns=now();
    std::unique_lock<std::mutex> lock(s->mutex,std::try_to_lock);
    auto finish=[&](int rc,const char *why){stats->end_ns=now();std::snprintf(error,size,"%s",why);return rc;};
    if(!lock.owns_lock())return finish(-1,"Concurrent emergency request: cancel active call and join owner first");
    if(s->paired_phase.load())return finish(-1,"Cancel and join native pair before emergency STOP");
    s->poisoned=true;result->ambiguous_mask=s->ambiguous&group_mask;
    if(!fd_ok(s)||deadline<=stats->begin_ns||deadline-stats->begin_ns<20000000||
       deadline-stats->begin_ns>500000000)return finish(-1,"Invalid emergency FD/budget");
    // Boot/cancel do NOT block STOP. Preserve bounded raw backlog; never use it as an ACK.
    // Each actuator gets its own slice even if the preceding actuator times out.
    // The caller's default aggregate budget is 250ms (about 83.3ms per selected ID),
    // not an active-control deadline. A late earlier ID remains rejected: a
    // larger explicit budget cannot retroactively validate an expired slice.
    unsigned char buffer[4096];size_t used=0;
    const uint64_t budget=deadline-stats->begin_ns;
    uint32_t selected_index=0;
    for(int axis=0;axis<6;++axis) {
        if(!(group_mask&(1U<<axis)))continue;
        auto &r=records[axis];stop_wire(s->first+axis,r.tx);
        const uint64_t axis_deadline=stats->begin_ns+budget*uint64_t(++selected_index)/3;
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
        if((group_mask&(1U<<axis))&&records[axis].written&&!records[axis].received)s->ambiguous|=1U<<axis;
    result->ambiguous_mask=s->ambiguous&group_mask;
    const bool complete=result->attempted_mask==group_mask&&result->confirmed_mask==group_mask;
    return finish(complete?0:1,complete?"All selected three STOP mode-zero replies observed":"STOP best effort incomplete/ambiguous; physical cutoff may be required");
}
