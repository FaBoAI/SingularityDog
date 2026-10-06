// No device calls: clock and syscall callbacks below are fully synthetic.
#include "bounded_trace.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
using namespace proposed_trace;
static int mode=0,wall_calls=0,cpu_calls=0,call_count=0,seen_errno=0;
static uint64_t wall_clock() noexcept {
    ++wall_calls;errno=ERANGE;
    if(mode==8)return 0;
    if(mode==2)return wall_calls%2?100:90;
    return uint64_t(100+wall_calls*20);
}
static uint64_t cpu_clock() noexcept {++cpu_calls;errno=EFAULT;return 10+cpu_calls*3;}
int main(int argc,char** argv) {
    mode=argc>1?std::atoi(argv[1]):0;Recorder<2> trace;
    Context context{Kind::Select,3,6,5,1000,900,400};const Context prior=context;
    auto call=[]() noexcept -> int64_t {++call_count;seen_errno=errno;errno=mode==5?EINTR:(mode==6?EIO:EBUSY);return mode==5||mode==6?-1:17;};
    int64_t returned=0;const int count=mode==1?5:1;
    for(int i=0;i<count;++i) {
        errno=EBADF;
        returned=measure(mode==4?static_cast<Recorder<2>*>(nullptr):&trace,context,
                         mode==9?nullptr:wall_clock,mode==3?nullptr:cpu_clock,call);
    }
    const int final_errno=errno;const Event event=trace.events[0];
    const bool unchanged=context.kind==prior.kind&&context.loop==prior.loop&&context.sent==prior.sent&&context.completed==prior.completed&&context.original_deadline_ns==prior.original_deadline_ns&&context.requested_wake_ns==prior.requested_wake_ns&&context.requested_wait_ns==prior.requested_wait_ns;
    std::printf("{\"returned\":%lld,\"final_errno\":%d,\"expected_errno\":%d,\"seen_errno\":%d,\"entry_errno\":%d,\"call_count\":%d,\"wall_calls\":%d,\"cpu_calls\":%d,\"stored\":%zu,\"calls\":%llu,\"dropped\":%llu,\"overflow\":%s,\"invalid_clock\":%s,\"clock_valid\":%s,\"cpu_measured\":%s,\"cpu_begin_ns\":%llu,\"cpu_end_ns\":%llu,\"event_returned\":%lld,\"event_errno\":%d,\"context_unchanged\":%s,\"event_deadline_ns\":%llu,\"event_wait_ns\":%llu}\n",(long long)returned,final_errno,mode==5?EINTR:(mode==6?EIO:EBUSY),seen_errno,EBADF,call_count,wall_calls,cpu_calls,trace.stored,(unsigned long long)trace.calls,(unsigned long long)trace.dropped,trace.overflow?"true":"false",trace.invalid_clock?"true":"false",event.clock_valid?"true":"false",event.cpu_measured?"true":"false",(unsigned long long)event.cpu_begin_ns,(unsigned long long)event.cpu_end_ns,(long long)event.returned,event.saved_errno,unchanged?"true":"false",(unsigned long long)event.context.original_deadline_ns,(unsigned long long)event.context.requested_wait_ns);
}
