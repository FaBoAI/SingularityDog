// Experimental all-four native ownership. Includes the reviewed wire kernel
// unchanged. No device opening, model execution, fallback or output admission.
// Lifecycle wrapper registration is local to this composite DSO. The copied
// source bytes and ordinary external ABI remain unchanged.
#define sda_create sdg_base_create
#define sda_destroy sdg_base_destroy
#ifdef FOUR_BUS_ACTIVE_SOURCE
#include FOUR_BUS_ACTIVE_SOURCE
#else
#include "../four_bus_type1/subset_active.cpp"
#endif
#undef sda_create
#undef sda_destroy
#include <sys/socket.h>

extern "C" {
struct SDGPort {
    uint64_t owner_started_ns, owner_finished_ns;
    int32_t status;
    uint32_t count;
    SDRecord records[3];
    SDStats stats;
    char error[256];
};
struct SDGResult {
    uint64_t ticket, submitted_ns, validated_ns, released_ns, collected_ns, cancel_requested_ns;
    SDGPort ports[4];
};
}
namespace {
std::mutex group_session_registry_mutex;
std::set<Session*> group_session_registry;
struct NativeGroup {
    Session *sessions[4];
    const std::array<uint32_t,4> masks;
    const std::array<uint64_t,4> cpu_masks;
    const uint64_t timer_slack;
    const bool socket_fixture;
    int cancellation[2]{-1,-1};
    std::thread workers[4];
    std::mutex operation, mutex;
    std::condition_variable release, completion;
    std::atomic<bool> cancelled_flag{false};
    std::atomic<uint64_t> cancel_stamp{0};
    bool closing=false, settings_restore=false;
    uint32_t task=0, issued_bits=0, settings_done=0, hold_done=0, voltage_done=0, output_done=0;
    uint64_t job_generation=0, ticket=0, deadline=0, write_deadline=0;
    bool hold_taken=false, voltage_taken=false, ready_for_cycle=true, initial_done=false;
    std::array<std::array<unsigned char,51>,4> hold_wires{}, output_wires{};
    std::array<std::array<unsigned char,17>,4> voltage_wires{};
    SDGResult hold{}, voltage{}, output{};
    SDPairOwnerSettings settings[4]{};
    NativeGroup(Session **s,const uint32_t *m,const uint64_t *c,uint64_t slack,bool mock)
        :sessions{s[0],s[1],s[2],s[3]},masks{m[0],m[1],m[2],m[3]},
         cpu_masks{c[0],c[1],c[2],c[3]},timer_slack(slack),socket_fixture(mock) {}
    void cancel() {
        if(!cancelled_flag.exchange(true)) {
            cancel_stamp.store(now());
            const unsigned char byte=1;
            const ssize_t ignored=write(cancellation[1],&byte,1);(void)ignored;
        }
        release.notify_all();completion.notify_all();
    }
    void placement(unsigned i,bool restore) {
        auto &row=settings[i];row.status=-1;
        if(socket_fixture) {row.status=0;row.restored=restore?1:0;return;}
#ifdef __APPLE__
        (void)restore;
#else
        cpu_set_t before;CPU_ZERO(&before);
        if(sched_getaffinity(0,sizeof(before),&before))return;
        uint64_t original=0;
        for(int c=0;c<CPU_SETSIZE;++c)if(CPU_ISSET(c,&before)) {
            if(c>=64)return;
            original|=uint64_t(1)<<c;
        }
        const long slack=prctl(PR_GET_TIMERSLACK);if(slack<=0)return;
        row.native_tid=uint64_t(syscall(SYS_gettid));
        if(!row.configured) {row.original_cpu_mask=original;row.original_timer_slack_ns=uint64_t(slack);}
        const uint64_t target=restore?row.original_cpu_mask:cpu_masks[i];
        const uint64_t target_slack=restore?row.original_timer_slack_ns:timer_slack;
        if(!target||!target_slack)return;
        cpu_set_t desired;CPU_ZERO(&desired);
        for(int c=0;c<64;++c)if(target&(uint64_t(1)<<c))CPU_SET(c,&desired);
        row.configured=1;row.restored=0;
        if(sched_setaffinity(0,sizeof(desired),&desired)||
           prctl(PR_SET_TIMERSLACK,static_cast<unsigned long>(target_slack)))return;
        cpu_set_t actual;CPU_ZERO(&actual);if(sched_getaffinity(0,sizeof(actual),&actual))return;
        row.cpu_mask=0;
        for(int c=0;c<CPU_SETSIZE;++c)if(CPU_ISSET(c,&actual)) {
            if(c>=64)return;
            row.cpu_mask|=uint64_t(1)<<c;
        }
        const long readback=prctl(PR_GET_TIMERSLACK);if(readback<=0)return;
        row.timer_slack_ns=uint64_t(readback);
        if(row.cpu_mask!=target||row.timer_slack_ns!=target_slack)return;
        row.status=0;row.restored=restore?1:0;
#endif
    }
    void phase_exchange(unsigned i,SDGPort &row,const unsigned char *wire,uint32_t count,
                        uint64_t reply_cap,uint64_t write_cap=0) {
        row.count=count;row.owner_started_ns=now();
        if(cancelled_flag.load()) {
            row.status=-1;std::snprintf(row.error,sizeof(row.error),"Native group cancelled before owner exchange");
        } else {
            row.status=exchange_owned(sessions[i],wire,count,0,reply_cap,row.records,&row.stats,
                row.error,sizeof(row.error),this,cancellation[0],write_cap);
            if(row.status||sessions[i]->poisoned)cancel();
        }
        row.owner_finished_ns=now();
    }
    void owner(unsigned i) noexcept {
        uint64_t seen=0;
        for(;;) {
            std::unique_lock<std::mutex> lock(mutex);
            release.wait(lock,[&]{return closing||job_generation!=seen;});
            if(closing)return;
            seen=job_generation;const uint32_t selected=task;
            const bool restore=settings_restore;const uint64_t cap=deadline,write_cap=write_deadline;
            lock.unlock();
            try {
                if(selected==1) {
                    placement(i,restore);
                    lock.lock();++settings_done;completion.notify_all();
                } else if(selected==2) {
                    phase_exchange(i,hold.ports[i],hold_wires[i].data(),3,cap);
                    lock.lock();++hold_done;completion.notify_all();lock.unlock();
                    phase_exchange(i,voltage.ports[i],voltage_wires[i].data(),1,cap);
                    lock.lock();++voltage_done;completion.notify_all();
                } else if(selected==3||selected==4) {
                    phase_exchange(i,output.ports[i],output_wires[i].data(),3,cap,write_cap);
                    lock.lock();++output_done;completion.notify_all();
                }
            } catch(...) {
                cancel();
                if(!lock.owns_lock())lock.lock();
                if(selected==1) {settings[i].status=-1;++settings_done;}
                else if(selected==2) {
                    auto &h=hold.ports[i];auto &v=voltage.ports[i];
                    if(h.owner_finished_ns==0) {h.status=-1;h.owner_finished_ns=now();++hold_done;}
                    v.status=-1;v.owner_finished_ns=now();++voltage_done;
                } else if(selected==3||selected==4) {output.ports[i].status=-1;output.ports[i].owner_finished_ns=now();++output_done;}
                completion.notify_all();
            }
        }
    }
    void settings_phase(bool restore) {
        std::unique_lock<std::mutex> lock(mutex);
        settings_restore=restore;task=1;settings_done=0;++job_generation;release.notify_all();
        completion.wait(lock,[&]{return settings_done==4;});
    }
    bool placement_ok() const {
        for(auto &row:settings)if(row.status)return false;
        return true;
    }
    const char *session_problem() {
        if(cancelled_flag.load())return "Native group cancelled; active retry prohibited";
        for(unsigned i=0;i<4;++i) {
            auto *s=sessions[i];
            if(s->pair_owner!=this||!s->paired_phase.load())return "Native group ownership binding changed";
            if(s->poisoned)return "Native group session poisoned; active retry prohibited";
            if(!fd_ok(s))return "Native group FD binding changed";
            if(!boot_ok(s))return "Native group boot identity changed";
            if(cancelled(s,cancellation[0]))return "Native group cancellation observed";
            if(s->gap!=900000ULL||s->window!=3)return "Native group gap/window changed";
        }
        return nullptr;
    }
    void poison_all() {
        for(auto *s:sessions)s->poisoned=true;
        cancel();
    }
    void result_meta(SDGResult &r,uint64_t submitted,uint64_t validated,uint64_t released) {
        r={};r.ticket=ticket;r.submitted_ns=submitted;r.validated_ns=validated;r.released_ns=released;
    }
};
void group_error(char *out,uint32_t size,const char *why) {
    if(out&&size)std::snprintf(out,size,"%s",why);
}
const char *triple_problem(NativeGroup *g,unsigned index,const unsigned char *wires) {
    if(const char *problem=subset_problem(g->sessions[index],g->masks[index],wires,3))return problem;
    for(unsigned i=0;i<3;++i)if((id(wires+17*i)>>24)!=1)
        return "Native group triple must contain exact mask-ordered Type1 commands";
    return nullptr;
}
void lock_sessions(NativeGroup *g,std::unique_lock<std::mutex> *locks) {
    // Create fixes session order and forbids aliasing. Operations in this ABI
    // always use this order; ordinary calls use try_lock and cannot deadlock.
    for(unsigned i=0;i<4;++i)locks[i]=std::unique_lock<std::mutex>(g->sessions[i]->mutex);
}
int wait_result(NativeGroup *g,uint64_t ticket,SDGResult *out,bool is_voltage,char *error,uint32_t size) {
    if(!g||!out||!error||!size)return -1;
    *out={};error[0]=0;
    std::unique_lock<std::mutex> op(g->operation,std::try_to_lock);
    if(!op.owns_lock()){group_error(error,size,"Concurrent native group operation");return -1;}
    std::unique_lock<std::mutex> lock(g->mutex);
    bool &taken=is_voltage?g->voltage_taken:g->hold_taken;
    if(ticket!=g->ticket||!ticket||g->task!=2||taken||(is_voltage&&!g->hold_taken)) {
        group_error(error,size,"Exact current ticket and unconsumed ordered group phase required");return -1;
    }
    auto &done=is_voltage?g->voltage_done:g->hold_done;
    while(done!=4) {
        const uint64_t stamp=now();
        if(!stamp||stamp>=g->deadline)g->cancel();
        const uint64_t remaining=stamp<g->deadline?g->deadline-stamp:1000000ULL;
        g->completion.wait_for(lock,std::chrono::nanoseconds(remaining));
    }
    auto &result=is_voltage?g->voltage:g->hold;
    result.collected_ns=now();result.cancel_requested_ns=g->cancel_stamp.load();*out=result;taken=true;
    bool success=!g->cancelled_flag.load();
    for(auto &row:result.ports)if(row.status)success=false;
    if(!success)group_error(error,size,"Native group phase failed; cancel/join before existing bounded STOP cleanup");
    return success?0:-1;
}
}

extern "C" void *sda_create(int fd,int cancel_fd,int boot_fd,const char *boot,
        int first,const SDLimits *limits,uint64_t gap,uint32_t window,char *error,uint32_t size) {
    auto *s=static_cast<Session*>(sdg_base_create(fd,cancel_fd,boot_fd,boot,first,limits,gap,window,error,size));
    if(!s)return nullptr;
    try {
        std::lock_guard<std::mutex> lock(group_session_registry_mutex);
        group_session_registry.insert(s);
    } catch(...) {
        sdg_base_destroy(s);group_error(error,size,"Composite Session registration failed");return nullptr;
    }
    return s;
}
extern "C" void sda_destroy(void *handle) {
    auto *s=static_cast<Session*>(handle);if(!s)return;
    std::lock_guard<std::mutex> registry(group_session_registry_mutex);
    if(!group_session_registry.count(s))return;
    {std::lock_guard<std::mutex> lock(s->mutex);if(s->pair_owner)return;}
    // As with the original ABI, callers cannot destroy a Session concurrently
    // with its create/borrow/lifetime operations. Borrowed destruction refuses.
    group_session_registry.erase(s);sdg_base_destroy(s);
}

extern "C" uint32_t sdg_abi() {return sizeof(SDRecord)==88&&sizeof(SDStopResult)==36?1U:0U;}
extern "C" uint32_t sdg_result_size() {return sizeof(SDGResult);}
extern "C" uint32_t sdg_port_size() {return sizeof(SDGPort);}

// Borrow same-library handles only. The host retains descriptors and Sessions.
// All-zero placement is restricted to AF_UNIX socket fixtures; it cannot open
// or admit a serial deployment. Production placement must succeed by readback.
extern "C" void *sdg_create(void **handles,const uint32_t *masks,const uint64_t *cpu_masks,
        uint64_t timer_slack,SDPairOwnerSettings *settings,char *error,uint32_t size) {
    if(!handles||!masks||!cpu_masks||!settings||!error||!size)return nullptr;
    std::memset(settings,0,sizeof(*settings)*4);error[0]=0;
    Session *sessions[4]{};
    bool mock=timer_slack==0;
    // Reject foreign or stale pointers before dereferencing Session memory.
    std::unique_lock<std::mutex> registered(group_session_registry_mutex);
    uint32_t covered=0;
    for(unsigned i=0;i<4;++i) {
        sessions[i]=static_cast<Session*>(handles[i]);
        if(!group_session_registry.count(sessions[i])) {
            group_error(error,size,"Session handle did not originate from this composite DSO");return nullptr;
        }
        if(!sessions[i]||(masks[i]!=7&&masks[i]!=56)||
           (sessions[i]->first!=1&&sessions[i]->first!=7)) {
            group_error(error,size,"Canonical distinct four-port binding required");return nullptr;
        }
        const uint32_t ids=masks[i]<<(sessions[i]->first-1);
        if(covered&ids) {group_error(error,size,"Overlapping native group motor-ID masks");return nullptr;}
        covered|=ids;
        for(unsigned j=0;j<i;++j)if(sessions[i]==sessions[j]) {
            group_error(error,size,"Duplicate native Session handle");return nullptr;
        }
        if(mock?cpu_masks[i]!=0:cpu_masks[i]==0) {
            group_error(error,size,"Explicit four owner placement and timer slack required");return nullptr;
        }
    }
    if(covered!=0xfffU) {group_error(error,size,"Native group must cover exactly motor IDs 1..12");return nullptr;}
    std::unique_lock<std::mutex> locks[4];
    for(unsigned i=0;i<4;++i)locks[i]=std::unique_lock<std::mutex>(sessions[i]->mutex);
    for(unsigned i=0;i<4;++i) {
        auto *s=sessions[i];struct stat st{};
        sockaddr_storage address{};socklen_t address_size=sizeof(address);
        int socket_type=0;socklen_t type_size=sizeof(socket_type);
        if(s->pair_owner||s->paired_phase.load()||s->poisoned||!fd_ok(s)||!boot_ok(s)||
           cancelled(s)||s->gap!=900000ULL||s->window!=3||
           std::memcmp(s->boot,sessions[0]->boot,36)||
           (mock&&(fstat(s->fd,&st)||!S_ISSOCK(st.st_mode)||
             getsockname(s->fd,reinterpret_cast<sockaddr*>(&address),&address_size)||
             address.ss_family!=AF_UNIX||
             getsockopt(s->fd,SOL_SOCKET,SO_TYPE,&socket_type,&type_size)||socket_type!=SOCK_STREAM))) {
            group_error(error,size,"Invalid/owned/poisoned group Session, boot, FD, gap/window or socket-only fixture");return nullptr;
        }
        for(unsigned j=0;j<i;++j)if(s->binding==sessions[j]->binding) {
            group_error(error,size,"Aliased group descriptor binding");return nullptr;
        }
    }
    auto *g=new(std::nothrow) NativeGroup(sessions,masks,cpu_masks,timer_slack,mock);
    if(!g){group_error(error,size,"Native group allocation failed");return nullptr;}
    if(pipe(g->cancellation)||g->cancellation[0]>=FD_SETSIZE||g->cancellation[1]>=FD_SETSIZE) {
        for(int fd:g->cancellation)if(fd>=0)close(fd);
        delete g;group_error(error,size,"Native group cancellation pipe failed");return nullptr;
    }
    for(int fd:g->cancellation) {
        const int flags=fcntl(fd,F_GETFL);
        if(flags<0||fcntl(fd,F_SETFL,flags|O_NONBLOCK)||fcntl(fd,F_SETFD,FD_CLOEXEC)) {
            close(g->cancellation[0]);close(g->cancellation[1]);delete g;
            group_error(error,size,"Native group cancellation flags failed");return nullptr;
        }
    }
    for(auto *s:sessions){s->pair_owner=g;s->paired_phase.store(true);}
    try {for(unsigned i=0;i<4;++i)g->workers[i]=std::thread(&NativeGroup::owner,g,i);}
    catch(...) {
        {std::lock_guard<std::mutex> lock(g->mutex);g->closing=true;}
        g->release.notify_all();for(auto &thread:g->workers)if(thread.joinable())thread.join();
        for(auto *s:sessions){s->paired_phase.store(false);s->pair_owner=nullptr;}
        close(g->cancellation[0]);close(g->cancellation[1]);delete g;
        group_error(error,size,"Native group thread creation failed");return nullptr;
    }
    for(auto &lock:locks)lock.unlock();
    g->settings_phase(false);std::memcpy(settings,g->settings,sizeof(g->settings));
    if(!g->placement_ok()) {
        g->settings_phase(true);
        std::memcpy(settings,g->settings,sizeof(g->settings));
        {std::lock_guard<std::mutex> lock(g->mutex);g->closing=true;}
        g->release.notify_all();for(auto &thread:g->workers)thread.join();
        lock_sessions(g,locks);for(auto *s:sessions){s->paired_phase.store(false);s->pair_owner=nullptr;}
        close(g->cancellation[0]);close(g->cancellation[1]);delete g;
        group_error(error,size,"Native group placement/slack readback failed; not admitted");return nullptr;
    }
    return g;
}

extern "C" int sdg_start_hold(void *handle,const unsigned char *triples,
        const unsigned char *voltages,uint64_t deadline,uint64_t *ticket,char *error,uint32_t size) {
    auto *g=static_cast<NativeGroup*>(handle);
    if(!g||!triples||!voltages||!ticket||!error||!size)return -1;
    *ticket=0;error[0]=0;
    std::unique_lock<std::mutex> operation(g->operation,std::try_to_lock);
    if(!operation.owns_lock()){group_error(error,size,"Concurrent native group operation");return -1;}
    std::unique_lock<std::mutex> lock(g->mutex);
    if(!g->ready_for_cycle||g->closing) {group_error(error,size,"Previous native group cycle remains unconsumed");return -1;}
    std::unique_lock<std::mutex> session_locks[4];lock_sessions(g,session_locks);
    auto fail=[&](const char *why) {g->poison_all();group_error(error,size,why);return -1;};
    if(const char *why=g->session_problem())return fail(why);
    const uint64_t stamp=now();
    if(!stamp||deadline<=stamp||deadline-stamp>250000000ULL)return fail("Invalid bounded native group absolute deadline");
    // Private copies of ALL input phases before validation/publication.
    for(unsigned i=0;i<4;++i) {
        std::memcpy(g->hold_wires[i].data(),triples+51*i,51);
        std::memcpy(g->voltage_wires[i].data(),voltages+17*i,17);
        if(const char *why=triple_problem(g,i,g->hold_wires[i].data()))return fail(why);
        const auto *v=g->voltage_wires[i].data();
        if(const char *why=subset_problem(g->sessions[i],g->masks[i],v,1))return fail(why);
        if((id(v)>>24)!=17||v[7]!=0x1c||v[8]!=0x70)
            return fail("Native group voltage phase requires canonical bus-voltage parameter request");
    }
    const uint64_t validated=now(),released=now();
    if(!validated||released>=deadline)return fail("Native group validation reached deadline before publication");
    ++g->ticket;g->deadline=deadline;g->write_deadline=0;g->hold_done=g->voltage_done=0;
    g->hold_taken=g->voltage_taken=false;g->ready_for_cycle=false;g->task=2;
    g->issued_bits=3;g->output={};
    g->result_meta(g->hold,stamp,validated,released);g->result_meta(g->voltage,stamp,validated,released);
    ++g->job_generation;*ticket=g->ticket;
    for(auto &guard:session_locks)guard.unlock();
    lock.unlock();g->release.notify_all();return 0;
}
// Optional fresh zero-gain sample after persistent owners are configured and
// ready. This is an actual new ACK sample, never a rebasing of previous stamps.
extern "C" int sdg_initial_hold(void *handle,const unsigned char *triples,
        uint64_t deadline,SDGResult *out,char *error,uint32_t size) {
    auto *g=static_cast<NativeGroup*>(handle);
    if(!g||!triples||!out||!error||!size)return -1;
    *out={};error[0]=0;
    std::unique_lock<std::mutex> operation(g->operation,std::try_to_lock);
    if(!operation.owns_lock()){group_error(error,size,"Concurrent native group operation");return -1;}
    std::unique_lock<std::mutex> lock(g->mutex);
    if(g->ticket||g->initial_done||g->task!=1||g->closing) {
        group_error(error,size,"Initial zero-gain hold is permitted once before any regular ticket");return -1;
    }
    std::unique_lock<std::mutex> guards[4];lock_sessions(g,guards);
    auto fail=[&](const char *why){g->poison_all();group_error(error,size,why);return -1;};
    if(const char *why=g->session_problem())return fail(why);
    const uint64_t stamp=now();
    if(!stamp||deadline<=stamp||deadline-stamp>250000000ULL)return fail("Invalid bounded initial hold deadline");
    for(unsigned i=0;i<4;++i) {
        std::memcpy(g->output_wires[i].data(),triples+51*i,51);
        if(const char *why=triple_problem(g,i,g->output_wires[i].data()))return fail(why);
        for(unsigned axis=0;axis<3;++axis)
            if(!zero(g->output_wires[i].data()+17*axis+11,4))
                return fail("Initial native group bootstrap permits Kp/Kd zero only");
    }
    const uint64_t validated=now(),released=now();
    if(!validated||released>=deadline)return fail("Initial hold validation reached deadline before publication");
    g->deadline=deadline;g->write_deadline=0;g->output_done=0;g->task=4;
    g->issued_bits=8;
    g->result_meta(g->output,stamp,validated,released);++g->job_generation;
    for(auto &guard:guards)guard.unlock();
    g->release.notify_all();
    while(g->output_done!=4) {
        const uint64_t current=now();if(!current||current>=deadline)g->cancel();
        g->completion.wait_for(lock,std::chrono::nanoseconds(current<deadline?deadline-current:1000000ULL));
    }
    g->output.collected_ns=now();g->output.cancel_requested_ns=g->cancel_stamp.load();*out=g->output;
    bool success=!g->cancelled_flag.load();for(auto &row:g->output.ports)if(row.status)success=false;
    g->initial_done=success;
    if(!success)group_error(error,size,"Initial native group zero-gain hold failed; bounded STOP required");
    return success?0:-1;
}
extern "C" int sdg_wait_hold(void *handle,uint64_t ticket,SDGResult *result,char *error,uint32_t size) {
    return wait_result(static_cast<NativeGroup*>(handle),ticket,result,false,error,size);
}
extern "C" int sdg_wait_voltage(void *handle,uint64_t ticket,SDGResult *result,char *error,uint32_t size) {
    return wait_result(static_cast<NativeGroup*>(handle),ticket,result,true,error,size);
}
extern "C" int sdg_output(void *handle,uint64_t ticket,const unsigned char *triples,
        uint64_t reply_deadline,uint64_t write_deadline,int validated_inputs,
        SDGResult *out,char *error,uint32_t size) {
    auto *g=static_cast<NativeGroup*>(handle);
    if(!g||!triples||!out||!error||!size)return -1;
    *out={};error[0]=0;
    std::unique_lock<std::mutex> operation(g->operation,std::try_to_lock);
    if(!operation.owns_lock()){group_error(error,size,"Concurrent native group operation");return -1;}
    std::unique_lock<std::mutex> lock(g->mutex);
    // Correlate a rejected API attempt without calling it an issued phase.
    // released_ns==0 and count/owner/record/stats fields remain zero. In
    // particular it must not collide with the actual ticket-zero bootstrap.
    out->ticket=g->ticket;out->submitted_ns=now();
    auto not_released=[&](const char *why) {
        out->collected_ns=now();out->cancel_requested_ns=g->cancel_stamp.load();
        for(auto &row:out->ports) {
            row.status=-1;std::snprintf(row.error,sizeof(row.error),"%s",why);
        }
        group_error(error,size,why);return -1;
    };
    if(ticket!=g->ticket||!ticket||g->task!=2||!g->hold_taken||!g->voltage_taken||
       g->hold_done!=4||g->voltage_done!=4||validated_inputs!=1||g->closing) {
        return not_released("Both successful current-ticket input phases and explicit validation required");
    }
    std::unique_lock<std::mutex> session_locks[4];lock_sessions(g,session_locks);
    auto fail=[&](const char *why) {g->poison_all();return not_released(why);};
    if(const char *why=g->session_problem())return fail(why);
    const uint64_t stamp=now();
    if(!stamp||write_deadline<=stamp||write_deadline>reply_deadline||reply_deadline-stamp>250000000ULL)
        return fail("Require now<write_deadline<=reply_deadline<=now+250ms");
    for(unsigned i=0;i<4;++i) {
        if(g->hold.ports[i].status||g->voltage.ports[i].status)return fail("Failed input phase cannot authorize native group output");
        std::memcpy(g->output_wires[i].data(),triples+51*i,51);
        if(const char *why=triple_problem(g,i,g->output_wires[i].data()))return fail(why);
    }
    const uint64_t validated=now(),released=now();
    if(!validated||released>=write_deadline)return fail("Native group validation reached output write deadline");
    g->deadline=reply_deadline;g->write_deadline=write_deadline;g->output_done=0;g->task=3;
    g->issued_bits|=4;
    g->result_meta(g->output,stamp,validated,released);++g->job_generation;
    for(auto &guard:session_locks)guard.unlock();
    g->release.notify_all();
    while(g->output_done!=4) {
        const uint64_t current=now();if(!current||current>=reply_deadline)g->cancel();
        g->completion.wait_for(lock,std::chrono::nanoseconds(current<reply_deadline?reply_deadline-current:1000000ULL));
    }
    g->output.collected_ns=now();g->output.cancel_requested_ns=g->cancel_stamp.load();*out=g->output;
    bool success=!g->cancelled_flag.load();for(auto &row:g->output.ports)if(row.status)success=false;
    g->ready_for_cycle=success;
    if(!success)group_error(error,size,"Native group output failed; cancel/join then existing bounded STOP required");
    return success?0:-1;
}
extern "C" void sdg_cancel(void *handle) {auto *g=static_cast<NativeGroup*>(handle);if(g)g->cancel();}
// Preserve native partial exchanges even when Python never consumed a phase.
// Cancellation is sticky. Collection is not success/admission; owner status,
// transmitted/received bytes, original deadlines and ambiguity are unchanged.
extern "C" int sdg_cancel_snapshot(void *handle,SDGResult *hold,SDGResult *voltage,
        SDGResult *output,uint32_t *phase_bits,char *error,uint32_t size) {
    auto *g=static_cast<NativeGroup*>(handle);
    if(!g||!hold||!voltage||!output||!phase_bits||!error||!size)return -1;
    *hold={};*voltage={};*output={};*phase_bits=0;error[0]=0;
    g->cancel();std::unique_lock<std::mutex> operation(g->operation);
    std::unique_lock<std::mutex> lock(g->mutex);
    if(g->closing) {group_error(error,size,"Native group closing during snapshot");return -1;}
    if(g->task==2)g->completion.wait(lock,[&]{return g->voltage_done==4;});
    else if(g->task==3||g->task==4)g->completion.wait(lock,[&]{return g->output_done==4;});
    const uint64_t stamp=now(),cancel=g->cancel_stamp.load();
    auto copy=[&](SDGResult &source,SDGResult *dest) {
        if(!source.collected_ns)source.collected_ns=stamp;
        source.cancel_requested_ns=cancel;*dest=source;
    };
    *phase_bits=g->issued_bits;
    if(g->issued_bits&1)copy(g->hold,hold);
    if(g->issued_bits&2)copy(g->voltage,voltage);
    if(g->issued_bits&(4|8))copy(g->output,output);
    return 0;
}
extern "C" int sdg_close(void *handle,SDPairOwnerSettings *settings,char *error,uint32_t size) {
    auto *g=static_cast<NativeGroup*>(handle);if(!g||!settings||!error||!size)return -1;
    g->cancel();std::unique_lock<std::mutex> operation(g->operation);
    // A start call may have returned while its voltage owners are still active.
    // Join that job before reusing their generation for restoration.
    {std::unique_lock<std::mutex> lock(g->mutex);
        if(g->task==2)g->completion.wait(lock,[&]{return g->voltage_done==4;});
        else if(g->task==3||g->task==4)g->completion.wait(lock,[&]{return g->output_done==4;});}
    g->settings_phase(true);std::memcpy(settings,g->settings,sizeof(g->settings));
    const bool restored=g->placement_ok();
    {std::lock_guard<std::mutex> lock(g->mutex);g->closing=true;}
    g->release.notify_all();for(auto &thread:g->workers)if(thread.joinable())thread.join();
    {std::unique_lock<std::mutex> guards[4];lock_sessions(g,guards);
        // This ownership handback is terminal: only the unchanged emergency
        // STOP API may follow. Even idle cancellation cannot re-arm ordinary
        // active/read exchanges through a raw C caller after the group closes.
        for(auto *s:g->sessions){s->poisoned=true;s->paired_phase.store(false);s->pair_owner=nullptr;}}
    close(g->cancellation[0]);close(g->cancellation[1]);operation.unlock();delete g;
    group_error(error,size,restored?"":"Native group placement restoration readback failed");return restored?0:-1;
}
