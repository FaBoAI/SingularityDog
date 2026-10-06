// Isolated fake pipe prototype. NO CAN frame codec, sd_exchange or motor opcode.
// Persistent pthread owners; fixed generation/phase slots and monotonic waits.
#if defined(__linux__) && !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <map>
#include <mutex>
#include <new>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

extern "C" {
struct FakePacket { unsigned char magic[8]; uint64_t generation; uint32_t phase, bus; uint64_t nonce; };
struct FakeRecord {
    uint64_t generation, start_ns, finish_ns, deadline_ns, read_start_ns, received_ns, prior_finish_ns, nonce;
    uint32_t bus, phase, written, received, status, reserved;
    unsigned char tx[32], rx[32];
};
struct FakePair { uint64_t generation; uint32_t phase, reserved; FakeRecord records[2]; };
struct FakeStatus {
    uint32_t abi, state, error_code, error_bus, error_phase, cancelled, stop_admitted, exited_mask;
    uint32_t joined_mask, owned_closed_mask, ready_mask, consumed_mask;
    uint64_t generation, deadline_ns, notifications, last_finish_ns[2];
    char error[160]; FakeRecord records[6];
};
uint32_t sdf_abi() { return 1; }
uint32_t sdf_record_size() { return sizeof(FakeRecord); }
uint32_t sdf_pair_size() { return sizeof(FakePair); }
uint32_t sdf_status_size() { return sizeof(FakeStatus); }
}
static_assert(sizeof(FakePacket)==32 && sizeof(FakeRecord)==152 && sizeof(FakePair)==320, "Fake ABI changed");
namespace {
constexpr uint64_t BUDGET=20000000, GAP=900000, CHECKPOINT=200000, CLOSE_MAX=250000000;
constexpr uint32_t IDLE=0, RUNNING=1, AWAIT_STOP=2, COMPLETE=3, POISONED=4, CLOSING=5, CLOSED=6;
constexpr uint32_t DEADLINE=2, CANCELLED=3, PROTOCOL=4, IO=5, FD_CHANGED=6, CLOCK_ERROR=7;
const unsigned char QUERY[8]={'S','D','F','A','K','E','Q','1'};
const unsigned char REPLY[8]={'S','D','F','K','R','E','P','1'};
uint64_t now() { timespec t{}; return clock_gettime(CLOCK_MONOTONIC,&t)?0:uint64_t(t.tv_sec)*1000000000ULL+t.tv_nsec; }
struct Session;
struct Owner { Session *session{}; int bus{}, tx{-1}, rx{-1}; dev_t tx_dev{},rx_dev{}; ino_t tx_ino{},rx_ino{}; pthread_t thread{}; bool started{}; };
struct Session {
    pthread_mutex_t mutex{}; pthread_cond_t condition{}; pthread_t coordinator{};
    std::atomic_flag coordinating=ATOMIC_FLAG_INIT;
    std::atomic<bool> cancelled{false}, closing{false}, poisoned{false};
    Owner owners[2]{}; FakeStatus status{}; uint64_t seed{}; bool stop_requested{};
};
// Tokens never repeat and are not native addresses. Holding this registry
// mutex through a C entry prevents lookup/deletion races, including a Python
// interrupt after close returns but before its wrapper clears the token.
std::mutex registry_mutex;
std::map<uintptr_t,Session*> registry;
uintptr_t next_token=1;
struct Lookup {
    std::unique_lock<std::mutex> lock{registry_mutex};
    Session *session{};
    explicit Lookup(void *token) {
        auto found=registry.find(reinterpret_cast<uintptr_t>(token));
        if(found!=registry.end())session=found->second;
    }
};
void message(char *out,uint32_t size,const char *text) { if(out&&size) std::snprintf(out,size,"%s",text); }
struct Entry {
    Session *s; bool held{};
    Entry(Session *p,char *e,uint32_t n):s(p) {
        if(!s) { message(e,n,"Null fake session"); return; }
        if(!pthread_equal(s->coordinator,pthread_self())) { message(e,n,"Wrong coordinator thread"); return; }
        held=!s->coordinating.test_and_set();
        if(!held) message(e,n,"Reentrant coordinator call");
    }
    ~Entry(){ if(held)s->coordinating.clear(); }
};
void notify(Session *s) { ++s->status.notifications; pthread_cond_broadcast(&s->condition); }
void poison_locked(Session *s,uint32_t code,uint32_t bus,uint32_t phase,const char *text) {
    // First native error remains authoritative even if cancellation follows.
    if(!s->poisoned.load()) {
        s->poisoned.store(true);s->status.error_code=code;s->status.error_bus=bus;s->status.error_phase=phase;
        message(s->status.error,sizeof(s->status.error),text);s->status.state=POISONED;
    }
    notify(s);
}
void poison(Session *s,uint32_t code,uint32_t bus,uint32_t phase,const char *text) {
    pthread_mutex_lock(&s->mutex);poison_locked(s,code,bus,phase,text);pthread_mutex_unlock(&s->mutex);
}
int cond_until(Session *s,uint64_t deadline) {
    const uint64_t current=now(); if(!current||current>=deadline)return ETIMEDOUT;
    const uint64_t end=std::min(deadline,current+CHECKPOINT);
#ifdef __APPLE__
    // Darwin lacks condattr_setclock. A bounded relative wait uses the same
    // fresh monotonic deadline; it is NOT a realtime-absolute conversion.
    timespec relative{time_t((end-current)/1000000000ULL),long((end-current)%1000000000ULL)};
    return pthread_cond_timedwait_relative_np(&s->condition,&s->mutex,&relative);
#else
    timespec absolute{time_t(end/1000000000ULL),long(end%1000000000ULL)};
    return pthread_cond_timedwait(&s->condition,&s->mutex,&absolute);
#endif
}
bool identity(int fd,dev_t device,ino_t inode,bool write) {
    struct stat st{};int flags=fcntl(fd,F_GETFL);
    return fstat(fd,&st)==0&&S_ISFIFO(st.st_mode)&&st.st_dev==device&&st.st_ino==inode&&flags>=0&&
        (flags&O_NONBLOCK)&&((flags&O_ACCMODE)==(write?O_WRONLY:O_RDONLY));
}
bool checked(Session *s,Owner &owner,FakeRecord &record) {
    if(s->poisoned.load()||s->closing.load()||s->cancelled.load()) {
        record.status=CANCELLED;return false;
    }
    if(!identity(owner.tx,owner.tx_dev,owner.tx_ino,true)||!identity(owner.rx,owner.rx_dev,owner.rx_ino,false)) {
        record.status=FD_CHANGED;poison(s,FD_CHANGED,owner.bus,record.phase,"Owned fake pipe identity/flags changed");return false;
    }
    const uint64_t current=now();
    if(!current){record.status=CLOCK_ERROR;poison(s,CLOCK_ERROR,owner.bus,record.phase,"Monotonic clock failure");return false;}
    if(current>=record.deadline_ns){record.status=DEADLINE;poison(s,DEADLINE,owner.bus,record.phase,"Fake exchange deadline exceeded");return false;}
    return true;
}
bool wait_fd(Session *s,Owner &owner,FakeRecord &record,int fd,short events,uint64_t not_before=0) {
    while(checked(s,owner,record)) {
        uint64_t current=now();if(not_before&&current>=not_before)return true;
        uint64_t end=std::min(record.deadline_ns,current+CHECKPOINT);
        if(not_before)end=std::min(end,not_before);
        timespec delay{time_t((end-current)/1000000000ULL),long((end-current)%1000000000ULL)};
        pollfd item{fd,events,0};int result;
        if(not_before){result=poll(nullptr,0,0);if(result==0)nanosleep(&delay,nullptr);}
        else {
#ifdef __APPLE__
            // poll timeout rounds to milliseconds; select's timeval preserves
            // the <=200us checkpoint on the local fixture platform.
            fd_set readset,writeset;FD_ZERO(&readset);FD_ZERO(&writeset);
            if(fd>=FD_SETSIZE){record.status=FD_CHANGED;poison(s,FD_CHANGED,owner.bus,record.phase,"Fake fd exceeds local wait bound");return false;}
            if(events&POLLIN)FD_SET(fd,&readset);if(events&POLLOUT)FD_SET(fd,&writeset);
            timeval timeout{time_t((end-current)/1000000000ULL),suseconds_t(((end-current)%1000000000ULL+999)/1000)};
            result=select(fd+1,&readset,&writeset,nullptr,&timeout);
#else
            result=ppoll(&item,1,&delay,nullptr);
#endif
        }
        if(result<0&&errno!=EINTR){record.status=IO;poison(s,IO,owner.bus,record.phase,"Fake pipe wait error");return false;}
        if(!not_before&&result>0)return checked(s,owner,record);
    }
    return false;
}
FakeRecord exchange(Session *s,Owner &owner,uint64_t generation,uint32_t phase,uint64_t deadline,uint64_t seed) {
    FakeRecord record{};record.generation=generation;record.bus=owner.bus;record.phase=phase;record.deadline_ns=deadline;
    record.nonce=seed^generation^(uint64_t(owner.bus)<<48)^(uint64_t(phase)<<56);
    pthread_mutex_lock(&s->mutex);record.prior_finish_ns=s->status.last_finish_ns[owner.bus];pthread_mutex_unlock(&s->mutex);
    FakePacket packet{};std::memcpy(packet.magic,QUERY,8);packet.generation=generation;packet.phase=phase;packet.bus=owner.bus;packet.nonce=record.nonce;
    std::memcpy(record.tx,&packet,32);
    if(record.prior_finish_ns&&now()<record.prior_finish_ns+GAP)
        if(!wait_fd(s,owner,record,-1,0,record.prior_finish_ns+GAP)){record.finish_ns=now();return record;}
    record.start_ns=now();
    while(record.written<32&&checked(s,owner,record)) {
        ssize_t result=write(owner.tx,record.tx+record.written,32-record.written);
        if(result>0)record.written+=uint32_t(result);
        else if(result<0&&(errno==EINTR||errno==EAGAIN)){if(!wait_fd(s,owner,record,owner.tx,POLLOUT))break;}
        else{record.status=IO;poison(s,IO,owner.bus,phase,"Fake request write failed");break;}
    }
    record.read_start_ns=now();
    while(record.written==32&&record.received<32&&checked(s,owner,record)) {
        ssize_t result=read(owner.rx,record.rx+record.received,32-record.received);
        if(result>0) {
            record.received+=uint32_t(result);record.received_ns=now();
            if(record.received==32) {
                FakePacket reply{};std::memcpy(&reply,record.rx,32);
                // Check data errors before considering a late completion.
                if(std::memcmp(reply.magic,REPLY,8)||reply.generation!=generation||reply.phase!=phase||reply.bus!=uint32_t(owner.bus)||reply.nonce!=record.nonce) {
                    record.status=PROTOCOL;poison(s,PROTOCOL,owner.bus,phase,"Fake reply generation/phase/bus/nonce mismatch");break;
                }
                if(checked(s,owner,record))record.status=1;
                break;
            }
        } else if(result<0&&(errno==EINTR||errno==EAGAIN)){if(!wait_fd(s,owner,record,owner.rx,POLLIN))break;}
        else{record.status=IO;poison(s,IO,owner.bus,phase,"Fake reply EOF/read failure");break;}
    }
    record.finish_ns=now();if(record.status==0)record.status=CANCELLED;
    return record;
}
void publish(Session *s,Owner &owner,const FakeRecord &record) {
    pthread_mutex_lock(&s->mutex);
    const uint32_t slot=owner.bus*3+record.phase-1;
    s->status.records[slot]=record;s->status.ready_mask|=1U<<slot;
    if(record.status==1)s->status.last_finish_ns[owner.bus]=record.finish_ns;
    if(record.phase==2&&(s->status.ready_mask&0x12U)==0x12U&&!s->poisoned.load())s->status.state=AWAIT_STOP;
    if(record.phase==3&&(s->status.ready_mask&0x24U)==0x24U&&!s->poisoned.load())s->status.state=COMPLETE;
    notify(s);pthread_mutex_unlock(&s->mutex);
}
void *owner_main(void *arg) {
    Owner &owner=*static_cast<Owner*>(arg);Session *s=owner.session;uint64_t seen=0;
    // A closed fake reader becomes EPIPE, never a process-wide SIGPIPE exit.
    sigset_t blocked;sigemptyset(&blocked);sigaddset(&blocked,SIGPIPE);pthread_sigmask(SIG_BLOCK,&blocked,nullptr);
    pthread_mutex_lock(&s->mutex);
    while(!s->closing.load()) {
        while(!s->closing.load()&&(s->status.generation==seen||s->poisoned.load()))pthread_cond_wait(&s->condition,&s->mutex);
        if(s->closing.load())break;
        const uint64_t generation=s->status.generation,deadline=s->status.deadline_ns,seed=s->seed;seen=generation;
        pthread_mutex_unlock(&s->mutex);
        for(uint32_t phase=1;phase<=2&&!s->poisoned.load()&&!s->closing.load();++phase)publish(s,owner,exchange(s,owner,generation,phase,deadline,seed));
        pthread_mutex_lock(&s->mutex);
        while(!s->closing.load()&&!s->poisoned.load()&&!s->stop_requested){
            if(now()>=deadline){poison_locked(s,DEADLINE,owner.bus,3,"Fake validation/STOP submission deadline expired");break;}
            cond_until(s,deadline);
        }
        if(!s->closing.load()&&!s->poisoned.load()) {
            pthread_mutex_unlock(&s->mutex);publish(s,owner,exchange(s,owner,generation,3,deadline,seed));pthread_mutex_lock(&s->mutex);
        }
    }
    s->status.exited_mask|=1U<<owner.bus;notify(s);pthread_mutex_unlock(&s->mutex);return nullptr;
}
void status_copy(Session *s,FakeStatus *out){if(out){*out=s->status;out->cancelled=s->cancelled.load();}}
bool descriptors(int tx,int rx,Owner &owner,char *error,uint32_t size) {
    struct stat a{},b{};int af=fcntl(tx,F_GETFL),bf=fcntl(rx,F_GETFL);
    if(tx<0||rx<0||tx==rx||fstat(tx,&a)||fstat(rx,&b)||!S_ISFIFO(a.st_mode)||!S_ISFIFO(b.st_mode)||
       af<0||bf<0||!(af&O_NONBLOCK)||!(bf&O_NONBLOCK)||(af&O_ACCMODE)!=O_WRONLY||(bf&O_ACCMODE)!=O_RDONLY||
       (a.st_dev==b.st_dev&&a.st_ino==b.st_ino)) {message(error,size,"Distinct nonblocking read/write FIFO pipes required");return false;}
    owner.tx=fcntl(tx,F_DUPFD_CLOEXEC,0);owner.rx=fcntl(rx,F_DUPFD_CLOEXEC,0);
    if(owner.tx<0||owner.rx<0){message(error,size,"Fake pipe duplication failed");return false;}
    owner.tx_dev=a.st_dev;owner.tx_ino=a.st_ino;owner.rx_dev=b.st_dev;owner.rx_ino=b.st_ino;return true;
}
void close_fds(Session *s) {for(auto &owner:s->owners){if(owner.tx>=0){close(owner.tx);owner.tx=-1;}if(owner.rx>=0){close(owner.rx);owner.rx=-1;}}}
}
extern "C" uint64_t sdf_now_ns(){return now();}
static void *create_impl(int front_tx,int front_rx,int rear_tx,int rear_rx,char *error,uint32_t size) {
    message(error,size,"");if(!error||!size)return nullptr;
    Session *s=new(std::nothrow)Session;if(!s){message(error,size,"Allocation failed");return nullptr;}
    s->coordinator=pthread_self();s->status.abi=1;s->status.error_bus=s->status.error_phase=UINT32_MAX;
    int inputs[4]={front_tx,front_rx,rear_tx,rear_rx};
    for(int a=0;a<4;++a)for(int b=0;b<a;++b)if(inputs[a]==inputs[b]){message(error,size,"Duplicate supplied fake fd");delete s;return nullptr;}
    for(int bus=0;bus<2;++bus){s->owners[bus].session=s;s->owners[bus].bus=bus;if(!descriptors(inputs[bus*2],inputs[bus*2+1],s->owners[bus],error,size)){close_fds(s);delete s;return nullptr;}}
    struct stat physical[4]{};for(int a=0;a<4;++a)fstat(inputs[a],&physical[a]);
    for(int a=0;a<4;++a)for(int b=0;b<a;++b)
        if(physical[a].st_dev==physical[b].st_dev&&physical[a].st_ino==physical[b].st_ino){message(error,size,"Fake buses must have four disjoint pipes");close_fds(s);delete s;return nullptr;}
    if(pthread_mutex_init(&s->mutex,nullptr)){message(error,size,"Fake mutex initialization failed");close_fds(s);delete s;return nullptr;}
    pthread_condattr_t attr;
    if(pthread_condattr_init(&attr)){message(error,size,"Fake condition attribute initialization failed");pthread_mutex_destroy(&s->mutex);close_fds(s);delete s;return nullptr;}
#ifndef __APPLE__
    if(pthread_condattr_setclock(&attr,CLOCK_MONOTONIC)){message(error,size,"Monotonic condition clock unavailable");pthread_condattr_destroy(&attr);pthread_mutex_destroy(&s->mutex);close_fds(s);delete s;return nullptr;}
#endif
    if(pthread_cond_init(&s->condition,&attr)){message(error,size,"Fake condition initialization failed");pthread_condattr_destroy(&attr);pthread_mutex_destroy(&s->mutex);close_fds(s);delete s;return nullptr;}
    pthread_condattr_destroy(&attr);
    for(auto &owner:s->owners) {
        if(pthread_create(&owner.thread,nullptr,owner_main,&owner)) {
            message(error,size,"Fake owner thread creation failed");s->closing.store(true);pthread_mutex_lock(&s->mutex);notify(s);pthread_mutex_unlock(&s->mutex);
            for(auto &other:s->owners)if(other.started)pthread_join(other.thread,nullptr);
            pthread_cond_destroy(&s->condition);pthread_mutex_destroy(&s->mutex);close_fds(s);delete s;return nullptr;
        }owner.started=true;
    }
    try {
        std::lock_guard<std::mutex> lock(registry_mutex);
        if(next_token==0||next_token==UINTPTR_MAX)throw std::bad_alloc();
        const uintptr_t token=next_token++;registry.emplace(token,s);return reinterpret_cast<void*>(token);
    } catch(...) {
        message(error,size,"Fake token registration failed");s->closing.store(true);
        pthread_mutex_lock(&s->mutex);notify(s);pthread_mutex_unlock(&s->mutex);
        for(auto &owner:s->owners)pthread_join(owner.thread,nullptr);
        pthread_cond_destroy(&s->condition);pthread_mutex_destroy(&s->mutex);close_fds(s);delete s;return nullptr;
    }
}
extern "C" int sdf_create_into(int front_tx,int front_rx,int rear_tx,int rear_rx,void **token_out,char *error,uint32_t size) {
    message(error,size,"");
    if(!token_out||*token_out||!error||!size){message(error,size,"Empty caller-owned token cell required");return -1;}
    void *token=create_impl(front_tx,front_rx,rear_tx,rear_rx,error,size);
    if(!token)return -1;
    // Publish into an already reachable owner before returning through ctypes.
    // An interrupt at the Python return boundary cannot hide native resources.
    *token_out=token;return 0;
}
extern "C" int sdf_begin(void *handle,uint64_t generation,uint64_t deadline,uint64_t seed,char *error,uint32_t size) {
    Lookup lookup(handle);auto *s=lookup.session;Entry entry(s,error,size);if(!entry.held)return -1;message(error,size,"");
    pthread_mutex_lock(&s->mutex);const uint64_t current=now();int result=-1;
    if(s->poisoned.load()||s->closing.load()||s->cancelled.load())message(error,size,"Sticky fake session poison/cancellation");
    else if(!generation||generation<=s->status.generation||!current||deadline<=current||deadline-current>BUDGET)message(error,size,"Fresh increasing generation and <=20ms monotonic deadline required");
    else if(s->status.state!=IDLE&&(s->status.state!=COMPLETE||s->status.consumed_mask!=63))message(error,size,"Previous generation must be complete and fully acknowledged");
    else {
        s->status.generation=generation;s->status.deadline_ns=deadline;s->status.state=RUNNING;s->seed=seed;s->stop_requested=false;
        s->status.ready_mask=s->status.consumed_mask=s->status.stop_admitted=0;std::memset(s->status.records,0,sizeof(s->status.records));notify(s);result=0;
    }
    pthread_mutex_unlock(&s->mutex);return result;
}
extern "C" int sdf_collect(void *handle,uint64_t generation,uint32_t phase,uint64_t wait_ns,FakePair *out,uint64_t *actual_ns,char *error,uint32_t size) {
    if(out)std::memset(out,0,sizeof(*out));if(actual_ns)*actual_ns=0;message(error,size,"");
    Lookup lookup(handle);auto *s=lookup.session;Entry entry(s,error,size);if(!entry.held||!out||!actual_ns)return -1;
    pthread_mutex_lock(&s->mutex);int result=-1;uint64_t current=now();
    if(generation!=s->status.generation||phase<1||phase>3||!current||wait_ns>CHECKPOINT){message(error,size,"Exact generation/phase and <=200us collect slice required");goto done;}
    {
    const uint64_t wait_until=current+wait_ns;
    for(;;){
        // Authoritative native failure wins timeout/readiness checks.
        if(s->poisoned.load()){message(error,size,s->status.error);break;}
        if(s->closing.load()||s->cancelled.load()){message(error,size,"Fake session cancelled");break;}
        current=now();if(!current||current>=s->status.deadline_ns){poison_locked(s,DEADLINE,UINT32_MAX,phase,"Fake coordinator generation deadline exceeded");message(error,size,s->status.error);break;}
        const uint32_t bits=(1U<<(phase-1))|(1U<<(phase+2));
        if((s->status.ready_mask&bits)==bits){
            out->generation=generation;out->phase=phase;out->records[0]=s->status.records[phase-1];out->records[1]=s->status.records[phase+2];result=1;break;
        }
        if(current>=wait_until){result=0;break;}
        int status=cond_until(s,std::min(wait_until,s->status.deadline_ns));
        if(status!=0&&status!=ETIMEDOUT&&status!=EINTR){poison_locked(s,IO,UINT32_MAX,phase,"Fake condition wait failure");message(error,size,s->status.error);break;}
    }
    }
 done:*actual_ns=now();pthread_mutex_unlock(&s->mutex);return result;
}
extern "C" int sdf_ack(void *handle,const FakePair *pair,char *error,uint32_t size) {
    Lookup lookup(handle);auto *s=lookup.session;Entry entry(s,error,size);if(!entry.held||!pair)return -1;message(error,size,"");
    pthread_mutex_lock(&s->mutex);int result=-1;
    if(s->poisoned.load()||s->closing.load())message(error,size,"Sticky fake session poison");
    else if(pair->generation!=s->status.generation||pair->phase<1||pair->phase>3||pair->reserved)message(error,size,"Exact fake pair generation/phase required");
    else {
        const uint32_t phase=pair->phase,bits=(1U<<(phase-1))|(1U<<(phase+2));
        if((s->status.ready_mask&bits)!=bits||(s->status.consumed_mask&bits))message(error,size,"Unconsumed complete pair required");
        else if(pair->records[0].status!=1||pair->records[1].status!=1||std::memcmp(&pair->records[0],&s->status.records[phase-1],sizeof(FakeRecord))||std::memcmp(&pair->records[1],&s->status.records[phase+2],sizeof(FakeRecord))) {
            poison_locked(s,PROTOCOL,UINT32_MAX,phase,"Collected fake pair changed before validation acknowledgement");message(error,size,s->status.error);
        } else{s->status.consumed_mask|=bits;result=0;}
    }
    pthread_mutex_unlock(&s->mutex);return result;
}
extern "C" int sdf_submit_stop(void *handle,uint64_t generation,uint32_t validated,char *error,uint32_t size) {
    Lookup lookup(handle);auto *s=lookup.session;Entry entry(s,error,size);if(!entry.held)return -1;message(error,size,"");
    pthread_mutex_lock(&s->mutex);int result=-1;
    if(s->poisoned.load()||s->closing.load())message(error,size,"Sticky fake session poison");
    else if(generation!=s->status.generation||validated!=1||s->stop_requested||(s->status.consumed_mask&0x1bU)!=0x1bU)message(error,size,"Both same-generation feedback/voltage pairs must be explicitly validated/acknowledged before fake STOP_ONLY");
    else if(now()>=s->status.deadline_ns){poison_locked(s,DEADLINE,UINT32_MAX,3,"Fake STOP admission deadline exceeded");message(error,size,s->status.error);}
    else{s->stop_requested=true;s->status.stop_admitted=1;notify(s);result=0;}
    pthread_mutex_unlock(&s->mutex);return result;
}
extern "C" int sdf_cancel(void *handle) {
    Lookup lookup(handle);auto *s=lookup.session;if(!s)return -1;s->cancelled.store(true);
    pthread_mutex_lock(&s->mutex);s->status.cancelled=1;poison_locked(s,CANCELLED,UINT32_MAX,0,"External fake cancellation");pthread_mutex_unlock(&s->mutex);return 0;
}
extern "C" int sdf_status(void *handle,FakeStatus *out,char *error,uint32_t size) {
    if(out)std::memset(out,0,sizeof(*out));Lookup lookup(handle);auto *s=lookup.session;Entry entry(s,error,size);if(!entry.held||!out)return -1;
    pthread_mutex_lock(&s->mutex);status_copy(s,out);pthread_mutex_unlock(&s->mutex);return 0;
}
extern "C" int sdf_close(void *handle,uint64_t deadline,FakeStatus *out,char *error,uint32_t size) {
    if(out)std::memset(out,0,sizeof(*out));Lookup lookup(handle);auto *s=lookup.session;
    if(!s&&handle){message(error,size,"Native fake token already closed; cleanup evidence unavailable");return 2;}
    Entry entry(s,error,size);if(!entry.held||!out)return -1;message(error,size,"");
    uint64_t current=now();if(!current||deadline<=current||deadline-current>CLOSE_MAX){message(error,size,"Fresh <=250ms cleanup deadline required");return -1;}
    s->closing.store(true);pthread_mutex_lock(&s->mutex);
    if(s->status.state==RUNNING||s->status.state==AWAIT_STOP){s->cancelled.store(true);poison_locked(s,CANCELLED,UINT32_MAX,0,"Closing in-flight fake generation");}
    s->status.state=CLOSING;notify(s);
    while(s->status.exited_mask!=3&&now()<deadline)cond_until(s,deadline);
    status_copy(s,out);pthread_mutex_unlock(&s->mutex);
    if(out->exited_mask!=3){message(error,size,"Fake owners not reaped by cleanup deadline; retain session");return -1;}
    for(auto &owner:s->owners){
        if(s->status.joined_mask&(1U<<owner.bus))continue;
#ifdef __linux__
        int rc;
        do {rc=pthread_tryjoin_np(owner.thread,nullptr);if(rc==EBUSY){timespec delay{0,long(CHECKPOINT)};nanosleep(&delay,nullptr);}}while(rc==EBUSY&&now()<deadline);
#else
        // All owner exit flags are set after their final access under mutex.
        // Darwin's join has no timed API: wall deadline is checked afterward,
        // and a late join is reported as cleanup-late, never bounded success.
        int rc=pthread_join(owner.thread,nullptr);
#endif
        if(rc){message(error,size,"Fake owner join incomplete; retain session");pthread_mutex_lock(&s->mutex);status_copy(s,out);pthread_mutex_unlock(&s->mutex);return -1;}
        s->status.joined_mask|=1U<<owner.bus;
    }
    close_fds(s);s->status.owned_closed_mask=15;s->status.state=CLOSED;status_copy(s,out);
    const bool late=now()>=deadline;
    // Entry must not access the freed coordination flag.
    entry.held=false;s->coordinating.clear();registry.erase(reinterpret_cast<uintptr_t>(handle));
    pthread_cond_destroy(&s->condition);pthread_mutex_destroy(&s->mutex);delete s;
    if(late){message(error,size,"Fake cleanup completed after deadline");return 1;}
    return 0;
}
