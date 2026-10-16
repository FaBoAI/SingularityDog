// Independent CPU-only return probe. No CAN, serial, sockets, frames or foreign
// function pointers. Every counter belongs to the calling native thread.
// Counter snapshots have explicit sampling windows; they are not instantaneous
// measurements of the interval between native return and Python execution.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <sys/resource.h>
#include <time.h>
#include <unistd.h>
#if defined(__linux__)
#include <sched.h>
#include <sys/syscall.h>
#elif defined(__APPLE__)
#include <pthread.h>
#include <mach/mach_time.h>
#endif

struct GPSnapshot {
    uint64_t sample_started_ns, sample_finished_ns, thread_cpu_ns;
    uint64_t user_cpu_ns, system_cpu_ns, minflt, majflt, nvcsw, nivcsw;
    uint64_t sched_cpu_ns, sched_run_delay_ns, sched_timeslices, native_tid;
    int32_t cpu, policy;
    uint32_t valid_bits, reserved;
};
struct GPRecord {
    uint32_t abi;
    int32_t status;
    uint64_t call_started_ns, wait_requested_ns, spin_requested_ns;
    uint64_t wait_started_ns, wait_finished_ns, native_ready_to_return_ns;
    GPSnapshot before, after;
};
static_assert(sizeof(GPSnapshot)==120, "Snapshot ABI changed");
static_assert(sizeof(GPRecord)==296, "Record ABI changed");
enum : uint32_t { THREAD_CPU=1, THREAD_RUSAGE=2, SCHEDSTAT=4, LINUX_PLACEMENT=8 };
static std::atomic<uint32_t> entered{0};

static uint64_t now_ns(clockid_t clock=CLOCK_MONOTONIC) {
#if defined(__APPLE__)
    // CPython time.monotonic_ns() uses mach_absolute_time on macOS, whose
    // suspend semantics differ from Darwin CLOCK_MONOTONIC. Host controls must
    // compare one real clock domain rather than invent a return-gap offset.
    // Actual Jetson/Linux and the original transport both use CLOCK_MONOTONIC.
    if(clock==CLOCK_MONOTONIC) {
        static const mach_timebase_info_data_t info=[] {
            mach_timebase_info_data_t value{};mach_timebase_info(&value);return value;
        }();
        return uint64_t((static_cast<unsigned __int128>(mach_absolute_time())*info.numer)/info.denom);
    }
#endif
    timespec time{};
    if(clock_gettime(clock,&time))return 0;
    return uint64_t(time.tv_sec)*1000000000ULL+uint64_t(time.tv_nsec);
}
static uint64_t timeval_ns(const timeval &value) {
    return uint64_t(value.tv_sec)*1000000000ULL+uint64_t(value.tv_usec)*1000ULL;
}
static uint64_t native_tid() {
#if defined(__linux__)
    return uint64_t(syscall(SYS_gettid));
#elif defined(__APPLE__)
    uint64_t tid=0;
    return pthread_threadid_np(nullptr,&tid) ? 0 : tid;
#else
    return 0;
#endif
}
static void capture(GPSnapshot &result) {
    std::memset(&result,0,sizeof(result));
    result.cpu=result.policy=-1;
    result.sample_started_ns=now_ns();
    result.native_tid=native_tid();
#if defined(CLOCK_THREAD_CPUTIME_ID)
    result.thread_cpu_ns=now_ns(CLOCK_THREAD_CPUTIME_ID);
    if(result.thread_cpu_ns)result.valid_bits|=THREAD_CPU;
#endif
#if defined(__linux__)
    rusage usage{};
    if(!getrusage(RUSAGE_THREAD,&usage)) {
        result.user_cpu_ns=timeval_ns(usage.ru_utime);
        result.system_cpu_ns=timeval_ns(usage.ru_stime);
        result.minflt=uint64_t(usage.ru_minflt);result.majflt=uint64_t(usage.ru_majflt);
        result.nvcsw=uint64_t(usage.ru_nvcsw);result.nivcsw=uint64_t(usage.ru_nivcsw);
        result.valid_bits|=THREAD_RUSAGE;
    }
    result.cpu=sched_getcpu();result.policy=sched_getscheduler(0);
    if(result.cpu>=0 && result.policy>=0)result.valid_bits|=LINUX_PLACEMENT;
    // Linux can retain cpu/runtime/timeslices after schedstats is disabled.
    // Those nonzero values therefore cannot authenticate zero runnable delay.
    bool schedstats_enabled=false;
    int setting=open("/proc/sys/kernel/sched_schedstats",O_RDONLY|O_CLOEXEC);
    if(setting>=0) {
        char value[16]{};
        const ssize_t amount=pread(setting,value,sizeof(value)-1,0);
        close(setting);
        int enabled=-1;
        schedstats_enabled=amount>0 && std::sscanf(value,"%d",&enabled)==1 && enabled==1;
    }
    char path[96], bytes[128]{};
    std::snprintf(path,sizeof(path),"/proc/self/task/%llu/schedstat",
                  static_cast<unsigned long long>(result.native_tid));
    int fd=open(path,O_RDONLY|O_CLOEXEC);
    if(fd>=0) {
        ssize_t amount=pread(fd,bytes,sizeof(bytes)-1,0);
        close(fd);
        unsigned long long cpu=0,delay=0,slices=0;
        // The actual parsed zero is a measurement only while the explicit
        // kernel setting is on. If off/unreadable, Python exposes None via bit4.
        if(schedstats_enabled && amount>0 && std::sscanf(bytes,"%llu %llu %llu",&cpu,&delay,&slices)==3) {
            result.sched_cpu_ns=cpu;result.sched_run_delay_ns=delay;
            result.sched_timeslices=slices;result.valid_bits|=SCHEDSTAT;
        }
    }
#endif
    result.sample_finished_ns=now_ns();
}
static int fail(char *error,uint32_t size,const char *message) {
    if(error && size)std::snprintf(error,size,"%s",message);
    return -1;
}
extern "C" uint32_t gp_abi() {return 1;}
extern "C" uint32_t gp_snapshot_size() {return sizeof(GPSnapshot);}
extern "C" uint32_t gp_record_size() {return sizeof(GPRecord);}
extern "C" void gp_reset_entered() {entered.store(0,std::memory_order_release);}
extern "C" uint32_t gp_entered() {return entered.load(std::memory_order_acquire);}
extern "C" int gp_snapshot(GPSnapshot *snapshot,char *error,uint32_t size) {
    if(!snapshot || !error || !size)return fail(error,size,"Snapshot/error storage required");
    error[0]=0;capture(*snapshot);
    if(!snapshot->sample_started_ns || snapshot->sample_finished_ns<snapshot->sample_started_ns)
        return fail(error,size,"Monotonic snapshot clock failed");
    return 0;
}
extern "C" int gp_wait(uint64_t wait_ns,uint64_t spin_ns,GPRecord *record,char *error,uint32_t size) {
    if(!record || !error || !size)return fail(error,size,"Record/error storage required");
    std::memset(record,0,sizeof(*record));record->abi=1;record->status=-1;error[0]=0;
    record->call_started_ns=now_ns();record->wait_requested_ns=wait_ns;record->spin_requested_ns=spin_ns;
    if(!wait_ns || wait_ns>1000000000ULL || spin_ns>wait_ns)
        return fail(error,size,"Wait must be1ns..1s, with spin no longer than wait");
    capture(record->before);
    record->wait_started_ns=now_ns();
    if(!record->wait_started_ns || record->wait_started_ns>std::numeric_limits<uint64_t>::max()-wait_ns)
        return fail(error,size,"Monotonic wait clock failed");
    const uint64_t end=record->wait_started_ns+wait_ns;
    entered.store(1,std::memory_order_release);
    while(true) {
        const uint64_t now=now_ns();
        if(!now)return fail(error,size,"Monotonic clock failed during wait");
        if(now>=end)break;
        const uint64_t remaining=end-now;
        if(remaining>spin_ns) {
            const uint64_t sleeping=remaining-spin_ns;
            const timespec request{time_t(sleeping/1000000000ULL),long(sleeping%1000000000ULL)};
            if(nanosleep(&request,nullptr) && errno!=EINTR)
                return fail(error,size,"Native nanosleep failed");
        } else {
            // Pure native bounded CPU load, also used for same-CPU OS contention.
            std::atomic_signal_fence(std::memory_order_seq_cst);
        }
    }
    record->wait_finished_ns=now_ns();
    capture(record->after);
    record->status=0;
    // Last native timestamp. Python's first timestamp follows the real ctypes
    // return/GIL reacquisition; this field never contains a predicted deadline.
    record->native_ready_to_return_ns=now_ns();
    return 0;
}
