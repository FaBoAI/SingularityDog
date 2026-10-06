// Standalone proposal primitive. Not included by any transport or controller.
#pragma once
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace proposed_trace {
enum class Kind : uint32_t { BootRead, Select, Read, Write };
struct Context {
    Kind kind;
    uint32_t loop, sent, completed;
    uint64_t original_deadline_ns, requested_wake_ns, requested_wait_ns;
};
struct Event {
    Context context;
    uint64_t wall_begin_ns, wall_end_ns, cpu_begin_ns, cpu_end_ns;
    int64_t returned;
    int saved_errno;
    bool cpu_measured, clock_valid;
};
template<std::size_t Capacity> struct Recorder {
    static_assert(Capacity > 0, "A nonempty fixed buffer is required");
    Event events[Capacity]{};
    std::size_t stored=0;
    uint64_t calls=0, dropped=0;
    bool overflow=false, invalid_clock=false;
    void append(const Event& event) noexcept {
        if(calls<std::numeric_limits<uint64_t>::max())++calls;
        invalid_clock|=!event.clock_valid;
        if(stored<Capacity)events[stored++]=event;
        else {
            overflow=true;
            if(dropped<std::numeric_limits<uint64_t>::max())++dropped;
        }
    }
};
using Clock=uint64_t(*)() noexcept;

// No retry, deadline computation, allocation, printing or system call is added
// here. Clocks and the synthetic/system-call functor are supplied explicitly.
// Successful or failed clocks cannot overwrite the caller's syscall errno.
template<std::size_t Capacity,class Call>
auto measure(Recorder<Capacity>* recorder,const Context& context,
             Clock wall,Clock cpu,Call call) noexcept -> decltype(call()) {
    static_assert(noexcept(call()),"Only a noexcept syscall/fake callable is accepted");
    static_assert(std::is_integral<decltype(call())>::value&&std::is_signed<decltype(call())>::value,
                  "Signed integral syscall result required");
    if(!recorder)return call();
    const int before_errno=errno;
    const uint64_t wb=wall?wall():0,cb=cpu?cpu():0;
    errno=before_errno;
    const auto returned=call();
    const int syscall_errno=errno;
    const uint64_t ce=cpu?cpu():0,we=wall?wall():0;
    const bool valid=wb&&we>=wb&&(!cpu||(cb&&ce>=cb));
    recorder->append(Event{context,wb,we,cb,ce,int64_t(returned),syscall_errno,cpu!=nullptr,valid});
    errno=syscall_errno;
    return returned;
}
}  // namespace proposed_trace
