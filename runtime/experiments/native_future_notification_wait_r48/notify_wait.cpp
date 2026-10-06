// Isolated notification wait. No CAN/IMU/model/device opens, commands or output.
// Only caller-created FIFO read descriptors are accepted. Anonymous provenance
// is a Python fixture property; S_ISFIFO alone also permits a named FIFO.
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

namespace {
uint64_t now_ns() {
    timespec value{};
#ifdef __APPLE__
    // Match CPython mach_absolute_time; CLOCK_MONOTONIC includes host sleep.
    if (clock_gettime(CLOCK_UPTIME_RAW, &value)) return 0;
#else
    if (clock_gettime(CLOCK_MONOTONIC, &value)) return 0;
#endif
    return uint64_t(value.tv_sec) * 1000000000ULL + uint64_t(value.tv_nsec);
}
bool fifo_read_fd(int fd, bool nonblocking) {
    struct stat value{};
    const int flags = fcntl(fd, F_GETFL);
    return fd >= 0 && fd < FD_SETSIZE && flags >= 0 &&
        (flags & O_ACCMODE) == O_RDONLY &&
        (!nonblocking || (flags & O_NONBLOCK)) &&
        !fstat(fd, &value) && S_ISFIFO(value.st_mode);
}
}

extern "C" uint32_t nw_abi() { return 1; }
extern "C" uint64_t nw_now_ns() { return now_ns(); }

// 0=TICK, 1=NOTIFIED, -1=invalid/cancel/I/O/interruption, -2=hard deadline.
// TICK is an actual clock observation, not a backdated planned timestamp.
// NOTIFIED can return before tick; it does not prove any Future is complete.
extern "C" int nw_wait(int notify_fd, int cancel_fd, uint64_t tick_ns,
        uint64_t hard_ns, uint64_t* actual_ns, char* error, uint32_t error_size) {
    if (actual_ns) *actual_ns = 0;
    if (!actual_ns || !error || error_size == 0) return -1;
    error[0] = 0;
    auto fail = [&](int status, const char* message) {
        std::snprintf(error, error_size, "%s", message);
        return status;
    };
    const uint64_t begin = now_ns();
    if (!begin || !tick_ns || !hard_ns || tick_ns > hard_ns ||
            hard_ns > begin + 1000000000ULL || notify_fd == cancel_fd ||
            !fifo_read_fd(notify_fd, true) || !fifo_read_fd(cancel_fd, false))
        return fail(-1, "Invalid bounded notification wait arguments");
    uint32_t interrupts = 0;
    auto cancel_now = [&]() {
        fd_set ready; FD_ZERO(&ready); FD_SET(cancel_fd, &ready);
        timespec zero{};
        const int result = pselect(cancel_fd + 1, &ready, nullptr, nullptr, &zero, nullptr);
        if (result < 0) return errno == EINTR ? 2 : -1;
        return FD_ISSET(cancel_fd, &ready) ? 1 : 0;
    };
    while (true) {
        // Cancellation has priority, even when both fds or deadlines are ready.
        const int cancelled = cancel_now();
        if (cancelled == 1) return fail(-1, "Notification wait cancelled");
        if (cancelled == -1) return fail(-1, "Notification cancellation check failed");
        if (cancelled == 2) {
            if (++interrupts > 32) return fail(-1, "Notification wait interrupted too often");
            continue;
        }
        const uint64_t current = now_ns();
        if (!current) return fail(-1, "Notification monotonic clock failed");
        if (current >= hard_ns) return fail(-2, "Notification hard deadline reached");
        if (current >= tick_ns) {
            const int after = cancel_now();
            if (after == 1) return fail(-1, "Notification wait cancelled at tick");
            if (after != 0) return fail(-1, "Notification tick cancellation check failed");
            const uint64_t actual = now_ns();
            if (!actual || actual < current) return fail(-1, "Noncausal notification tick clock");
            if (actual >= hard_ns) return fail(-2, "Notification hard deadline reached at tick");
            *actual_ns = actual;
            return 0;
        }
        const uint64_t remaining = tick_ns - current;
        timespec timeout{time_t(remaining / 1000000000ULL), long(remaining % 1000000000ULL)};
        fd_set ready; FD_ZERO(&ready);
        FD_SET(notify_fd, &ready); FD_SET(cancel_fd, &ready);
        const int result = pselect((notify_fd > cancel_fd ? notify_fd : cancel_fd) + 1,
                                  &ready, nullptr, nullptr, &timeout, nullptr);
        if (result < 0) {
            if (errno == EINTR && ++interrupts <= 32) continue;
            return fail(-1, errno == EINTR ? "Notification wait interrupted too often" : "Notification pselect failed");
        }
        if (FD_ISSET(cancel_fd, &ready)) return fail(-1, "Notification wait cancelled");
        if (!FD_ISSET(notify_fd, &ready)) continue;
        // Nonblocking, finite drain. Normal scope registers at most three
        // callbacks, each writes one byte once. Saturation is tested separately.
        char bytes[4096];
        for (uint32_t reads = 0; reads < 16; ++reads) {
            const ssize_t count = read(notify_fd, bytes, sizeof(bytes));
            if (count > 0) continue;
            if (count == 0) return fail(-1, "Notification writer closed");
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            if (errno == EINTR && ++interrupts <= 32) { --reads; continue; }
            return fail(-1, errno == EINTR ? "Notification drain interrupted too often" : "Notification drain failed");
        }
        const int after = cancel_now();
        if (after == 1) return fail(-1, "Notification wait cancelled after drain");
        if (after != 0) return fail(-1, "Notification final cancellation check failed");
        const uint64_t actual = now_ns();
        if (!actual || actual < current) return fail(-1, "Noncausal notification clock");
        if (actual >= hard_ns) return fail(-2, "Notification hard deadline reached after drain");
        *actual_ns = actual;
        return 1;
    }
}
