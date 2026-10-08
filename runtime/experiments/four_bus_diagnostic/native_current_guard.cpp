// Read-only current binding checks. No CAN open, read, write or ownership.
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

extern "C" {
struct CGDevice { uint64_t dev, ino, mode, rdev; };
struct CGAlias { uint64_t dev, ino, mode; int64_t size, mtime_ns, ctime_ns; };
struct CGParent { char path[1024]; uint64_t dev, ino, mode; };
struct CGPort { char path[1024], resolved[1024], link[1024]; CGAlias alias; CGDevice target; };
struct CGPins {
    uint32_t abi, parent_count;
    int32_t boot_fd;
    uint32_t boot_size;
    CGDevice boot_identity;
    unsigned char boot_bytes[128];
    CGParent parents[32];
    CGPort ports[4];
};
struct CGTiming {
    uint64_t started_ns, boot_checked_ns, ancestors_checked_ns, ports_checked_ns, finished_ns;
    uint32_t phase, index;
    int32_t system_errno;
    uint32_t reserved;
};
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
bool terminated(const char* s, size_t n) { return s[0] && std::memchr(s, 0, n); }
CGDevice device(const struct stat& s) {
    return {uint64_t(s.st_dev), uint64_t(s.st_ino), uint64_t(s.st_mode), uint64_t(s.st_rdev)};
}
bool same(const CGDevice& a, const CGDevice& b) {
    return a.dev==b.dev && a.ino==b.ino && a.mode==b.mode && a.rdev==b.rdev;
}
int64_t mtime(const struct stat& s) {
#ifdef __APPLE__
    return int64_t(s.st_mtimespec.tv_sec)*1000000000LL+s.st_mtimespec.tv_nsec;
#else
    return int64_t(s.st_mtim.tv_sec)*1000000000LL+s.st_mtim.tv_nsec;
#endif
}
int64_t ctime(const struct stat& s) {
#ifdef __APPLE__
    return int64_t(s.st_ctimespec.tv_sec)*1000000000LL+s.st_ctimespec.tv_nsec;
#else
    return int64_t(s.st_ctim.tv_sec)*1000000000LL+s.st_ctim.tv_nsec;
#endif
}
bool alias_same(const CGAlias& a, const struct stat& s) {
    return a.dev==uint64_t(s.st_dev) && a.ino==uint64_t(s.st_ino) &&
        a.mode==uint64_t(s.st_mode) && a.size==int64_t(s.st_size) &&
        a.mtime_ns==mtime(s) && a.ctime_ns==ctime(s);
}
bool whitespace(unsigned char b) { return b==32 || (b>=9 && b<=13); }
}

extern "C" uint32_t sdcg_abi() {
    // Exact ctypes layout, including padding, on the supported 64-bit hosts.
    return sizeof(CGDevice)==32 && sizeof(CGAlias)==48 && sizeof(CGParent)==1048 &&
        sizeof(CGPort)==3152 && sizeof(CGPins)==46320 && sizeof(CGTiming)==56 &&
        offsetof(CGPins, boot_identity)==16 && offsetof(CGPins, boot_bytes)==48 &&
        offsetof(CGPins, parents)==176 && offsetof(CGPins, ports)==33712 &&
        offsetof(CGTiming, phase)==40 && offsetof(CGTiming, system_errno)==48 ? 1 : 0;
}

extern "C" int sdcg_check(const CGPins* p, CGTiming* timing, char* error, uint32_t size) {
    if (!timing || !error || size<2) return -1;
    std::memset(timing, 0, sizeof(*timing)); error[0]=0;
    timing->started_ns=now();
    auto fail=[&](const char* message, int number=0) {
        timing->system_errno=number;
        std::snprintf(error,size,"Current guard phase %u index %u: %s%s%s",
            timing->phase,timing->index,message,number?": ":"",number?std::strerror(number):"");
        timing->finished_ns=now(); return -1;
    };
    if (!p) return fail("Missing bounded pins");
    // Never reread caller-owned length/path fields while the GIL is released.
    const CGPins snapshot=*p;
    p=&snapshot;
    if (!sdcg_abi() || !timing->started_ns || p->abi!=1 || p->boot_fd<0 ||
        !p->boot_size || p->boot_size>127 || !p->parent_count || p->parent_count>32)
        return fail("Invalid bounded pins/layout");
    for (uint32_t i=0;i<p->parent_count;++i)
        if (!terminated(p->parents[i].path,1024) || p->parents[i].path[0]!='/' ||
            !S_ISDIR(p->parents[i].mode)) return fail("Invalid parent pins");
    for (const auto& port:p->ports)
        if (!terminated(port.path,1024) || !terminated(port.resolved,1024) ||
            !terminated(port.link,1024) || port.path[0]!='/' || port.resolved[0]!='/' ||
            !S_ISLNK(port.alias.mode) || !S_ISCHR(port.target.mode))
            return fail("Invalid port pins");
    timing->phase=1;
    struct stat info{};
    if (fstat(p->boot_fd,&info)) return fail("boot fstat failed",errno);
    if (!same(device(info),p->boot_identity)) return fail("boot FD identity changed");
    unsigned char bytes[128];
    const ssize_t read=pread(p->boot_fd,bytes,sizeof(bytes),0);
    if (read<0) return fail("boot pread failed",errno);
    size_t begin=0,end=size_t(read);
    while (begin<end && whitespace(bytes[begin])) ++begin;
    while (end>begin && whitespace(bytes[end-1])) --end;
    if (end-begin!=p->boot_size || std::memcmp(bytes+begin,p->boot_bytes,p->boot_size))
        return fail("boot bytes changed");
    timing->boot_checked_ns=now(); timing->phase=2;
    for (uint32_t i=0;i<p->parent_count;++i) {
        timing->index=i; const auto& parent=p->parents[i];
        if (lstat(parent.path,&info)) return fail("ancestor lstat failed",errno);
        if (uint64_t(info.st_dev)!=parent.dev || uint64_t(info.st_ino)!=parent.ino ||
            uint64_t(info.st_mode)!=parent.mode) return fail("ancestor directory identity changed");
    }
    timing->ancestors_checked_ns=now(); timing->phase=3;
    for (uint32_t i=0;i<4;++i) {
        timing->index=i; const auto& port=p->ports[i];
        if (lstat(port.path,&info)) return fail("alias lstat failed",errno);
        if (!alias_same(port.alias,info)) return fail("alias identity/timestamps changed");
        char text[1024]; const ssize_t length=readlink(port.path,text,sizeof(text));
        if (length<0) return fail("alias readlink failed",errno);
        if (size_t(length)!=std::strlen(port.link) || std::memcmp(text,port.link,size_t(length)))
            return fail("alias link text changed");
        if (stat(port.path,&info)) return fail("alias target stat failed",errno);
        if (!same(device(info),port.target)) return fail("alias target device identity changed");
        if (lstat(port.resolved,&info)) return fail("canonical target lstat failed",errno);
        if (!same(device(info),port.target)) return fail("canonical device identity changed");
    }
    timing->ports_checked_ns=now(); timing->phase=4; timing->index=0;
    timing->finished_ns=now();
    if (!timing->finished_ns || timing->started_ns>timing->boot_checked_ns ||
        timing->boot_checked_ns>timing->ancestors_checked_ns ||
        timing->ancestors_checked_ns>timing->ports_checked_ns ||
        timing->ports_checked_ns>timing->finished_ns) return fail("monotonic clock failed");
    return 0;
}
