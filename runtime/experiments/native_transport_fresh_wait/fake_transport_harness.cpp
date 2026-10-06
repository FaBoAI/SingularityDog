// File-only syscall simulator. No real FD, USB, CAN, motor or native library.
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>
static uint64_t sim_ns=1000000000ULL,boot_delay=2000000,reply_at=0;
static int test_mode=0,wait_calls=0,writes=0,reads=0,mid=0,interrupts=0;
static uint64_t written_at[12]{};
static bool pending=false;
static const char* expected_boot="11111111-2222-3333-4444-555555555555";
static int sim_clock_gettime(clockid_t,timespec* t){t->tv_sec=sim_ns/1000000000;t->tv_nsec=sim_ns%1000000000;return 0;}
static int sim_fcntl(int,int,...){return O_NONBLOCK;}
static ssize_t sim_pread(int,void* dst,size_t,off_t){sim_ns+=boot_delay;std::memcpy(dst,expected_boot,36);return 36;}
static int sim_pselect(int,fd_set* input,fd_set*,fd_set*,const timespec* timeout,const sigset_t*){
 ++wait_calls;FD_ZERO(input);
 if(wait_calls>1 && test_mode==2 && interrupts++==0){sim_ns+=100000;errno=EINTR;return -1;}
 if(wait_calls>1 && test_mode==3){FD_SET(4,input);return 1;}
 if(wait_calls>1 && test_mode==6){errno=EINTR;return -1;}
 uint64_t duration=uint64_t(timeout->tv_sec)*1000000000+timeout->tv_nsec;
 if(pending && test_mode!=1 && test_mode!=2 && reply_at<=sim_ns+duration){sim_ns=std::max(sim_ns,reply_at);FD_SET(3,input);return 1;}
 sim_ns+=duration;return 0;
}
static ssize_t sim_write(int,const void* src,size_t n){
 auto w=static_cast<const unsigned char*>(src);uint32_t c=(uint32_t(w[2])<<24|uint32_t(w[3])<<16|uint32_t(w[4])<<8|w[5])>>3;
 mid=c&255;written_at[writes++]=sim_ns;pending=true;reply_at=sim_ns+1000;
 return test_mode==4?ssize_t(n-1):ssize_t(n);
}
static ssize_t sim_read(int,void* dst,size_t n){
 if(!pending||n<17){errno=EAGAIN;return -1;}++reads;pending=false;
 unsigned char w[17]={'A','T',0,0,0,0,8,1,2,3,4,5,6,7,8,13,10};uint32_t cid=((uint32_t(mid)<<8|0xfe)<<3)|4;
 for(int i=0;i<4;++i)w[2+i]=(cid>>(24-8*i))&255;std::memcpy(dst,w,17);return 17;
}
#define clock_gettime sim_clock_gettime
#define fcntl sim_fcntl
#define pread sim_pread
#define pselect sim_pselect
#define read sim_read
#define write sim_write
#include SOURCE_FILE
#undef clock_gettime
#undef fcntl
#undef pread
#undef pselect
#undef read
#undef write
int main(int argc,char** argv){
 test_mode=argc>1?std::atoi(argv[1]):0;
 if(test_mode==5||test_mode==6)boot_delay=0;
 if(test_mode==7)boot_delay=12000000;
 unsigned char wires[34]{};
 for(int i=0;i<2;++i){auto w=wires+17*i;w[0]='A';w[1]='T';w[6]=8;w[15]=13;w[16]=10;
  uint32_t cid=((0xfdU<<8|uint32_t(i+1))<<3)|4;if(test_mode==8)cid|=(1U<<27);
  for(int j=0;j<4;++j)w[2+j]=(cid>>(24-8*j))&255;
 }
 SDRecord records[2]{};SDStats stats{};char error[256]{};
 uint64_t deadline=sim_ns+((test_mode==1||test_mode==2||test_mode==7)?10000000:40000000);
 int status=sd_exchange(3,4,5,expected_boot,wires,2,1,0,5000000,3,deadline,0,records,&stats,error,sizeof(error));
 std::printf("{\"status\":%d,\"writes\":%d,\"reads\":%d,\"waits\":%d,\"end_ns\":%llu,\"deadline_ns\":%llu,\"first_write_ns\":%llu,\"second_write_ns\":%llu,\"error\":\"%s\"}\n",status,writes,reads,wait_calls,(unsigned long long)stats.end_ns,(unsigned long long)deadline,(unsigned long long)written_at[0],(unsigned long long)written_at[1],error);
 return 0;
}
