// File-only active-session syscall simulator. No actual FD/device/native library.
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <mutex>
#include <new>
#include <set>
#include <sys/select.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
static uint64_t sim_ns=1000000000ULL,boot_delay=0,reply_at=0;
static int test_mode=0,wait_calls=0,writes=0,reads=0,mid=0,kind=0,interrupts=0,boot_reads=0;
static uint64_t written_at[16]{};
static unsigned char written_wire[16][17]{};
static bool pending=false,recovering=false;
static const char* expected_boot="11111111-2222-3333-4444-555555555555";
static int sim_clock_gettime(clockid_t,timespec* t){t->tv_sec=sim_ns/1000000000;t->tv_nsec=sim_ns%1000000000;return 0;}
static int sim_fcntl(int,int,...){return O_NONBLOCK;}
static int sim_fstat(int fd,struct stat* st){std::memset(st,0,sizeof(*st));st->st_dev=1;st->st_ino=fd;st->st_rdev=2;return 0;}
static ssize_t sim_pread(int,void* dst,size_t,off_t){++boot_reads;sim_ns+=(test_mode==7?(boot_reads==3?12000000:0):boot_delay);std::memcpy(dst,expected_boot,36);return 36;}
static int sim_pselect(int,fd_set* input,fd_set*,fd_set*,const timespec* timeout,const sigset_t*){
 ++wait_calls;bool serial=input&&FD_ISSET(3,input),cancel=input&&FD_ISSET(4,input);if(input)FD_ZERO(input);
 if(cancel&&!recovering&&(test_mode==3||(test_mode==9&&writes))){FD_SET(4,input);return 1;}
 if(serial&&cancel&&!recovering&&test_mode==6){errno=EINTR;return -1;}
 if(serial&&cancel&&!recovering&&test_mode==2&&writes&&interrupts++==0){sim_ns+=100000;errno=EINTR;return -1;}
 uint64_t duration=uint64_t(timeout->tv_sec)*1000000000+timeout->tv_nsec;
 if(serial&&pending&&(recovering||(test_mode!=1&&test_mode!=2))&&reply_at<=sim_ns+duration){sim_ns=std::max(sim_ns,reply_at);FD_SET(3,input);return 1;}
 sim_ns+=duration;return 0;
}
static ssize_t sim_write(int,const void* src,size_t n){
 auto w=static_cast<const unsigned char*>(src);uint32_t c=(uint32_t(w[2])<<24|uint32_t(w[3])<<16|uint32_t(w[4])<<8|w[5])>>3;
 mid=c&255;kind=c>>24;std::memcpy(written_wire[writes],w,17);written_at[writes++]=sim_ns;pending=true;reply_at=sim_ns+1000;
 return test_mode==4&&!recovering?ssize_t(n-1):ssize_t(n);
}
static ssize_t sim_read(int,void* dst,size_t n){
 if(!pending||n<17){errno=EAGAIN;return -1;}++reads;pending=false;
 unsigned char w[17]={'A','T',0,0,0,0,8,0x7f,0xff,0x7f,0xff,0x7f,0xff,0,250,13,10};
 uint32_t cid=(((2U<<24)|(kind==1?2U<<22:0)|(uint32_t(mid)<<8)|0xfd)<<3)|4;
 for(int i=0;i<4;++i)w[2+i]=(cid>>(24-8*i))&255;std::memcpy(dst,w,17);return 17;
}
#define clock_gettime sim_clock_gettime
#define fcntl sim_fcntl
#define fstat sim_fstat
#define pread sim_pread
#define pselect sim_pselect
#define read sim_read
#define write sim_write
#include SOURCE_FILE
#undef clock_gettime
#undef fcntl
#undef fstat
#undef pread
#undef pselect
#undef read
#undef write
int main(int argc,char** argv){
 test_mode=argc>1?std::atoi(argv[1]):0;
 SDLimits limits{};for(int i=0;i<6;++i){limits.lower[i]=-1;limits.upper[i]=1;limits.kp[i]=3;limits.kd[i]=.15;}
 char error[256]{};void* handle=sda_create(3,4,5,expected_boot,1,&limits,5000000,3,error,sizeof(error));if(!handle)return 2;
 boot_delay=(test_mode==5||test_mode==6||test_mode>=8)?0:2000000;
 unsigned char wires[34]{};
 for(int i=0;i<2;++i){auto w=wires+17*i;w[0]='A';w[1]='T';w[6]=8;w[15]=13;w[16]=10;
  uint32_t c=(4U<<24)|(0xfdU<<8)|uint32_t(i+1);
  if(test_mode==4||test_mode==8||test_mode==10||test_mode==11){c=(1U<<24)|(32767U<<8)|uint32_t(i+1);w[7]=test_mode==8?0xff:0x7f;w[8]=0xff;w[9]=0x7f;w[10]=0xff;if(test_mode==11)c^=1U<<8;}
  c=(c<<3)|4;for(int j=0;j<4;++j)w[2+j]=(c>>(24-8*j))&255;
 }
 SDRecord records[2]{};SDStats stats{};
 uint64_t deadline=sim_ns+((test_mode==1||test_mode==2||test_mode==7)?10000000:40000000);
 int status=sda_exchange(handle,wires,2,test_mode==12?1:0,deadline,records,&stats,error,sizeof(error));
 const uint64_t end_ns=stats.end_ns,exchange_waits=stats.waits;const int exchange_writes=writes,exchange_reads=reads;const uint32_t ambiguous=static_cast<Session*>(handle)->ambiguous;const bool poisoned=static_cast<Session*>(handle)->poisoned;
 const uint64_t first_write=written_at[0],second_write=written_at[1];char tx0[35]{},tx1[35]{};
 for(int i=0;i<17;++i){std::snprintf(tx0+2*i,3,"%02x",written_wire[0][i]);std::snprintf(tx1+2*i,3,"%02x",written_wire[1][i]);}
 char original_error[256]{};std::memcpy(original_error,error,sizeof(error));SDStopResult stop{};int stop_status=-99,retry_status=-99;int extra_retry_writes=0;
 if(status<0){SDRecord retry_records[2]{};SDStats retry_stats{};int before=writes;retry_status=sda_exchange(handle,wires,2,0,sim_ns+40000000,retry_records,&retry_stats,error,sizeof(error));extra_retry_writes=writes-before;}
 if(test_mode==4){recovering=true;boot_delay=0;SDRecord sr[6]{};SDStats ss{};stop_status=sda_emergency_stop(handle,sim_ns+250000000,sr,&ss,&stop,error,sizeof(error));}
 sda_destroy(handle);
 std::printf("{\"status\":%d,\"writes\":%d,\"reads\":%d,\"exchange_waits\":%llu,\"end_ns\":%llu,\"deadline_ns\":%llu,\"first_write_ns\":%llu,\"second_write_ns\":%llu,\"tx0_hex\":\"%s\",\"tx1_hex\":\"%s\",\"poisoned\":%s,\"ambiguous_mask\":%u,\"retry_status\":%d,\"extra_retry_writes\":%d,\"emergency_status\":%d,\"emergency_attempted\":%u,\"emergency_confirmed\":%u,\"emergency_ambiguous\":%u,\"error\":\"%s\"}\n",status,exchange_writes,exchange_reads,(unsigned long long)exchange_waits,(unsigned long long)end_ns,(unsigned long long)deadline,(unsigned long long)first_write,(unsigned long long)second_write,tx0,tx1,poisoned?"true":"false",ambiguous,retry_status,extra_retry_writes,stop_status,stop.attempted_mask,stop.confirmed_mask,stop.ambiguous_mask,original_error);
 return 0;
}
