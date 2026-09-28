// File-only scalar projection experiment. No devices, networking or actuation.
// Derived from hash-pinned SwingCore.step projection block; not a controller.
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace {
constexpr double pi = 3.141592653589793238462643383279502884;
constexpr double lower[3] = {-.5, -.9, -2.2};
constexpr double upper[3] = {.5, 1.2, -.08};
constexpr double origins[4][3] = {{.155,.110,0},{.155,-.110,0},{-.155,.110,0},{-.155,-.110,0}};
constexpr double signs[4] = {1,-1,1,-1};
double safe_lower(int j) { return lower[j] + .02 + 1e-7; }
double safe_upper(int j) { return upper[j] - .02 - 1e-7; }
double wrapped(double x) {
  double z = std::fmod(x + pi, 2*pi);
  if (z < 0) z += 2*pi;
  return z-pi;
}
struct IK { double q[3]; bool domain; bool ok; };
IK feasible_ik(int leg, const double* feet) {
  const double x=feet[0]-origins[leg][0], y=feet[1]-origins[leg][1], z=feet[2];
  const double rad=y*y+z*z-.064*.064;
  const double cosine=(x*x+rad-2*.12*.12)/(2*.12*.12);
  IK r{};
  r.domain=rad>0 && cosine>=-1 && cosine<=1 &&
      std::isfinite(feet[0]) && std::isfinite(feet[1]) && std::isfinite(feet[2]);
  const double zs=-std::sqrt(std::max(rad,0.));
  const double calf=-std::acos(std::clamp(cosine,-1.,1.));
  r.q[0]=wrapped(std::atan2(z,y)-std::atan2(zs,.064*signs[leg]));
  r.q[1]=wrapped(std::atan2(-x,-zs)-std::atan2(.12*std::sin(calf),.12+.12*std::cos(calf)));
  r.q[2]=calf;
  r.ok=r.domain;
  for(int j=0;j<3;++j) r.ok=r.ok && r.q[j]>=safe_lower(j)-2e-12 && r.q[j]<=safe_upper(j)+2e-12;
  return r;
}
void fk(int leg, const double* q, double* out) {
  const double a=q[0],b=q[1],c=q[2];
  const double x=-.12*(std::sin(b)+std::sin(b+c));
  const double z=-.12*(std::cos(b)+std::cos(b+c));
  const double y=.064*signs[leg];
  out[0]=x+origins[leg][0];
  out[1]=std::cos(a)*y-std::sin(a)*z+origins[leg][1];
  out[2]=std::sin(a)*y+std::cos(a)*z;
}
bool finite(const double* a, int n) {for(int i=0;i<n;++i) if(!std::isfinite(a[i])) return false;return true;}
}

extern "C" {
struct ProjectionResult {
  double solved[12], endpoint_q[12], fraction[4], first_invalid[4], hi[4], applied[4], roundtrip[12], joint_margin[12];
  int32_t reason[4], endpoint_domain[4], endpoint_ok[4], path_rejected[4];
};

// Caller must supply fixed-size arrays. Status0=valid; nonzero never usable.
//1 input contract,2 margin,3 unexpected physical clip,4 FK closure.
int projection_file_only(const double* pre, const double* start, const double* up,
    const double* requested, const double* alpha, const double* uncapped, ProjectionResult* out) {
  if(!pre||!start||!up||!requested||!alpha||!uncapped||!out) return 1;
  *out=ProjectionResult{};
  if(!finite(pre,12)||!finite(start,12)||!finite(up,3)||!finite(requested,4)||!finite(alpha,4)||!finite(uncapped,4)) return 1;
  // Preserve the supplied sensor_up exactly. The original observation path
  // validates and normalizes float32 gravity before storing float64 sensor_up;
  // imposing tighter normalization here would reject valid source rounding.
  for(int leg=0;leg<4;++leg) {
    if(requested[leg]<0 || requested[leg]>.060+1e-12 || alpha[leg]<0 || alpha[leg]>1 || uncapped[leg]<requested[leg]-1e-12) return 1;
    out->hi[leg]=1.;out->first_invalid[leg]=1.;
    for(int j=0;j<3;++j) out->solved[3*leg+j]=pre[3*leg+j];
  }
  auto evaluate=[&](int leg,double fraction) {
    double point[3];
    for(int j=0;j<3;++j) point[j]=start[3*leg+j]+(requested[leg]*fraction)*up[j];
    return feasible_ik(leg,point);
  };
  bool active[4]={true,true,true,true};
  for(int leg=0;leg<4;++leg) {
    IK e=evaluate(leg,1.);
    out->endpoint_domain[leg]=e.domain;out->endpoint_ok[leg]=e.ok;
    for(int j=0;j<3;++j) out->endpoint_q[3*leg+j]=e.q[j];
  }
  for(int k=1;k<=16;++k) {
    const double fraction=double(k)/16.;
    for(int leg=0;leg<4;++leg) {
      IK q=evaluate(leg,fraction);
      bool reject=active[leg]&&!q.ok,accept=active[leg]&&q.ok;
      if(reject) out->hi[leg]=out->first_invalid[leg]=fraction;
      if(accept) {
        out->fraction[leg]=fraction;
        for(int j=0;j<3;++j) out->solved[3*leg+j]=q.q[j];
      }
      active[leg]=accept;
    }
  }
  if(!(active[0]&&active[1]&&active[2]&&active[3])) {
    for(int k=0;k<14;++k) for(int leg=0;leg<4;++leg) {
      const double mid=(out->fraction[leg]+out->hi[leg])*.5;
      IK q=evaluate(leg,mid);
      if(q.ok) {
        out->fraction[leg]=mid;
        for(int j=0;j<3;++j) out->solved[3*leg+j]=q.q[j];
      } else out->hi[leg]=mid;
    }
  }
  for(int leg=0;leg<4;++leg) {
    out->applied[leg]=requested[leg]*out->fraction[leg];
    out->path_rejected[leg]=!active[leg];
    out->reason[leg]=(alpha[leg]<1.-1e-12)+2*(!out->endpoint_domain[leg])+
        4*((out->endpoint_domain[leg]&&!out->endpoint_ok[leg])||!active[leg])+8*(uncapped[leg]>requested[leg]+1e-12);
    for(int j=0;j<3;++j) {
      double q=out->solved[3*leg+j];
      if(q<lower[j]+.02-1e-10||q>upper[j]-.02+1e-10) return 2;
      if(std::abs(std::clamp(q,lower[j],upper[j])-q)>1e-12) return 3;
      out->joint_margin[3*leg+j]=std::min(q-lower[j],upper[j]-q);
    }
    fk(leg,&out->solved[3*leg],&out->roundtrip[3*leg]);
    for(int j=0;j<3;++j) if(std::abs(out->roundtrip[3*leg+j]-(start[3*leg+j]+out->applied[leg]*up[j]))>1e-9) return 4;
  }
  return 0;
}

// Separate file-test entry for finite geometric domain/margin boundary cases.
int feasible_ik_file_only(int leg,const double* feet,double* q,int32_t* masks) {
  if(leg<0||leg>=4||!feet||!q||!masks||!finite(feet,3)) return 1;
  IK r=feasible_ik(leg,feet);
  for(int j=0;j<3;++j) q[j]=r.q[j];
  masks[0]=r.domain;masks[1]=r.ok;return 0;
}
}
