// Experimental fixed-shape C++/ATen fusion of SwingCore._step_inputs.
// The numerical kernels remain ATen's; this removes TorchScript dispatch and
// temporary dictionary construction around the recurrent filter/reference path.
// No transport, hardware, thread, clock, or output API is present.
#include <ATen/ATen.h>
#include <torch/library.h>
#include <cmath>
#include <vector>

namespace {
void check(const at::Tensor& x, at::IntArrayRef shape, at::ScalarType dtype, const char* name) {
  TORCH_CHECK(x.device().is_cpu() && x.scalar_type() == dtype && x.layout() == at::kStrided &&
              x.sizes() == shape && x.is_contiguous() && !x.is_neg() && !x.is_conj(),
              name, " shape/dtype/layout differs");
}

std::vector<at::Tensor> step(const at::Tensor& raw, const at::Tensor& command,
    at::Tensor phase, at::Tensor elapsed, at::Tensor filters, at::Tensor yaw_filters,
    at::Tensor heading_error, const at::Tensor& sensor_yaw_rate,
    at::Tensor previous_clipped, const at::Tensor& offsets,
    const at::Tensor& signs, const at::Tensor& anchor,
    const at::Tensor& origins, const at::Tensor& lower,
    const at::Tensor& upper) {
  check(raw,{1,12},at::kFloat,"raw"); check(command,{1,3},at::kFloat,"command");
  check(phase,{1},at::kDouble,"phase"); check(elapsed,{1},at::kDouble,"elapsed");
  check(filters,{1,3,3},at::kDouble,"filters");
  check(yaw_filters,{1,3},at::kDouble,"yaw_filters");
  check(heading_error,{1},at::kDouble,"heading_error");
  check(sensor_yaw_rate,{1},at::kDouble,"sensor_yaw_rate");
  check(previous_clipped,{1,12},at::kDouble,"previous_clipped");
  check(offsets,{4},at::kDouble,"offsets"); check(signs,{4},at::kDouble,"signs");
  check(anchor,{4,3},at::kDouble,"anchor"); check(origins,{4,3},at::kDouble,"origins");
  check(lower,{12},at::kDouble,"lower"); check(upper,{12},at::kDouble,"upper");
  TORCH_CHECK(raw.isfinite().all().item<bool>() && command.isfinite().all().item<bool>(),
              "nonfinite controller input");
  auto action = raw.to(at::kDouble);
  auto cmd = command.to(at::kDouble);
  const auto* cv = cmd.const_data_ptr<double>();
  const bool translation = std::hypot(cv[0],cv[1]) > 1e-9;
  const bool turning = std::abs(cv[2]) > 1e-9;
  TORCH_CHECK(!(std::abs(cv[2]) > .25+1e-8 || cv[0] > .46+3e-8 ||
                cv[0] < -.12-1e-8 || std::abs(cv[1]) > .12+1e-8 ||
                (std::abs(cv[0]) > 1e-9 && std::abs(cv[1]) > 1e-9) ||
                (translation && turning)),
              "Outside registered L11 cardinal, pure-yaw or stop command domain");

  auto cmd_z = cmd.select(1,2);
  auto next_error = heading_error + .02*(cmd_z-sensor_yaw_rate);
  heading_error.copy_(at::atan2(at::sin(next_error),at::cos(next_error)));
  auto correction = translation && !turning
      ? at::clamp(.8*heading_error,-.12,.12)
      : at::zeros_like(next_error);
  auto effective_yaw = cmd_z + correction;
  auto active = at::full({1},translation || turning,
                         at::TensorOptions().device(at::kCPU).dtype(at::kBool)).to(at::kDouble);
  auto desired = at::cat({cmd.slice(1,0,2),active.unsqueeze(1)},1);
  const double b=.02/.15, a=0.8751733190429475;
  auto delta = filters-desired.unsqueeze(1);
  auto d0=delta.select(1,0), d1=delta.select(1,1), d2=delta.select(1,2);
  auto f0=desired+a*d0;
  auto f1=desired+a*(d1+b*d0);
  auto f2=desired+a*(d2+b*d1+.5*b*b*d0);
  filters.copy_(at::stack({f0,f1,f2},1));
  auto yaw_delta=yaw_filters-effective_yaw.unsqueeze(1);
  auto yd0=yaw_delta.select(1,0), yd1=yaw_delta.select(1,1), yd2=yaw_delta.select(1,2);
  auto y0=effective_yaw+a*yd0;
  auto y1=effective_yaw+a*(yd1+b*yd0);
  auto y2=effective_yaw+a*(yd2+b*yd1+.5*b*b*yd0);
  yaw_filters.copy_(at::stack({y0,y1,y2},1));
  phase.copy_(at::remainder(phase+.02/.56,1.));
  elapsed.add_(.02);
  auto progress=at::clamp_max(elapsed/2.,1.);
  auto boot=at::pow(progress,3)*(10.-15.*progress+6.*at::pow(progress,2));
  auto u=at::remainder(phase.unsqueeze(1)+offsets,1.);
  auto s=at::clamp((u-.60)/(1.-.60),0.,1.);
  auto smooth=at::pow(s,3)*(10.-15.*s+6.*at::pow(s,2));
  auto travel=.56*(.60/2.-u+smooth);
  auto anchor_y=anchor.select(1,1).unsqueeze(0)+signs.unsqueeze(0)*.02*boot.unsqueeze(1);
  auto yaw=yaw_filters.select(1,2).unsqueeze(1);
  auto vx=filters.select(1,2).select(1,0).unsqueeze(1)-yaw*anchor_y;
  auto vy=filters.select(1,2).select(1,1).unsqueeze(1)+yaw*anchor.select(1,0).unsqueeze(0);
  auto foot_velocity=at::stack({vx,vy},2);
  auto xy=anchor.slice(1,0,2).unsqueeze(0)+boot.unsqueeze(1).unsqueeze(2)*travel.unsqueeze(2)*foot_velocity;
  auto clearance=boot.unsqueeze(1)*filters.select(1,2).select(1,2).unsqueeze(1)*64.*.020*
                 at::pow(s,3)*at::pow(1.-s,3);
  auto z=anchor.select(1,2).unsqueeze(0)+clearance;
  auto feet=at::cat({xy,z.unsqueeze(2)},2);
  auto r=feet-origins.unsqueeze(0);
  auto x=r.select(2,0), yy=r.select(2,1), zz=r.select(2,2);
  auto rad=yy*yy+zz*zz-std::pow(.064,2);
  auto cosine=(x*x+rad-2.*std::pow(.12,2))/(2.*std::pow(.12,2));
  TORCH_CHECK(!(rad.le(0).any().item<bool>() || cosine.lt(-1.).any().item<bool>() ||
                cosine.gt(1.).any().item<bool>()),
              "Reference IK failed; no hole filling or simulator state override");
  auto zs=-at::sqrt(rad), calf=-at::acos(cosine);
  auto hip=at::remainder(at::atan2(zz,yy)-at::atan2(zs,.064*signs)+M_PI,2.*M_PI)-M_PI;
  auto thigh=at::remainder(at::atan2(-x,-zs)-
      at::atan2(.12*at::sin(calf),.12+.12*at::cos(calf))+M_PI,2.*M_PI)-M_PI;
  auto qref=at::stack({hip,thigh,calf},2).reshape({1,12});
  TORCH_CHECK(!(qref.lt(lower-1e-12).any().item<bool>() ||
                qref.gt(upper+1e-12).any().item<bool>()),
              "Reference outside physical q");
  auto clipped=0.*previous_clipped+(1.-0.)*at::clamp(action,-1.,1.);
  previous_clipped.copy_(clipped);
  // The Python source rounds these three values to output_anchor's float32.
  return {clipped.to(at::kFloat),feet.to(at::kFloat),clearance.to(at::kFloat),boot};
}
}

TORCH_LIBRARY(sd_step_fileonly_r1,m) {
  m.def("step(Tensor raw, Tensor command, Tensor(a!) phase, Tensor(b!) elapsed, Tensor(c!) filters, Tensor(d!) yaw_filters, Tensor(e!) heading_error, Tensor sensor_yaw_rate, Tensor(f!) previous_clipped, Tensor offsets, Tensor signs, Tensor anchor, Tensor origins, Tensor lower, Tensor upper) -> Tensor[]");
}
TORCH_LIBRARY_IMPL(sd_step_fileonly_r1,CPU,m) {
  m.impl("step",TORCH_FN(step));
}
