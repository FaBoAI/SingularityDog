// File-only, fixed-shape CPU observation fusion. All arithmetic remains ATen.
// No hardware, transport, scheduling, or output interface is present.
#include <ATen/ATen.h>
#include <torch/library.h>
#include <cmath>

namespace {
void shape(const at::Tensor& value, at::IntArrayRef expected, const char* message) {
  TORCH_CHECK(value.sizes() == expected, message);
}

at::Tensor observation(const at::Tensor& gyro, const at::Tensor& gravity,
    const at::Tensor& command, const at::Tensor& q, const at::Tensor& dq,
    const at::Tensor& exposure_h, const at::Tensor& nominal,
    const at::Tensor& previous_clipped, const at::Tensor& filters,
    const at::Tensor& phase, const at::Tensor& elapsed,
    const at::Tensor& yaw_filters, const at::Tensor& heading_error_rad,
    at::Tensor sensor_q, at::Tensor sensor_up, at::Tensor sensor_yaw_rate) {
  shape(gyro,{1,3},"observation shape");
  shape(gravity,{1,3},"observation shape");
  shape(command,{1,3},"observation shape");
  shape(q,{1,12},"observation shape");
  shape(dq,{1,12},"observation shape");
  shape(exposure_h,{1,12},"observation shape");
  TORCH_CHECK(gyro.isfinite().all().item<bool>() && gravity.isfinite().all().item<bool>() &&
              command.isfinite().all().item<bool>() && q.isfinite().all().item<bool>() &&
              dq.isfinite().all().item<bool>() && exposure_h.isfinite().all().item<bool>(),
              "nonfinite observation");
  TORCH_CHECK(!(exposure_h.lt(0).any().item<bool>() || exposure_h.gt(1).any().item<bool>()),
              "exposure outside [0,1]");
  // Match _sensor_observation's float64 conversion before every expression.
  auto gy=gyro.to(at::kDouble), gr=gravity.to(at::kDouble);
  auto cmd=command.to(at::kDouble), joints=q.to(at::kDouble);
  auto velocity=dq.to(at::kDouble), h=exposure_h.to(at::kDouble);
  auto obs=at::cat({gy*.25,gr,cmd,joints-nominal,velocity*.05,
      previous_clipped,h,filters.reshape({1,9}),
      at::sin(2.*M_PI*phase).unsqueeze(1),
      at::cos(2.*M_PI*phase).unsqueeze(1),
      at::clamp_max(elapsed/2.,1.).unsqueeze(1)},1).to(at::kFloat);
  // The source intentionally computes gravity norm and up in the input dtype.
  auto norm=at::linalg_vector_norm(gravity,2,{1},true);
  TORCH_CHECK(!at::abs(norm-1.).gt(.01).any().item<bool>(),
              "IMU gravity vector is not normalized");
  auto up=(-gravity/norm).to(at::kDouble);
  auto denominator=at::pow(up.select(1,1),2)+at::pow(up.select(1,2),2);
  TORCH_CHECK(!denominator.le(1e-6).any().item<bool>(),
              "IMU heading rate singular near vertical pitch");
  auto yaw_rate=(up.select(1,1)*gy.select(1,1)+up.select(1,2)*gy.select(1,2))/denominator;
  sensor_q.copy_(q.to(at::kDouble));
  sensor_up.copy_(up);
  sensor_yaw_rate.copy_(yaw_rate);
  auto extra=at::cat({yaw_filters,at::sin(heading_error_rad).unsqueeze(1),
                      at::cos(heading_error_rad).unsqueeze(1)},1);
  return at::cat({obs,extra.to(at::kFloat)},1);
}
}

TORCH_LIBRARY(sd_observation_fileonly_r1,m) {
  m.def("observe(Tensor gyro, Tensor gravity, Tensor command, Tensor q, Tensor dq, Tensor exposure_h, Tensor nominal, Tensor previous_clipped, Tensor filters, Tensor phase, Tensor elapsed, Tensor yaw_filters, Tensor heading_error_rad, Tensor(a!) sensor_q, Tensor(b!) sensor_up, Tensor(c!) sensor_yaw_rate) -> Tensor");
}
TORCH_LIBRARY_IMPL(sd_observation_fileonly_r1,CPU,m) {
  m.impl("observe",TORCH_FN(observation));
}
