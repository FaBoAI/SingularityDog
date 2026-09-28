// Experimental scalar, fixed-shape version of SwingCore._step_inputs.
// File-only candidate: reuse the TorchScript operator schema in a fresh process
// so a pinned model can compare this library with the validated ATen library.
// The two libraries cannot be registered in the same process. No device I/O.
#include <ATen/ATen.h>
#include <torch/library.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace {

void check(const at::Tensor& x, at::IntArrayRef shape, at::ScalarType dtype,
           const char* name) {
  TORCH_CHECK(x.device().is_cpu() && x.scalar_type() == dtype &&
              x.layout() == at::kStrided && x.sizes() == shape &&
              x.is_contiguous() && !x.is_neg() && !x.is_conj(),
              name, " shape/dtype/layout differs");
}

double remainder_one(double value) {
  double result = std::fmod(value, 1.0);
  return result < 0.0 ? result + 1.0 : result;
}

double remainder_tau(double value) {
  constexpr double tau = 2.0 * M_PI;
  double result = std::fmod(value, tau);
  return result < 0.0 ? result + tau : result;
}

std::vector<at::Tensor> step_scalar(const at::Tensor& raw,
    const at::Tensor& command, at::Tensor phase, at::Tensor elapsed,
    at::Tensor filters, at::Tensor yaw_filters, at::Tensor heading_error,
    const at::Tensor& sensor_yaw_rate, at::Tensor previous_clipped,
    const at::Tensor& offsets, const at::Tensor& signs,
    const at::Tensor& anchor, const at::Tensor& origins,
    const at::Tensor& lower, const at::Tensor& upper) {
  check(raw, {1, 12}, at::kFloat, "raw");
  check(command, {1, 3}, at::kFloat, "command");
  check(phase, {1}, at::kDouble, "phase");
  check(elapsed, {1}, at::kDouble, "elapsed");
  check(filters, {1, 3, 3}, at::kDouble, "filters");
  check(yaw_filters, {1, 3}, at::kDouble, "yaw_filters");
  check(heading_error, {1}, at::kDouble, "heading_error");
  check(sensor_yaw_rate, {1}, at::kDouble, "sensor_yaw_rate");
  check(previous_clipped, {1, 12}, at::kDouble, "previous_clipped");
  check(offsets, {4}, at::kDouble, "offsets");
  check(signs, {4}, at::kDouble, "signs");
  check(anchor, {4, 3}, at::kDouble, "anchor");
  check(origins, {4, 3}, at::kDouble, "origins");
  check(lower, {12}, at::kDouble, "lower");
  check(upper, {12}, at::kDouble, "upper");

  const float* rv = raw.const_data_ptr<float>();
  const float* cv = command.const_data_ptr<float>();
  for (int i = 0; i < 12; ++i) {
    TORCH_CHECK(std::isfinite(rv[i]), "nonfinite controller input");
  }
  for (int i = 0; i < 3; ++i) {
    TORCH_CHECK(std::isfinite(cv[i]), "nonfinite controller input");
  }
  const double cmd[3] = {static_cast<double>(cv[0]),
                         static_cast<double>(cv[1]),
                         static_cast<double>(cv[2])};
  const bool translation = std::hypot(cmd[0], cmd[1]) > 1e-9;
  const bool turning = std::abs(cmd[2]) > 1e-9;
  TORCH_CHECK(!(std::abs(cmd[2]) > .25 + 1e-8 ||
                cmd[0] > .46 + 3e-8 || cmd[0] < -.12 - 1e-8 ||
                std::abs(cmd[1]) > .12 + 1e-8 ||
                (std::abs(cmd[0]) > 1e-9 && std::abs(cmd[1]) > 1e-9) ||
                (translation && turning)),
              "Outside registered L11 cardinal, pure-yaw or stop command domain");

  // Keep this one-element ATen boundary exact. Small libm differences in the
  // wrapped heading accumulate across recurrent calls and change later state.
  const auto cmd_tensor = command.to(at::kDouble);
  const auto cmd_z = cmd_tensor.select(1, 2);
  const auto next_error = heading_error + .02 * (cmd_z - sensor_yaw_rate);
  heading_error.copy_(at::atan2(at::sin(next_error), at::cos(next_error)));
  const double* error = heading_error.const_data_ptr<double>();
  const double correction = translation && !turning
      ? std::clamp(.8 * error[0], -.12, .12) : 0.0;
  const double effective_yaw = cmd[2] + correction;
  const double desired[3] = {cmd[0], cmd[1], translation || turning ? 1.0 : 0.0};
  constexpr double b = .02 / .15;
  constexpr double a = 0.8751733190429475;
  double* f = filters.data_ptr<double>();
  double updated[9];
  for (int j = 0; j < 3; ++j) {
    const double d0 = f[j] - desired[j];
    const double d1 = f[3 + j] - desired[j];
    const double d2 = f[6 + j] - desired[j];
    updated[j] = desired[j] + a * d0;
    updated[3 + j] = desired[j] + a * (d1 + b * d0);
    updated[6 + j] = desired[j] + a * (d2 + b * d1 + .5 * b * b * d0);
  }
  std::copy(updated, updated + 9, f);
  double* yf = yaw_filters.data_ptr<double>();
  const double yd0 = yf[0] - effective_yaw;
  const double yd1 = yf[1] - effective_yaw;
  const double yd2 = yf[2] - effective_yaw;
  const double y0 = effective_yaw + a * yd0;
  const double y1 = effective_yaw + a * (yd1 + b * yd0);
  const double y2 = effective_yaw + a * (yd2 + b * yd1 + .5 * b * b * yd0);
  yf[0] = y0; yf[1] = y1; yf[2] = y2;

  double* phase_value = phase.data_ptr<double>();
  double* elapsed_value = elapsed.data_ptr<double>();
  phase_value[0] = remainder_one(phase_value[0] + .02 / .56);
  elapsed_value[0] += .02;
  const double progress = std::min(elapsed_value[0] / 2.0, 1.0);
  // ATen's integer tensor powers 2 and 3 use repeated multiplication. libm
  // pow() differs by one ULP for some finite inputs (including cycle 3).
  const double progress2 = progress * progress;
  const double progress3 = progress2 * progress;
  const double boot = progress3 *
      (10.0 - 15.0 * progress + 6.0 * progress2);

  const double* off = offsets.const_data_ptr<double>();
  const double* sign = signs.const_data_ptr<double>();
  const double* anc = anchor.const_data_ptr<double>();
  const double* origin = origins.const_data_ptr<double>();
  const double* lo = lower.const_data_ptr<double>();
  const double* hi = upper.const_data_ptr<double>();
  double feet[12];
  double clearance[4];
  double relative[12];
  double radius[4];
  double cosine[4];
  for (int leg = 0; leg < 4; ++leg) {
    const double u = remainder_one(phase_value[0] + off[leg]);
    const double s = std::clamp((u - .60) / (1.0 - .60), 0.0, 1.0);
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double smooth = s3 * (10.0 - 15.0 * s + 6.0 * s2);
    const double travel = .56 * (.60 / 2.0 - u + smooth);
    const double anchor_y = anc[leg * 3 + 1] + sign[leg] * .02 * boot;
    const double vx = f[6] - yf[2] * anchor_y;
    const double vy = f[7] + yf[2] * anc[leg * 3];
    feet[leg * 3] = anc[leg * 3] + boot * travel * vx;
    feet[leg * 3 + 1] = anc[leg * 3 + 1] + boot * travel * vy;
    const double stance_tail = 1.0 - s;
    const double stance_tail3 = stance_tail * stance_tail * stance_tail;
    clearance[leg] = boot * f[8] * 64.0 * .020 * s3 * stance_tail3;
    feet[leg * 3 + 2] = anc[leg * 3 + 2] + clearance[leg];
    for (int axis = 0; axis < 3; ++axis) {
      relative[leg * 3 + axis] = feet[leg * 3 + axis] - origin[leg * 3 + axis];
    }
    const double x = relative[leg * 3];
    const double y = relative[leg * 3 + 1];
    const double z = relative[leg * 3 + 2];
    radius[leg] = y * y + z * z - std::pow(.064, 2.0);
    cosine[leg] = (x * x + radius[leg] - 2.0 * std::pow(.12, 2.0)) /
                  (2.0 * std::pow(.12, 2.0));
  }
  for (int leg = 0; leg < 4; ++leg) {
    TORCH_CHECK(radius[leg] > 0.0 && cosine[leg] >= -1.0 && cosine[leg] <= 1.0,
                "Reference IK failed; no hole filling or simulator state override");
  }
  for (int leg = 0; leg < 4; ++leg) {
    const double x = relative[leg * 3];
    const double y = relative[leg * 3 + 1];
    const double z = relative[leg * 3 + 2];
    const double zs = -std::sqrt(radius[leg]);
    const double calf = -std::acos(cosine[leg]);
    const double hip = remainder_tau(std::atan2(z, y) -
        std::atan2(zs, .064 * sign[leg]) + M_PI) - M_PI;
    const double thigh = remainder_tau(std::atan2(-x, -zs) -
        std::atan2(.12 * std::sin(calf), .12 + .12 * std::cos(calf)) + M_PI) - M_PI;
    const double qref[3] = {hip, thigh, calf};
    for (int axis = 0; axis < 3; ++axis) {
      const int joint = leg * 3 + axis;
      TORCH_CHECK(!(qref[axis] < lo[joint] - 1e-12 ||
                    qref[axis] > hi[joint] + 1e-12),
                  "Reference outside physical q");
    }
  }

  const auto float_options = at::TensorOptions().device(at::kCPU).dtype(at::kFloat);
  const auto double_options = at::TensorOptions().device(at::kCPU).dtype(at::kDouble);
  auto clipped_output = at::empty({1, 12}, float_options);
  auto feet_output = at::empty({1, 4, 3}, float_options);
  auto clearance_output = at::empty({1, 4}, float_options);
  auto boot_output = at::empty({1}, double_options);
  double* previous = previous_clipped.data_ptr<double>();
  float* clipped = clipped_output.data_ptr<float>();
  float* out_feet = feet_output.data_ptr<float>();
  float* out_clearance = clearance_output.data_ptr<float>();
  for (int i = 0; i < 12; ++i) {
    const double action = static_cast<double>(rv[i]);
    const double updated_clip = 0.0 * previous[i] +
        (1.0 - 0.0) * std::clamp(action, -1.0, 1.0);
    previous[i] = updated_clip;
    clipped[i] = static_cast<float>(updated_clip);
    out_feet[i] = static_cast<float>(feet[i]);
  }
  for (int i = 0; i < 4; ++i) {
    out_clearance[i] = static_cast<float>(clearance[i]);
  }
  boot_output.data_ptr<double>()[0] = boot;
  return {clipped_output, feet_output, clearance_output, boot_output};
}

}  // namespace

TORCH_LIBRARY(sd_step_fileonly_r1, m) {
  m.def("step(Tensor raw, Tensor command, Tensor(a!) phase, Tensor(b!) elapsed, Tensor(c!) filters, Tensor(d!) yaw_filters, Tensor(e!) heading_error, Tensor sensor_yaw_rate, Tensor(f!) previous_clipped, Tensor offsets, Tensor signs, Tensor anchor, Tensor origins, Tensor lower, Tensor upper) -> Tensor[]");
}
TORCH_LIBRARY_IMPL(sd_step_fileonly_r1, CPU, m) {
  m.impl("step", TORCH_FN(step_scalar));
}
