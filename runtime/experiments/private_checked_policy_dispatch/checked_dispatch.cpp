// Offline experiment only. Original policy/actor/controller/state remain in
// TorchScript. No sensor, clock, transport, approval or mutable global state.
#include <ATen/ATen.h>
#include <torch/library.h>
#include <cmath>
#include <vector>

namespace {
using T = at::Tensor;
void row(const T& value, int count, const char* marker) {
  TORCH_CHECK(value.defined() && value.device().is_cpu() &&
      value.scalar_type() == at::kFloat && value.layout() == at::kStrided &&
      value.sizes().equals({1, count}), marker);
}
T readable(const T& value) {
  // Tensor.tolist/aminmax honor negative and noncontiguous views. The owned
  // temporary is read only and never aliases or mutates model state.
  return value.resolve_neg().contiguous();
}
void finite(const T& value, int count, const char* marker) {
  const T copy = readable(value);
  const float* data = copy.const_data_ptr<float>();
  for (int i=0; i<count; ++i) TORCH_CHECK(std::isfinite(data[i]), marker);
}
std::vector<double> checked_targets(const T& target, const T& actor, const T& observation) {
  // Match the original helper/error precedence exactly: target ABI/finite,
  // actor ABI/finite, observation ABI/finite, then learned target range.
  row(target, 12, "PRIVATE_LIVE_CHECKED_TARGET_ABI_R1");
  const T values = readable(target);
  const float* data = values.const_data_ptr<float>();
  for (int i=0; i<12; ++i)
    TORCH_CHECK(std::isfinite(data[i]), "PRIVATE_LIVE_CHECKED_TARGET_FINITE_R1");
  row(actor, 12, "PRIVATE_LIVE_CHECKED_ACTOR_ABI_R1");
  finite(actor, 12, "PRIVATE_LIVE_CHECKED_ACTOR_FINITE_R1");
  row(observation, 74, "PRIVATE_LIVE_CHECKED_OBSERVATION_ABI_R1");
  finite(observation, 74, "PRIVATE_LIVE_CHECKED_OBSERVATION_FINITE_R1");
  constexpr double lower[3] = {-0.5, -0.8999999761581421, -2.200000047683716};
  constexpr double upper[3] = { 0.5,  1.2000000476837158, -0.07999999821186066};
  for (int i=0; i<12; ++i) {
    const double q = static_cast<double>(data[i]);
    TORCH_CHECK(lower[i%3] <= q && q <= upper[i%3],
        "PRIVATE_LIVE_CHECKED_TARGET_RANGE_R1");
  }
  constexpr int can[12] = {6,5,4,3,2,1,12,11,10,9,8,7};
  std::vector<double> result(12);
  for (int i=0; i<12; ++i) result[can[i]-1] = static_cast<double>(data[i]);
  return result; // Owned float list; including the original signed-zero bits.
}
}

TORCH_LIBRARY(sd_live_checked_dispatch_private_r1, m) {
  m.def("checked_targets(Tensor target, Tensor actor, Tensor observation) -> float[]");
}
TORCH_LIBRARY_IMPL(sd_live_checked_dispatch_private_r1, CompositeExplicitAutograd, m) {
  // A composite registration lets wrong-device/layout inputs reach the exact
  // admission checks rather than an unrelated missing CPU dispatch fallback.
  m.impl("checked_targets", TORCH_FN(checked_targets));
}
