// File-only port of the CURRENT scalar core's step_target tail, using ATen.
// Reuse the already verified scalar/projection ops. No transport, clock, model
// weights, global mutable state, or motor interface is present.
#include <ATen/ATen.h>
#include <ATen/core/dispatch/Dispatcher.h>
#include <torch/library.h>
#include <vector>

namespace {
using T = at::Tensor;
using ScalarStep = std::vector<T>(const T&, const T&, T, T, T, T, T,
                                const T&, T, const T&, const T&, const T&,
                                const T&, const T&, const T&);
using Projection = std::vector<T>(const T&, const T&, const T&,
                                 const T&, const T&, const T&);
constexpr double pi = 3.141592653589793;
constexpr double tau = 6.283185307179586;

T fk(const T& q, const T& phase, const T& signs_expanded,
     const T& origins_row) {
  auto v = q.to(phase).reshape({1, 4, 3});
  auto a = v.select(2, 0), b = v.select(2, 1), c = v.select(2, 2);
  // Reuse immutable ATen results only; keep every multiply/add/subtract order.
  auto bc = b + c;
  auto x = -.12 * (at::sin(b) + at::sin(bc));
  auto z = -.12 * (at::cos(b) + at::cos(bc));
  auto y = .064 * signs_expanded;
  auto cos_a = at::cos(a), sin_a = at::sin(a);
  return at::stack({x, cos_a * y - sin_a * z,
                      sin_a * y + cos_a * z}, 2) + origins_row;
}

T ik(const T& feet, const T& signs, const T& origins_row) {
  auto r = feet - origins_row;
  auto x = r.select(2, 0), y = r.select(2, 1), z = r.select(2, 2);
  auto rad = y * y + z * z - .004096;
  auto cosine = (x * x + rad - .0288) / .0288;
  TORCH_CHECK(!(rad.le(0).any() | cosine.lt(-1).any() |
                cosine.gt(1).any()).item<bool>(),
              "L07 swing target outside IK domain");
  auto zs = -at::sqrt(rad), calf = -at::acos(cosine);
  auto hip = at::remainder(at::atan2(z, y) -
      at::atan2(zs, .064 * signs) + pi, tau) - pi;
  auto thigh = at::remainder(at::atan2(-x, -zs) -
      at::atan2(.12 * at::sin(calf), .12 + .12 * at::cos(calf)) + pi,
      tau) - pi;
  return at::stack({hip, thigh, calf}, 2).reshape({1, 12});
}

T target(const T& raw, const T& command, T phase, T elapsed,
    T filters, T yaw_filters, T heading_error, const T& sensor_yaw_rate,
    T previous_clipped, const T& offsets, const T& signs, const T& anchor,
    const T& origins, const T& lower, const T& upper, const T& scale,
    const T& safe_lower, const T& safe_upper, const T& sensor_q,
    const T& sensor_up, const T& output_anchor, const T& signs_row,
    const T& signs_expanded, const T& origins_row, const T& up_column) {
  // Same native scalar implementation and mutation/error order as baseline.
  static const auto scalar = c10::Dispatcher::singleton()
      .findSchemaOrThrow("sd_step_fileonly_r1::step", "").typed<ScalarStep>();
  static const auto projection = c10::Dispatcher::singleton()
      .findSchemaOrThrow("sd_projection_fileonly_r1::project", "").typed<Projection>();
  auto d = scalar.call(raw, command, phase, elapsed, filters, yaw_filters,
      heading_error, sensor_yaw_rate, previous_clipped, offsets, signs, anchor,
      origins, lower, upper);
  TORCH_CHECK(d.size() == 4, "Pinned scalar intermediate count differs");
  // Do not bypass the scalar step's FLOAT32 intermediate rounding boundary.
  auto bump = d[2].to(phase) / .020;
  auto feet = d[1].to(phase).clone();
  feet.select(2, 2).add_(bump * (.035 - .020));
  feet.select(2, 1).add_(signs_row * .02 * d[3].to(phase).unsqueeze(1));
  auto ref = ik(feet, signs, origins_row);
  TORCH_CHECK(!((ref.lt(safe_lower) | ref.gt(safe_upper)).any()).item<bool>(),
              "L13 reference itself violates registered 0.02 rad joint margins");
  auto residual = d[3].to(phase).unsqueeze(1) * scale * d[0].to(phase);
  auto unbounded = ref + residual;
  auto safe_den = at::where(at::abs(residual).gt(1e-15), residual,
                            at::ones_like(residual));
  auto allowed = at::where(residual.gt(1e-15), (safe_upper - ref) / safe_den,
      at::where(residual.lt(-1e-15), (safe_lower - ref) / safe_den,
                at::ones_like(residual)));
  auto alpha = at::clamp(std::get<0>(allowed.reshape({1, 4, 3}).min(2)), 0., 1.);
  auto pre = ref + (residual.reshape({1, 4, 3}) * alpha.unsqueeze(2)).reshape({1, 12});
  auto start_feet = fk(pre, phase, signs_expanded, origins_row);
  auto measured_feet = fk(sensor_q, phase, signs_expanded, origins_row);
  auto plane = std::get<0>((measured_feet * up_column).sum(2).min(1));
  auto height = (start_feet * up_column).sum(2);
  auto gap = at::clamp_min(plane.unsqueeze(1) + .035 - height, 0.);
  auto requested = at::zeros_like(bump), uncapped = at::zeros_like(bump);
  // Exact pinned OPTIONS.use_plane=True; factory rejects other options.
  uncapped = bump * gap;
  requested = bump * at::clamp_max(gap, .060);
  auto native = projection.call(pre.contiguous(), start_feet.contiguous(),
      sensor_up.contiguous(), requested.contiguous(), alpha.contiguous(),
      uncapped.contiguous());
  TORCH_CHECK(native.size() == 9, "Pinned projection result count differs");
  auto solved = native[0], applied = native[5];
  auto result = at::minimum(at::maximum(solved, lower), upper);
  TORCH_CHECK(!((solved.lt(lower + .02 - 1e-10) |
                  solved.gt(upper - .02 + 1e-10)).any()).item<bool>(),
              "L13 feasible target violated the registered joint margin");
  TORCH_CHECK(!(at::abs(result - solved).gt(1e-12).any()).item<bool>(),
              "L13 physical clip unexpectedly modified a feasible target");
  auto roundtrip = fk(solved, phase, signs_expanded, origins_row);
  auto expected_feet = start_feet + applied.unsqueeze(2) * up_column;
  TORCH_CHECK(!(at::abs(roundtrip - expected_feet).gt(1e-9).any()).item<bool>(),
              "L13 feasible projection FK/IK closure failed");
  return result.to(output_anchor);
}
}

TORCH_LIBRARY(sd_target_tail_fk_cache_fileonly_r1, m) {
  m.def("target(Tensor raw, Tensor command, Tensor(a!) phase, Tensor(b!) elapsed, Tensor(c!) filters, Tensor(d!) yaw_filters, Tensor(e!) heading_error, Tensor sensor_yaw_rate, Tensor(f!) previous_clipped, Tensor offsets, Tensor signs, Tensor anchor, Tensor origins, Tensor lower, Tensor upper, Tensor scale, Tensor safe_lower, Tensor safe_upper, Tensor sensor_q, Tensor sensor_up, Tensor output_anchor, Tensor signs_row, Tensor signs_expanded, Tensor origins_row, Tensor up_column) -> Tensor");
}
TORCH_LIBRARY_IMPL(sd_target_tail_fk_cache_fileonly_r1, CPU, m) {
  m.impl("target", TORCH_FN(target));
}
