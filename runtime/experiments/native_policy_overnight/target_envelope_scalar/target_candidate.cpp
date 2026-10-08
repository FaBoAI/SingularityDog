// File-only port of the CURRENT scalar core's step_target tail, using ATen.
// Reuse the already verified scalar/projection ops. No transport, clock, model
// weights, global mutable state, or motor interface is present.
#include <ATen/ATen.h>
#include <ATen/core/dispatch/Dispatcher.h>
#include <torch/library.h>
#include <vector>
#include <cmath>
#include <c10/core/InferenceMode.h>

namespace {
using T = at::Tensor;
using ScalarStep = std::vector<T>(const T&, const T&, T, T, T, T, T,
                                const T&, T, const T&, const T&, const T&,
                                const T&, const T&, const T&);
using Projection = std::vector<T>(const T&, const T&, const T&,
                                 const T&, const T&, const T&);
constexpr double pi = 3.141592653589793;
constexpr double tau = 6.283185307179586;
// Bounded CPU-double loops. Each intermediate keeps the same IEEE operation
// boundary; build MUST disable contraction and fast-math. No trig, projection,
// model state, clocks, dispatch namespace or live admission is changed here.
bool dense_double(const T& t, at::IntArrayRef shape) {
  return c10::InferenceMode::is_enabled() && t.device().is_cpu() &&
      t.scalar_type() == at::kDouble && t.sizes().equals(shape) &&
      t.is_contiguous() && !t.is_conj() && !t.is_neg() && !t.requires_grad();
}

std::vector<T> residual_envelope_reference(const T& ref, const T& activation,
    const T& scale, const T& action, const T& lower, const T& upper) {
  auto residual = activation.unsqueeze(1) * scale * action;
  auto unbounded = ref + residual;
  auto safe_den = at::where(at::abs(residual).gt(1e-15), residual, at::ones_like(residual));
  auto allowed = at::where(residual.gt(1e-15), (upper-ref)/safe_den,
      at::where(residual.lt(-1e-15), (lower-ref)/safe_den, at::ones_like(residual)));
  auto alpha = at::clamp(std::get<0>(allowed.reshape({1,4,3}).min(2)), 0., 1.);
  auto pre = ref + (residual.reshape({1,4,3}) * alpha.unsqueeze(2)).reshape({1,12});
  return {residual, alpha, pre};
}

std::vector<T> residual_envelope(const T& ref, const T& activation,
    const T& scale, const T& action, const T& lower, const T& upper) {
  if (!(dense_double(ref,{1,12}) && dense_double(activation,{1}) &&
        dense_double(scale,{12}) && dense_double(action,{1,12}) &&
        dense_double(lower,{12}) && dense_double(upper,{12}))) {
    return residual_envelope_reference(ref, activation, scale, action, lower, upper);
  }
  auto residual=at::empty({1,12},ref.options());
  auto alpha=at::empty({1,4},ref.options());
  auto pre=at::empty({1,12},ref.options());
  const double* q=ref.const_data_ptr<double>();
  const double* s=scale.const_data_ptr<double>();
  const double* a=action.const_data_ptr<double>();
  const double* lo=lower.const_data_ptr<double>();
  const double* hi=upper.const_data_ptr<double>();
  const double active=activation.const_data_ptr<double>()[0];
  double* r=residual.mutable_data_ptr<double>();
  double* weights=alpha.mutable_data_ptr<double>();
  double* out=pre.mutable_data_ptr<double>();
  for(int leg=0;leg<4;++leg) {
    double allowed[3];
    for(int joint=0;joint<3;++joint) {
      const int i=3*leg+joint;
      const double scaled=active*s[i];
      r[i]=scaled*a[i];
      const double den=std::abs(r[i])>1e-15 ? r[i] : 1.;
      allowed[joint]=r[i]>1e-15 ? (hi[i]-q[i])/den :
          (r[i]<-1e-15 ? (lo[i]-q[i])/den : 1.);
    }
    // ATen dim-min retains the first equal value, including signed zero,
    // and selects the first NaN. Preserve that operand and its payload.
    double w=allowed[0];
    for(int joint=1;joint<3;++joint) {
      if(!std::isnan(w) && (std::isnan(allowed[joint]) || allowed[joint]<w))
        w=allowed[joint];
    }
    // ATen clamp_min canonicalizes either signed zero to positive zero.
    if(w<=0.)w=0.;
    if(w>1.)w=1.;
    weights[leg]=w;
    for(int joint=0;joint<3;++joint) {
      const int i=3*leg+joint;
      const double limited=r[i]*w;
      out[i]=q[i]+limited;
    }
  }
  return {residual,alpha,pre};
}

void check_solved_reference(const T& solved,const T& result,const T& lower,const T& upper) {
  TORCH_CHECK(!((solved.lt(lower+.02-1e-10) |
      solved.gt(upper-.02+1e-10)).any()).item<bool>(),
      "L13 feasible target violated the registered joint margin");
  TORCH_CHECK(!(at::abs(result-solved).gt(1e-12).any()).item<bool>(),
      "L13 physical clip unexpectedly modified a feasible target");
}

void check_solved_target(const T& solved,const T& result,const T& lower,const T& upper) {
  if(!(dense_double(solved,{1,12}) && dense_double(result,{1,12}) &&
       dense_double(lower,{12}) && dense_double(upper,{12}))) {
    check_solved_reference(solved,result,lower,upper); return;
  }
  const double* q=solved.const_data_ptr<double>();
  const double* out=result.const_data_ptr<double>();
  const double* lo=lower.const_data_ptr<double>();
  const double* hi=upper.const_data_ptr<double>();
  bool outside=false;
  for(int i=0;i<12;++i) {
    const double lower_margin=lo[i]+.02;
    const double lower_slack=lower_margin-1e-10;
    const double upper_margin=hi[i]-.02;
    const double upper_slack=upper_margin+1e-10;
    outside=outside || q[i]<lower_slack || q[i]>upper_slack;
  }
  TORCH_CHECK(!outside,"L13 feasible target violated the registered joint margin");
  bool changed=false;
  for(int i=0;i<12;++i)changed=changed || std::abs(out[i]-q[i])>1e-12;
  TORCH_CHECK(!changed,"L13 physical clip unexpectedly modified a feasible target");
}

void check_closure_reference(const T& roundtrip,const T& start,const T& applied,const T& up) {
  auto expected=start+applied.unsqueeze(2)*up;
  TORCH_CHECK(!(at::abs(roundtrip-expected).gt(1e-9).any()).item<bool>(),
      "L13 feasible projection FK/IK closure failed");
}

void check_closure(const T& roundtrip,const T& start,const T& applied,const T& up) {
  if(!(dense_double(roundtrip,{1,4,3}) && dense_double(start,{1,4,3}) &&
       dense_double(applied,{1,4}) && dense_double(up,{1,1,3}))) {
    check_closure_reference(roundtrip,start,applied,up); return;
  }
  const double* q=roundtrip.const_data_ptr<double>();
  const double* feet=start.const_data_ptr<double>();
  const double* lift=applied.const_data_ptr<double>();
  const double* direction=up.const_data_ptr<double>();
  bool failed=false;
  for(int leg=0;leg<4;++leg)for(int axis=0;axis<3;++axis) {
    const int i=3*leg+axis;
    const double delta=lift[leg]*direction[axis];
    const double expected=feet[i]+delta;
    failed=failed || std::abs(q[i]-expected)>1e-9;
  }
  TORCH_CHECK(!failed,"L13 feasible projection FK/IK closure failed");
}

T reference_checks(const T& solved,const T& result,const T& lower,const T& upper,
    const T& roundtrip,const T& start,const T& applied,const T& up) {
  check_solved_reference(solved,result,lower,upper);
  check_closure_reference(roundtrip,start,applied,up);
  return result;
}
T candidate_checks(const T& solved,const T& result,const T& lower,const T& upper,
    const T& roundtrip,const T& start,const T& applied,const T& up) {
  check_solved_target(solved,result,lower,upper);
  check_closure(roundtrip,start,applied,up);
  return result;
}


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
  // Keep both FLOAT32 -> phase conversions and scalar mutation order.
  auto envelope = residual_envelope(ref, d[3].to(phase), scale, d[0].to(phase),
                                    safe_lower, safe_upper);
  auto alpha = envelope[1], pre = envelope[2];
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
  check_solved_target(solved, result, lower, upper);
  auto roundtrip = fk(solved, phase, signs_expanded, origins_row);
  check_closure(roundtrip, start_feet, applied, up_column);
  return result.to(output_anchor);
}
}

TORCH_LIBRARY(sd_target_envelope_scalar_fileonly_r11, m) {
  m.def("target(Tensor raw, Tensor command, Tensor(a!) phase, Tensor(b!) elapsed, Tensor(c!) filters, Tensor(d!) yaw_filters, Tensor(e!) heading_error, Tensor sensor_yaw_rate, Tensor(f!) previous_clipped, Tensor offsets, Tensor signs, Tensor anchor, Tensor origins, Tensor lower, Tensor upper, Tensor scale, Tensor safe_lower, Tensor safe_upper, Tensor sensor_q, Tensor sensor_up, Tensor output_anchor, Tensor signs_row, Tensor signs_expanded, Tensor origins_row, Tensor up_column) -> Tensor");
}
TORCH_LIBRARY_IMPL(sd_target_envelope_scalar_fileonly_r11, CPU, m) {
  m.impl("target", TORCH_FN(target));
}

TORCH_LIBRARY(sd_target_envelope_components_fileonly_r11, m) {
  m.def("reference(Tensor ref, Tensor activation, Tensor scale, Tensor action, Tensor lower, Tensor upper) -> Tensor[]");
  m.def("candidate(Tensor ref, Tensor activation, Tensor scale, Tensor action, Tensor lower, Tensor upper) -> Tensor[]");
  m.def("reference_checks(Tensor solved, Tensor result, Tensor lower, Tensor upper, Tensor roundtrip, Tensor start, Tensor applied, Tensor up) -> Tensor");
  m.def("candidate_checks(Tensor solved, Tensor result, Tensor lower, Tensor upper, Tensor roundtrip, Tensor start, Tensor applied, Tensor up) -> Tensor");
}
TORCH_LIBRARY_IMPL(sd_target_envelope_components_fileonly_r11, CPU, m) {
  m.impl("reference", TORCH_FN(residual_envelope_reference));
  m.impl("candidate", TORCH_FN(residual_envelope));
  m.impl("reference_checks", TORCH_FN(reference_checks));
  m.impl("candidate_checks", TORCH_FN(candidate_checks));
}
