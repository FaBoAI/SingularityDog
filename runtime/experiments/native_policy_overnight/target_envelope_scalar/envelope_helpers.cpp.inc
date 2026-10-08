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
