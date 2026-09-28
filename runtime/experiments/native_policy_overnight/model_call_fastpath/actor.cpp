// Experimental CPU-only actor call fusion. This uses the same ATen linear/ELU
// kernels as Sequential and owns no state, transport, clock, or motor output.
#include <ATen/ATen.h>
#include <torch/library.h>

namespace {
void check(const at::Tensor& x, at::IntArrayRef shape, const char* name) {
  TORCH_CHECK(x.device().is_cpu() && x.scalar_type() == at::kFloat &&
              x.layout() == at::kStrided && x.sizes() == shape &&
              x.is_contiguous() && !x.is_neg() && !x.is_conj(),
              name, " must be a contiguous CPU float32 tensor of the pinned shape");
}

at::Tensor forward(const at::Tensor& observation,
                   const at::Tensor& w0, const at::Tensor& b0,
                   const at::Tensor& w1, const at::Tensor& b1,
                   const at::Tensor& w2, const at::Tensor& b2,
                   const at::Tensor& w3, const at::Tensor& b3) {
  check(observation, {1,74}, "observation");
  check(w0, {128,74}, "w0"); check(b0, {128}, "b0");
  check(w1, {128,128}, "w1"); check(b1, {128}, "b1");
  check(w2, {64,128}, "w2"); check(b2, {64}, "b2");
  check(w3, {12,64}, "w3"); check(b3, {12}, "b3");
  auto x = at::linear(observation, w0, b0);
  x = at::elu(x, 1.0, 1.0, 1.0);
  x = at::linear(x, w1, b1);
  x = at::elu(x, 1.0, 1.0, 1.0);
  x = at::linear(x, w2, b2);
  x = at::elu(x, 1.0, 1.0, 1.0);
  return at::linear(x, w3, b3);
}
}

TORCH_LIBRARY(sd_actor_fileonly_r1, m) {
  m.def("forward(Tensor observation, Tensor w0, Tensor b0, Tensor w1, Tensor b1, Tensor w2, Tensor b2, Tensor w3, Tensor b3) -> Tensor");
}
TORCH_LIBRARY_IMPL(sd_actor_fileonly_r1, CPU, m) {
  m.impl("forward", TORCH_FN(forward));
}
