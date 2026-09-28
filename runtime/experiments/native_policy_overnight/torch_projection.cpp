// Isolated file-only TorchScript CPU custom operator; no device I/O.
#include <ATen/ATen.h>
#include <torch/library.h>
#include <array>
#include <vector>
#include "projection.cpp"

namespace {
void validate(const at::Tensor& x, at::IntArrayRef shape, const char* name) {
  TORCH_CHECK(x.device().is_cpu(),name," must be CPU");
  TORCH_CHECK(x.scalar_type()==at::kDouble,name," must be float64");
  TORCH_CHECK(x.layout()==at::kStrided && x.sizes()==shape,name," shape/layout differs");
  TORCH_CHECK(x.is_contiguous(),name," must be contiguous");
  TORCH_CHECK(!x.is_neg() && !x.is_conj(),name," must not be a lazy negative/conjugate view");
  TORCH_CHECK(!x.requires_grad(),name," must be inference-only");
}
at::Tensor double_copy(const double* source, at::IntArrayRef shape) {
  auto result=at::empty(shape,at::TensorOptions().device(at::kCPU).dtype(at::kDouble));
  std::copy(source,source+result.numel(),result.data_ptr<double>());
  return result;
}
at::Tensor bool_copy(const int32_t* source) {
  auto result=at::empty({1,4},at::TensorOptions().device(at::kCPU).dtype(at::kBool));
  for(int j=0;j<4;++j) result.data_ptr<bool>()[j]=source[j]!=0;
  return result;
}
std::vector<at::Tensor> project(const at::Tensor& pre,const at::Tensor& start,const at::Tensor& up,
    const at::Tensor& requested,const at::Tensor& alpha,const at::Tensor& uncapped) {
  validate(pre,{1,12},"pre");validate(start,{1,4,3},"start");validate(up,{1,3},"up");
  validate(requested,{1,4},"requested");validate(alpha,{1,4},"alpha");validate(uncapped,{1,4},"uncapped");
  ProjectionResult result{};
  int status=projection_file_only(pre.const_data_ptr<double>(),start.const_data_ptr<double>(),up.const_data_ptr<double>(),
      requested.const_data_ptr<double>(),alpha.const_data_ptr<double>(),uncapped.const_data_ptr<double>(),&result);
  TORCH_CHECK(status==0,"File-only native projection rejected input/postcheck, status=",status);
  // New owning tensors; no borrowed C++ memory or input aliasing escapes.
  return {double_copy(result.solved,{1,12}),double_copy(result.endpoint_q,{1,4,3}),
      double_copy(result.fraction,{1,4}),double_copy(result.first_invalid,{1,4}),
      double_copy(result.hi,{1,4}),double_copy(result.applied,{1,4}),
      bool_copy(result.endpoint_domain),bool_copy(result.endpoint_ok),bool_copy(result.path_rejected)};
}
}

TORCH_LIBRARY(sd_projection_fileonly_r1,m) {
  m.def("project(Tensor pre, Tensor start, Tensor up, Tensor requested, Tensor alpha, Tensor uncapped) -> Tensor[]");
}
TORCH_LIBRARY_IMPL(sd_projection_fileonly_r1,CPU,m) {
  m.impl("project",TORCH_FN(project));
}
