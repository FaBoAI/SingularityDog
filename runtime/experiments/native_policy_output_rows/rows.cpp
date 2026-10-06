// File-only experiment: owned CPU float32 row -> Python double list.
// No model, device, motor, source freshness, deadline or runtime entry point.
#include <ATen/ATen.h>
#include <torch/library.h>
#include <cmath>
#include <tuple>
#include <vector>

namespace {
std::tuple<int64_t,std::vector<double>> row(const at::Tensor &value,int64_t count) {
    // 99 requests the original Python fallback. Recheck all metadata here
    // before accessing data; the Python selector is not a memory-safety proof.
    if((count!=12&&count!=74)||!value.device().is_cpu()||value.layout()!=at::kStrided||
       value.scalar_type()!=at::kFloat||value.dim()!=2||value.size(0)!=1||
       value.size(1)!=count||!value.is_contiguous()||value.is_neg()||value.is_conj())return {99,{}};
    const float *data=value.const_data_ptr<float>();
    std::vector<double> result;result.reserve(count);
    for(int64_t index=0;index<count;++index) {
        const double item=static_cast<double>(data[index]);
        if(!std::isfinite(item))return {2,{}};
        result.push_back(item); // Exact float32 widening, including signed zero.
    }
    return {0,std::move(result)};
}
}
TORCH_LIBRARY(sd_output_row_fileonly_r1,m) {
    m.def("row(Tensor value, int count) -> (int, float[])");
    m.impl("row",TORCH_FN(row));
}
