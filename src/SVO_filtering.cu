#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math.h>
#include <torch/extension.h>

#define THREADS 256



__global__ void filter_kernel( 
        float *res, 
        float *points,
        const int p_size,
        float* centers,
        int* childIds,
        bool* is_ends
        )
    {
    int index = threadIdx.x + blockIdx.x * blockDim.x;
    if (index >= p_size) return;
    float3 p = reinterpret_cast<float3*> (points + index*3)[0];
    int node_idx = 0;
    while (true){
        if (is_ends[node_idx]){
            reinterpret_cast<float3*>(res + index*3)[0] = make_float3(0.f,0.f,0.f);
            break;
        } 
        int pos = 0;
        float3 center = reinterpret_cast<float3*> (centers + node_idx*3)[0];
        if (p.z < center.z) pos += 4;
        if (p.y < center.y) pos += 2;
        if (p.x < center.x) pos += 1;

        int childId = childIds[node_idx*8 + pos];
        if (childId < 0)
        {reinterpret_cast<float3*>(res + index*3)[0] = p;
        break;}
        node_idx = childId;
    }
        
}

void SVO_filtering( torch::Tensor points, torch::Tensor filtered_points, std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> SVO){
    uint32_t p_size = points.size(0);
    torch::Tensor center_tensor = std::get<0>(SVO);
    torch::Tensor childId_tensor = std::get<1>(SVO);
    torch::Tensor is_end_tensor = std::get<2>(SVO);
    dim3 blocks = (THREADS + p_size)/THREADS;
    filter_kernel<<<blocks, THREADS>>>
    (
     filtered_points.data_ptr<float>(),
     points.data_ptr<float>(), 
     p_size, 
     center_tensor.data_ptr<float>(),
     childId_tensor.data_ptr<int>(),
     is_end_tensor.data_ptr<bool>()
     );

}

PYBIND11_MODULE(TORCH_EXTENSION_NAME, m) {
    m.def("run", &SVO_filtering, "build octree from points");
}   