#include <fmt/core.h>
#include <stdio.h>

#include "cuda_runtime.h"
#include "device_launch_parameters.h"

__global__ void cuda_hello() { printf("Hello World from GPU!\n"); }

int main() {
  cuda_hello<<<1, 1>>>();
  cudaError_t cudaerr = cudaDeviceSynchronize();
  if (cudaerr != cudaSuccess)
    printf("kernel launch failed with error \"%s\".\n", cudaGetErrorString(cudaerr));

  fmt::print("fmt test \n");
  return 0;
}