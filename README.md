# CPU ray tracer V-Image

A toy ray tracer written in C++. For now only uses the CPU. The project was setup using the [ModernCppStarter](https://github.com/TheLartians/ModernCppStarter) template.

## Features
-  **Performance**
    - Using AVX2 simd intrinsics to intersect 2 sibling AABB with 1 ray at once
    - SAH BVH with multi-threaded construction
    - Multi-importance sampling for better render quality

- **Materials**
    - Disney BSDF for variety of materials
    - Lambertian
    - Glass

- **Textures**
    - HDR Environment Map with importance sampling
    - Image textures
    - Trilinear Texture filtering, picking texture level of detail using Ray Cones

- **Camera**
    - Using R2 sequence for per pixel sampling
    - Thin lens depth of field

- **Scene format**
    - gltf
    - json
    - limited mitsuba scene support
    
## Usage

### Build

Use the following command to build the executable.

On Windows
```bash
cmake --build build --config Release --target v-img
```

On Linux
```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

Code has been tested on Linux and Windows, x86 machine.

### Render scenes

Use the following commands from the project's root directory to run the test suite.

```bash
./v_img <path to a scene json file>
```
For example
```bash
./v_img \scenes\empty_box.json
```

Possible integrators that can be set in scene file are "normal", "material" and "mis".

### Sample renders
All renders were done on a AMD Ryzen 7 4700U. Image resolution was set 800 by 800 pixels.

Sphere rendered with Material sampling. 200 samples per pixel and 64 max depth. Render time 43 sec and 145 ms
![Alt text](/renders/sphere_mat.png?raw=true "Sphere mat render 200 samples per pixel and 64 max depth")

Sphere rendered with Multi-importance sampling. 100 samples per pixel and 64 max depth. Render time 50 sec and 690 ms
![Alt text](/renders/sphere_mis.png?raw=true "Sphere mis render 100 samples per pixel and 64 max depth")

Sphere rendered with Material sampling for reference. 5000 samples per pixel and 64 max depth. Render time 20 min, 22 sec and 343 ms
![Alt text](/renders/sphere_ref.png?raw=true "Sphere mat reference render. 5000 samples per pixel and 64 max depth")