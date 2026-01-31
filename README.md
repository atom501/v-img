# CPU ray tracer V-Image

A toy ray tracer written in C++. For now only uses the CPU. The project was setup using the [ModernCppStarter](https://github.com/TheLartians/ModernCppStarter) template.

## Features
-  **Performance**
    - Using AVX2 simd intrinsics to intersect 2 sibling AABB with 1 ray at once
    - AABBs arranged in a cache friendly layout for Ray-AABB intersection tests
    - Fast Binned SAH BVH with multi-threaded construction
    - High quality Sweep SAH BVH with multi-threaded construction. Optimized *O(n log n)* construction, primitives are sorted once per axis and kept sorted within each subset during recursion. As described in [Bonsai BVH paper](https://jcgt.org/published/0004/03/02/)
    - Multi-importance sampling for better render quality
    - Russian roulette to stop paths with low contribution

- **Materials**
    - Disney BSDF for variety of materials
    - Lambertian
    - Glass

- **Textures**
    - HDR Environment Map with importance sampling
    - Image textures with mipmapping
    - Trilinear Texture filtering, picking texture level of detail using Ray Cones
    - Normal maps are supported

- **Camera**
    - Using R2 sequence for per pixel sampling
    - Thin lens depth of field

- **Scene formats**
    - gltf/glb
    - json
    - limited mitsuba scene support
    
## Usage

### Build

Use the following command to build the executable.

#### On Windows
Create folder called build and open the project root directory in Developer Command Prompt for VS. Then run commands
```bash
cmake -S . -B Build
cmake --build build --config Release --target v-img
```

#### On Linux
```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

Code has been tested on Linux and Windows, x86 machine with AVX2 supported CPU.

### Render scenes
gltf/glb scenes can be downloaded separately from here, [link](https://drive.google.com/drive/folders/1oGOL7pHzUzGY0wZgyGapAFy9W-ujGjQx?usp=sharing). They are not included in the repo to avoid large files.

Use the following command from the project's root directory to render the scenes.

```bash
./v-img -f <file_name>
```
For example
```bash
./v-img -f ./scenes/glass_in_box.json
```
For gltf/glb files, extra parameters for rendering are included in a json file. Command for rendering them is:
```
./v-img -f <gltf/glb file> -j < companion json file>
```

Integrators that can be set in scene file are "normal", "material" and "mis".

### Sample renders
All renders were done on a AMD Ryzen 7 7700, on Linux compiled with gcc.

Samples per pixel set to 512, max ray depth set to infinite so relying on Russian roulette for stopping a path and AgX tonemapper on the result image. High quality Sweep SAH BVH is used for all renders.

- Spheres rendered with Multi-importance sampling. Resolution 1800 x 800. Render time: 57 sec
![Alt text](/renders/disney_spheres_agx_512.png?raw=true "Spheres with Disney BSDF. Rendered at 512 samples per pixel")

- Mitsuba matpreview rendered with Multi-importance sampling, to show a variety of materials supported by Disney BSDF. Resolution 1366 x 1024. Render time: 4 min 43 sec
![Alt text](/renders/disney_arr_agx_512.png?raw=true "Mitsuba matpreview with Disney BSDF. Rendered at 512 samples per pixel")

- Lego glb model rendered. Resolution 1366 x 768. Render time: 2 min 24 sec
![Alt text](/renders/gandalf_lego_agx_512.png?raw=true "Lego minifigure. Rendered at 512 samples per pixel")

- Ajax and Roza statues with HDRI background. Resolution 1366 x 768. Showing off camera depth of field. Render time: 1 min 21 sec
![Alt text](/renders/statues_dof_agx_512.png?raw=true "Ajax and Roza statues with HDRI background, focus on Roza. Rendered at 512 samples per pixel")