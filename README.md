# CPU ray tracer V-Image

A toy ray tracer written in C++. For now only uses the CPU. The project was setup using the [ModernCppStarter](https://github.com/TheLartians/ModernCppStarter) template.

## Features
-  **Performance**
    - Using AVX2 simd intrinsics to intersect 2 sibling AABB with 1 ray at once
    - SAH BVH with multi-threaded construction
    - Multi-importance sampling for better render quality
    - Russian roulette to stop paths with low contribution

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

Samples per pixel set to 512, max ray depth set to infinite so relying on Russian roulette for stopping a path and AgX tonemapper on the result image. 

Spheres rendered with Multi-importance sampling. Resolution 1800 x 800. Render time: 1m 19s
![Alt text](/renders/disney_spheres_agx_512.png?raw=true "Spheres with Disney BSDF. Rendered at 512 samples per pixel")

Mitsuba matpreview rendered with Multi-importance sampling, to show a variety of materials supported by Disney BSDF. Render time: 6m 16s
![Alt text](/renders/disney_arr_agx_512.png?raw=true "Mitsuba matpreview with Disney BSDF. Rendered at 512 samples per pixel")

Lego glb model rendered. Render time: 2m 30s
![Alt text](/renders/gandalf_lego_agx_512.png?raw=true "Lego minifigure. Rendered at 512 samples per pixel")

Ajax and Roza statues with HDRI background. Showing off camera depth of field. Render time: 1m 32s
![Alt text](/renders/statues_dof_agx_512.png?raw=true "Ajax and Roza statues with HDRI background, focus on Roza. Rendered at 512 samples per pixel")