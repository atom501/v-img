# CPU ray tracer V-Image

A toy ray tracer written in C++. For now only uses the CPU. The project was setup using the [ModernCppStarter](https://github.com/TheLartians/ModernCppStarter) template.

## Implemented Features

- Multi threaded
- BVH with SAH
- Material sampling and Multi-importance sampling
- Scene loading. Currently from JSON files in scenes folder

## Future Features
- Triangle meshes
- More sampling options 
- Support more materials
- Moving to GPU
- Volumetric rendering
- Subsurface scattering

## Usage

### Build

Use the following command to build the executable.

```bash
cmake --build build --config Release --target v-img
```
So far only tested on Windows 10. Code is standard C++ so shouldn't cause issues in other OS, but haven't tested yet.

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