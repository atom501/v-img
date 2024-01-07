# CPU ray tracer V-Image

A toy ray tracer written in C++. For now only uses the CPU. The project was setup using the [ModernCppStarter](https://github.com/TheLartians/ModernCppStarter) template.

## Implemented Features

- Multi threaded
- BVH with SAH
- Material sampling and Multi-importance sampling
- Scene loading. Currently from JSON files in scenes folder

## Planned Features
- Triangle meshes
- Support more materials
- Moving to GPU
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

