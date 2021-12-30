# Welcome to `franka_plotter 1.0.0`!

`franka_plotter` is a program that makes Franka Emika Panda robot plot images! It is written in `C++` and uses [libfranka](https://frankaemika.github.io/docs/libfranka.html).

### Contents
1. [Welcome to franka_plotter](#welcome-to-franka_plotter)
2. [Contents](#contents)
3. [Dependencies](#dependencies)
4. [Usage](#usage)
5. [Building](#building)
6. [Contributors](#contributors)

### Dependencies
 - [libfranka](https://frankaemika.github.io/docs/libfranka.html)
 - [CMake](https://cmake.org)
 - (optional) [GIMP](https://www.gimp.org)
 - (optional) [Autotrace](https://github.com/autotrace/autotrace) (Follow [this](https://ourcodeworld.com/articles/read/1260/how-to-use-and-build-autotrace-from-source-in-ubuntu-18-04) guide to build from source)

### Usage
`franka_plotter` requires a [SVG](https://developer.mozilla.org/en-US/docs/Web/SVG) image to operate. The image can be passed through command line:
```
./franka_plotter image.svg
```
`franka_plotter_grasp` is an utility to grasp and release the robot's fingers.
```
./franka_plotter_grasp 10   #Grasp with force of 10 Newtons
./franka_plotter_grasp      #Release
```
Note that `franka_plotter` has only partial support of [SVG](https://developer.mozilla.org/en-US/docs/Web/SVG) format - it supports only `<path>` elements, all other elements will be ignored. One way to get such image is to use `vectorize.sh` script (requires [GIMP](https://www.gimp.org) and [Autotrace](https://github.com/autotrace/autotrace)):
```
./vectorize.sh image.png
```

`fraka_plotter` also reads configuration file `fraka_plotter.config`. Parameters, their description and default values could be found in the file.

### Building
The library can be built with [CMake](https://cmake.org) using the following commands:
```
mkdir build
cd build
cmake ..
cmake --build .
```

### Contributors
 - Kyrylo Sovailo
 - Models from `model` directory were initially generated with [Gazebo](http://gazebosim.org) from `ROS` package `franka_description`
 - Example image `derpy.png` depicts Derpy Hooves. All rights on the character are reserved by Hasbro Inc