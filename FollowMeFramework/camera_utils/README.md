# camera_utils

This package cointains some computer vision operations, especially the camera hardware use.

<!-- TABLE OF CONTENTS -->
<!-- <details> -->
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#description">Description</a></li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#dependencies">Dependencies</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li>
        <a href="#usage">Usage</a>
        <ul>
            <li><a href="#initialization">Initialization</a></li>
            <li><a href="#available-functions">Available functions</a></li>
        </ul>
    </li>
    <li><a href="#license">License</a></li>
    <li><a href="#authors">Authors</a></li>
  </ol>
<!-- </details> -->


## Description

In this python library are collected some useful wrappers to use common RGB-D cameras (i.e., Intel Realsense, Zed) and TOF cameras (i.e., Helios2) in a simple and standard way. Also other functions are provided to perform simple perception.

## Getting started

### Dependencies

Most of the dependencies are automatically collected during the ```camera_utils``` package installation except for ```pyzed``` and ```arena_api``` which have to be installed following respectively the instruction in [ZED SDK installation](docs/zed_installation.md) and [Helios SDK installation](docs/helios_installation.md).

### Installation

To install the camera_utils package on your system, clone the Gitbub repository in a folder of your choice, open the cloned repositery path in a terminal and run the following command

```
python3 -m pip install .
```

If you want to install the package in "editable" or "develop" mode (to prevent the uninstall/install of the 
package at every pkg modification) you have can run the following command:

```
python3 -m pip install -e .
```

#### ZED SDK installation

To install ZED SDK follow the instructions in [ZED SDK installation](docs/zed_installation.md) Readme.

#### Helios SDK installation

To install Helios SDK follow the instructions in [Helios SDK installation](docs/helios_installation.md) Readme.

## Usage

### Initialization

Camera wrapper modules in the ```cameras``` folder inherit their structure from a single camera interface script. They can be imported using a standard python import:
```python
from camera_utils.cameras.IntelRealsense import IntelRealsense
from camera_utils.cameras.Zed import Zed
from camera_utils.cameras.Helios import Helios
```
Then you can instanciate an object for the camera initialization (only Intel Realsense example will be provided but the idea is the same also for the other hardwares):
```python
camera = IntelRealsense()
```

camera initialization can accept diffent parameters depending on what camera configuration you want to address:
```python
camera = IntelRealsense(camera_resolution=IntelRealsense.Resolution.HD, fps=30, serial_number="")
```
available resolutions are: ```LOW```, ```FullHD```, ```HD```, ```QHD```(only for Zed).
FPS depends on what your camera accept in input. \
```serail_number``` is necessary when you attach more than one camera to one computer so that the initilizer will be able to distinguish cameras.

### Available functions

- ```camera.get_rgb()``` returns the rgb image in a ```numpy.array``` format (3 channels, 8 bits).
- ```camera.get_depth()```  returns the depth image in a ```numpy.array``` format (1 channels, 16 bits).
- ```camera.get_frames()```  returns both rgb and depth images in a ```numpy.array```.
- ```camera.get_aligned_frames()```  returns both rgb and depth images aligned (if possible) in a ```numpy.array```.
- ```camera.get_intrinsics()``` return a dictionary containing the intrinsic parameters of the camera (width, height, focal lengths, principal points).
- ```camera.get_pcd()``` return a pointcloud in ```open3d.geometry.PointCloud``` format.

## License

Distributed under the ```GPLv3``` License. See [LICENSE](LICENSE) for more information.

## Authors

The package is provided by:

- [Federico Rollo](https://github.com/FedericoRollo) [Mantainer]
- [Andrea Zunino](https://github.com/andreazuna89)
- [Fabio Amadio](https://github.com/fabio-amadio)
