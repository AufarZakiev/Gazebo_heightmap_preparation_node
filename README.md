# Gazebo-heightmap-preparation-node

## Overview

This is ROS node providing automatic images edition before importing it in Gazebo simulator as heightmap. 

## Features

  - User-controlled image cropping
  - Automatic grayscaling
  - Automatic Gazebo-defined resizing

## References

Please reference to **"Automatic mapping and filtering tool: from a sensor-based occupancy grid to a 3D Gazebo octomap"** paper by Roman Lavrenov, Aufar Zakiev, Evgeni Magid in academic publications used this tool.

## Installation

Node requires [ImageMagick](https://www.imagemagick.org/script/index.php) to build and run.

Switch to catkin workspace and download node

```
$ cd catkin_ws
$ git clone https://github.com/AufarZakiev/Gazebo_heightmap_preparation_node.git
```

Install the dependencies before building node.

```sh
$ sudo apt-get install libmagick++dev
```

Then build node

```sh
$ catkin_make
```

## Parameters

| Parameter name | Feature |
| ------ | ------ |
| -f <path_string> | Path to source image (required) |
| -s <path_string> | Path to processed image (optional) |
| -offset <x> <y> | Offset from top-left corner to crop (optional) |
| -w <width> | Minimum desired width (optional) |
| -h <height> | Minimum desired height (optional) |

## Examples

```
$ rosrun gazebo_heightmap_preparation gazebo_heightmap_preparation_node -f <path>/map.pgm -offset 1900 1900 -h 257

$ rosrun gazebo_heightmap_preparation gazebo_heightmap_preparation_node -f <path>/map.pgm -s <saving_path> -w 400 -h 200
```
