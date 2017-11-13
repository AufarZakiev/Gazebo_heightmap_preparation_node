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
| -s <path_string> | Prepared map and generated world saving folder path (optional) |
| -offset <x> <y> | Offset to crop from left-top corner of image (optional) |
| -w <width> | Minimum desired width (optional). Desired width is used to compute final size of image: Gazebo hrighmap needs 2^n + 1 pixel size images, so desired size increases to nearest 2^n + 1 size. For example, 400 px turns to 513 px size. |
| -h <height> | Minimum desired height (optional). Desired height is used to compute final size of image: Gazebo hrighmap needs 2^n + 1 pixel size images, so desired size increases to nearest 2^n + 1 size. For example, 400 px turns to 513 px size. |
| -use_median_filtering <true/false> | Median filter enabling (optional) |
| -use_modified_median_filtering <true/false> | Modified median filter enabling. Works only with median filter enabled (optional) |
| -color_inverse <true/false> | Image color inverse (optional) |
| -bot_trshd <bot_trshd> | Thresholds used to smooth out heightmap. Every pixel below "bot_trshd" turns to 0 value (optional) |
| -top_trshd <top_trshd> | Thresholds used to smooth out heightmap. Every pixel above "top_trshd" turns to 255 value (optional) |
| -iter <iterations> | Filtering iterations count (optional) |

## Examples

```
$ rosrun gazebo_heightmap_preparation gazebo_heightmap_preparation_node -f <path>/map.pgm -offset 1900 1900 -h 257

$ rosrun gazebo_heightmap_preparation gazebo_heightmap_preparation_node -f <path>/map.pgm -s <saving_path> -w 400 -h 200
```
