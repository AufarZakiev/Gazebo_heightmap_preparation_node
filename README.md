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
$ cd catkin_ws/src
$ git clone https://github.com/AufarZakiev/Gazebo_heightmap_preparation_node.git
```

Install the dependencies before building node.

```sh
$ sudo apt-get install libmagick++-dev
```

Then build node

```sh
$ catkin_make
```

## Parameters

| Parameter name | Feature |
| ------ | ------ |
| f:=<path_string> | Path to source image (required) |
| s:=<path_string> | Prepared map and generated world saving folder path (optional) |
| offset_x:=<int> offset_y:=<int> | Offset to crop from left-top corner of image (optional) |
| desired_width:=<int> | Minimum desired width (optional). Desired width is used to compute final size of image: Gazebo hrighmap needs 2^n + 1 pixel size images, so desired size increases to nearest 2^n + 1 size. For example, 400 px turns to 513 px size. |
| desired_height:=<int> | Minimum desired height (optional). Desired height is used to compute final size of image: Gazebo hrighmap needs 2^n + 1 pixel size images, so desired size increases to nearest 2^n + 1 size. For example, 400 px turns to 513 px size. |
| use_median_filtering:=<true/false> | Median filter enabling (optional) |
| use_modified_median_filtering:=<true/false> | Modified median filter enabling (optional) |
| use_color_inverse:=<true/false> | Image color inverse (optional) |
| bot_trshd:=<int> | Thresholds used to smooth out heightmap. Every pixel below "bot_trshd" turns to 0 value (optional) |
| top_trshd:=<int> | Thresholds used to smooth out heightmap. Every pixel above "top_trshd" turns to 255 value (optional) |
| iter:=<int> | Filtering iterations count (optional) |

## Examples

### Built-in example

If you are new to this tool, try preparing sample image (located in [sample_images folder](https://github.com/AufarZakiev/Gazebo_heightmap_preparation_node/tree/master/launch/sample_images)) and generating world based on this image by typing
```sh
roslaunch gazebo_heightmap_preparation prepare.launch
```
This filters sample_map.pgm with default parameters and saves prepared image to [result folder](https://github.com/AufarZakiev/Gazebo_heightmap_preparation_node/tree/master/launch/sample_images/result). Also there is `generated_world.world` file with heightmap model based on prepared image. To see it in Gazebo do following:
```sh 
cd <result folder>
gazebo generated_world.world 
```
Voila!
![Gazebo_1](https://user-images.githubusercontent.com/5558521/32720122-a1b1bfb8-c873-11e7-8f25-6d6216ffa5e8.png)

### Parameters usage example

Various parameters are controlled through launch file arguments. Here and following examples show results of launch `prepare.launch` with different arguments in image viewer and in Gazebo.

#### Color inverse
```sh
roslaunch gazebo_heightmap_preparation prepare.launch use_color_inverse:=true
```

#### Median filter
```sh
roslaunch gazebo_heightmap_preparation prepare.launch use_median_filter:=true
```

#### Modified median filter
```sh
roslaunch gazebo_heightmap_preparation prepare.launch use_modified_median_filter:=true
```

#### Offset
```sh
roslaunch gazebo_heightmap_preparation prepare.launch offset_x:=1700 offset_x:=1700
```

#### Desired sizes
```sh
roslaunch gazebo_heightmap_preparation prepare.launch desired_width:=200
```

#### Thresholds
```sh
roslaunch gazebo_heightmap_preparation prepare.launch bot_thshd:=206
```

#### Filtering iterations
```sh
roslaunch gazebo_heightmap_preparation prepare.launch use_median_filter:=true iter:=5
```