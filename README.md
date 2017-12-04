# Gazebo-heightmap-preparation

## Overview

This is ROS node providing automatic images edition before importing it in Gazebo simulator as heightmap.

## Features

- Image noise filtration
- User-controlled image cropping
- Automatic grayscaling
- Automatic Gazebo-defined resizing

## References

Please refer to **"Automatic mapping and filtering tool: from a sensor-based occupancy grid to a 3D Gazebo octomap"** paper by Roman Lavrenov, Aufar Zakiev, Evgeni Magid in academic publications used this tool.

## Installation

Node requires [ImageMagick](https://www.imagemagick.org/script/index.php) to build and run.

Switch to catkin workspace and download package:

```
$ cd catkin_ws/src
$ git clone https://github.com/AufarZakiev/Gazebo_heightmap_preparation_node.git
```

Install the dependencies before building nodes:

```sh
$ sudo apt-get install libmagick++-dev
```

Then build nodes:

```sh
$ catkin_make
```

## Parameters

| Parameter name | Feature |
| ------ | ------ |
| map_filepath:=<path_string> | Absolute path to source image (required) |
| saving_filepath:=<path_string> | Prepared map and generated world saving folder absolute path (optional) |
| offset_x:=<offset_int> offset_y:=<offset_int> | Offset to crop from left-top corner of image (optional) |
| desired_width:=<width_int> | Minimum desired width (optional). Desired width is used to compute final size of image: Gazebo hrighmap needs 2^n + 1 pixel size images, so desired size increases to nearest 2^n + 1 size. For example, 400 px turns to 513 px size. |
| desired_height:=<height_int> | Minimum desired height (optional). Desired height is used to compute final size of image: Gazebo hrighmap needs 2^n + 1 pixel size images, so desired size increases to nearest 2^n + 1 size. For example, 400 px turns to 513 px size. |
| use_median_filtering:=<true/false> | Median filter enabling (optional) |
| use_modified_median_filtering:=<true/false> | Modified median filter enabling (optional) |
| use_color_inverse:=<true/false> | Image color inverse (optional) |
| bot_trshd:=<trshd_int> | Thresholds used to smooth out heightmap. Every pixel below "bot_trshd" turns to 0 value (optional) |
| top_trshd:=<trshd_int> | Thresholds used to smooth out heightmap. Every pixel above "top_trshd" turns to 255 value (optional) |
| iter:=<iter_int> | Filtering iterations count (optional) |
| world_length:=<length_int> | Generated heightmap length in meters (optional) |
| world_width:=<width_int> | Generated heightmap width in meters (optional) |
| world_height:=<height_int> | Generated heightmap height in meters (optional) |

## Examples

Using launch-file `prepare.launch` is preferred. It contains `map_preparing_node` and `world_creator` nodes. To explore their args and usage type `--help` option, for example:
```sh
rosrun gazebo_heightmap_preparation world_creator --help
```

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

Various parameters are controlled through launch file arguments. Here and following examples show results of launch `prepare.launch` with different arguments in image viewer or/and in Gazebo.

#### Color inverse
```sh
roslaunch gazebo_heightmap_preparation prepare.launch use_color_inverse:=false
```
Inverted heights:
![Gazebo_2](https://user-images.githubusercontent.com/5558521/32825507-4f2532aa-c9f6-11e7-841c-048e72b2b70c.png)

#### Median filter
```sh
roslaunch gazebo_heightmap_preparation prepare.launch use_median_filtering:=false use_color_inverse:=false
```
| Before | After |
| ------ | ------ |
|![initial_map](https://user-images.githubusercontent.com/5558521/32825820-5ac67d0c-c9f7-11e7-967c-3b53b429f1e1.png) | ![filtered](https://user-images.githubusercontent.com/5558521/32825727-1222eeaa-c9f7-11e7-8f02-b41a9962655d.png) |

#### Modified median filter
```sh
roslaunch gazebo_heightmap_preparation prepare.launch use_modified_median_filtering:=true use_color_inverse:=false
```
| Before | After |
| ------ | ------ |
|![initial_map](https://user-images.githubusercontent.com/5558521/32825820-5ac67d0c-c9f7-11e7-967c-3b53b429f1e1.png)|![modified](https://user-images.githubusercontent.com/5558521/32825878-835c39d2-c9f7-11e7-8d76-e278258e3b6c.png)

#### Offset
```sh
roslaunch gazebo_heightmap_preparation prepare.launch offset_x:=1700 offset_x:=1700 use_color_inverse:=false
```
| Before (x=1800, y=1800) | After (x=1700, y=1700) |
| ------ | ------ |
|![initial_map](https://user-images.githubusercontent.com/5558521/32825820-5ac67d0c-c9f7-11e7-967c-3b53b429f1e1.png)|![modified](https://user-images.githubusercontent.com/5558521/32825952-b80aa02e-c9f7-11e7-84ba-6edbfd3e393b.png)

#### Desired sizes
```sh
roslaunch gazebo_heightmap_preparation prepare.launch desired_width:=200 desired_height:=200 use_color_inverse:=false
```
| Before | After |
| ------ | ------ |
|![initial_map](https://user-images.githubusercontent.com/5558521/32825820-5ac67d0c-c9f7-11e7-967c-3b53b429f1e1.png)|![sizes](https://user-images.githubusercontent.com/5558521/32826058-072852dc-c9f8-11e7-87e2-f1f595eb5594.png)

#### Thresholds
```sh
roslaunch gazebo_heightmap_preparation prepare.launch bot_trshd:=206 use_color_inverse:=false
```
| Before | After |
| ------ | ------ |
|![initial_map](https://user-images.githubusercontent.com/5558521/32825820-5ac67d0c-c9f7-11e7-967c-3b53b429f1e1.png)|![sizes](https://user-images.githubusercontent.com/5558521/32826260-c22c3b5c-c9f8-11e7-96df-3f478e5f8e9c.png)

#### World sizes
```sh
roslaunch gazebo_heightmap_preparation prepare.launch use_modified_median_filtering:=true use_color_inverse:=true world_height:=5
cd <result_folder>
gazebo genereated_world.world
```
All walls became higher
![](https://user-images.githubusercontent.com/5558521/32828808-3465bb96-ca01-11e7-8f16-029acb3a86ff.png)
