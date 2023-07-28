octomap_mapping ![CI](https://github.com/OctoMap/octomap_mapping/workflows/CI/badge.svg)
===============

ROS stack for mapping with OctoMap, contains the `octomap_server` package.

The main branch for ROS Kinetic and newer is `kinetic-devel`.

# Param Explanation
**Default param setting can be found in octomap_server/src/OctomapServer.cpp**

## Frame setting
- `frame_id`:
World frame id (octomap's frame)

- `base_frame_id`:
Base frame id (robot base frame)

## Color in octomap
- `height_map`:
If using height map to colorize octomap

- `color_factor`:
Color factor used in height map colorization option

****

- `colored_map`:
If use colored map


## Constrain octomap region in World Frame
**NOTE: this is to crop around the fixed world frame center, NOT rolling octomap**

- `pointcloud_min_x`: min x region around world center
- `pointcloud_max_x`: max x region around world center

- `pointcloud_min_y`: min y region around world center
- `pointcloud_max_y`: max y region around world center

- `pointcloud_min_z`: min z region around world center
- `pointcloud_max_z`: max z region around world center


## Rolling octomap 
**NOTE: this is to crop the octomap around base in the fixed world frame**
- `min_x`: min x region around base center in world frame
- `max_x`: max x region around base center in world frame

- `min_y`: min y region around base center in world frame 
- `max_y`: max y region around base center in world frame

- `min_z`: min z region around base center in world frame
- `max_z`: max z region around base center in world frame 


## Min size of volume current node can occupied
- `min_x_size`: (default: 0)
- `min_y_size`: (default: 0)


## Ground segmentation
**Enable to filter out the ground plane  (Should not working if your pointcloud_min_z and pointcloud_max_z is above 0)**

- `filter_ground`: If apply ground segmentation for point cloud in the base frame

### Customized axis
- `If_set_g_seg_Axis`: If customized the axis for ground segmentation (ground plane should be perpendicular to the axis)
- `g_segX`: axis x value
- `g_segY`: axis y value
- `g_segZ`: axis z value

### Filtering param
- `ground_filter/distance`: how close a point must be to the model in order to be considered an inlier

- `ground_filter/angle`: allowed angle of the ground plane (unit: rad)

- `ground_filter/plane_distance`: d-value threshold to classify ground plane or horizontal plane in 3d plane equation (ax+by+cz+d = 0)



## Octomap setting
- `kResolution`: octomap resolution

- `sensor_model/max_range`: maximum range to integrate points from sensor origin(used to speedup!) (set to negative to include all points)

- `sensor_model/hit`: sets the probability for a "hit" (will be converted to logodds) - sensor model (a voxel be classified as occupied)

- `sensor_model/miss`: sets the probability for a "miss" (will be converted to logodds) - sensor model (a voxel be classified as unoccupied)

- `sensor_model/min`: sets the minimum threshold for occupancy clamping (sensor model) value below it is set to this value

- `sensor_model/max`: sets the maximum threshold for occupancy clamping (sensor model) value above it is set to this value


## Data saver
**Used to save octomap, point cloud (ground and nonground), transform, timestamp**

- `If_save`: If save data (enable flag)
- `If_btmap`: If save octomap as binary (true for .bt, false for .ot)
- `If_save_cloud`: If save cloud data
- `savepath`: path to save data (must exist)


## Pre-transform point cloud (sensor to base)
**Apply transform (from calib yaml file) to incoming cloud (used for case when cloud is in sensor frame but no tf between sensor to base)**

- `If_pretransform_s2b`: If apply pre-transform (enable flag)
- `Tfilename_`: calibration file name that saves the pre-transform
-  `camname_`: 1st name field to access the pre-transform
-  `Tname_`: 2nd name field to access the pre-transform

i.e., pre-transform should be saved in yaml file like:
```YAML
camname_:
  Tname_:
    - [m11, m12, m13, m14]
    - [m21, m22, m23, m24]
    - [m31, m32, m33, m34]
    - [m41, m42, m43, m44]
```



## Misc
- `latch`:
If enable latch in the publisher (if the publisher no longer publish message. With latch enabled, when subscribing to it, you can still get one last msg it sent)

- `publish_free_space`:
If enable publisher to indicate free space in octomap visualization

- `occupancy_min_z`:
Related to occupancy map creation (Not sure actual usage)

- `occupancy_max_z`:
Related to occupancy map creation (Not sure actual usage)  

- `filter_speckles`:
Enable to filter out speckle node (single occupied node with no neighbor)

- `compress_map`: If Enable Lossless compression of the octree: A node will replace all of its eight children if they have identical values. (.prune())

- `incremental_2D_projection`:If Enable *incremental* 2d projection map update

- `use_sim_time`: If use sim time (same as bag file time if `--clock` is used)



# Simplified pipeline for octomap_server
![simple_octomap_server_pipeline](https://github.com/ori-drs/octomap_mapping/blob/rolling-octomap/Doc/vRecoPipeline%20-%20octomap_server%20basic%20pipeline.jpg)







