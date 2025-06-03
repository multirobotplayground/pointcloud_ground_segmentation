# Point Cloud Ground Segmentation

This ROS 2 module performs ground segmentation on LiDAR point clouds using the ground segmentation algorithm from the [Robust Field Autonomy Lab](https://github.com/RobustFieldAutonomyLab), originally developed as part of the [`lego_loam`](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) SLAM system.

- The segmentation algorithm was proposed by [Michael Himmelsbach](https://ieeexplore.ieee.org/author/37541729800) and the publication is available at [https://ieeexplore.ieee.org/document/5548059](https://ieeexplore.ieee.org/document/5548059):
- The vertical structures identification was proposed by [Igor Bogoslavskyi](https://ieeexplore.ieee.org/author/37085360086) and the publication is available at [https://ieeexplore.ieee.org/document/7759050](https://ieeexplore.ieee.org/document/7759050):

## Overview

Ground segmentation is a critical preprocessing step in many robotics applications such as autonomous navigation and SLAM. This implementation leverages the ground segmentation technique from LeGO-LOAM, which classifies ground and non-ground points in 3D LiDAR scans based on geometric features and elevation gradients.

## Features

- Efficient ground/non-ground point classification  
- Compatible with standard point cloud formats (e.g., ROS `sensor_msgs/PointCloud2`)  
- Designed for real-time performance in outdoor environments  
- Can be integrated as a preprocessing node in SLAM or obstacle detection pipelines  

## ROS Interface

### Subscribed Topics

- **`/lidar/points`** (`sensor_msgs/PointCloud2`)  
  The input point cloud data, typically from a 3D LiDAR sensor. This topic should contain the full point cloud from which ground and segmented (non-ground) points will be extracted.

### Published Topics

- **`/ground_cloud`** (`sensor_msgs/PointCloud2`)  
  Contains only the points classified as ground. Useful for mapping flat surfaces or terrain.

- **`/segmented_cloud_pure`** (`sensor_msgs/PointCloud2`)  
  Contains the remaining non-ground points (e.g., obstacles, structures, vegetation).

---

### Parameters

#### `laser` Namespace

| Parameter Name           | Type     | Default | Description |
|--------------------------|----------|---------|-------------|
| `num_vertical_scans`     | `int`    | `16`    | Number of vertical scan lines (rings) in the LiDAR sensor (e.g., Velodyne VLP-16). |
| `num_horizontal_scans`   | `int`    | `1800`  | Number of horizontal resolution bins. Higher values provide finer angular resolution. |
| `vertical_angle_bottom`  | `double` | `-15.0` | Vertical angle (in degrees) of the lowest LiDAR scan ring. |
| `vertical_angle_top`     | `double` | `15.0`  | Vertical angle (in degrees) of the highest LiDAR scan ring. |
| `sensor_mount_angle`     | `double` | `0.0`   | Mounting angle of the LiDAR sensor (in degrees). Adjust if the sensor is tilted. |
| `ground_scan_index`      | `int`    | `7`     | Index of the scan ring up to which points are considered for ground segmentation. |

#### `image_projection` Namespace

| Parameter Name             | Type     | Default | Description |
|----------------------------|----------|---------|-------------|
| `segment_valid_point_num`  | `int`    | `5`     | Minimum number of consecutive valid points required in a segment to consider it valid. |
| `segment_valid_line_num`   | `int`    | `3`     | Minimum number of different scan lines that must contain valid points in a segment. |
| `segment_theta`            | `double` | `60.0`  | Angle threshold (in degrees) used to determine whether a group of points belongs to the same segment. |

---

All parameters can be configured via a YAML configuration file or dynamically on the ROS parameter server.

## References

- M. Himmelsbach, F.V. Hundelshausen, and H-J. Wuensche, “Fast Segmentation of 3D Point Clouds for Ground Vehicles,” Proceedings of the IEEE Intelligent Vehicles Symposium, pp. 560-565, 2010.
- I. Bogoslavskyi and C. Stachniss, “Fast Range Image-based Segmentation of Sparse 3D Laser Scans for Online Operation,” Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems, pp. 163-169, 2016.
- Shan, T. & Englot, B. (2018). *LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain*. [IEEE IROS](https://doi.org/10.1109/IROS.2018.8594449)

## License

This project is licensed under the **BSD 3-Clause License (BSDv3)**.  
See the [LICENSE](./LICENSE) file for full license text.
