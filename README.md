# Point Cloud Ground Segmentation

This module performs ground segmentation on LiDAR point clouds using the ground segmentation algorithm from the [Robust Field Autonomy Lab](https://github.com/RobustFieldAutonomyLab), originally developed as part of the [`lego_loam`](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) SLAM system.

## Overview

Ground segmentation is a critical preprocessing step in many robotics applications such as autonomous navigation and SLAM. This implementation leverages the ground segmentation technique from LeGO-LOAM, which classifies ground and non-ground points in 3D LiDAR scans based on geometric features and elevation gradients.

## Features

- Efficient ground/non-ground point classification  
- Compatible with standard point cloud formats (e.g., ROS `sensor_msgs/PointCloud2`)  
- Designed for real-time performance in outdoor environments  
- Can be integrated as a preprocessing node in SLAM or obstacle detection pipelines  

## References

- Shan, T. & Englot, B. (2018). *LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain*. [IEEE IROS](https://doi.org/10.1109/IROS.2018.8594449)

## License

This project is licensed under the **BSD 3-Clause License (BSDv3)**.  
See the [LICENSE](./LICENSE) file for full license text.
