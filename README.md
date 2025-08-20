# kitti2bag - KITTI Dataset to ROS2 Bag Converter

A powerful Python tool to convert [KITTI datasets](http://www.cvlibs.net/datasets/kitti/) into ROS2 bag files, making autonomous driving research data easily accessible for ROS2-based robotics applications.

## 📝 Overview

This tool converts KITTI dataset files into ROS2 bag format, enabling seamless integration with ROS2 robotics workflows. It supports multiple KITTI dataset types including raw synchronized data and odometry datasets, converting various sensor modalities such as:

- **Stereo Cameras** (grayscale and color)
- **Velodyne LiDAR** point clouds
- **IMU/OXTS** navigation data
- **GPS** positioning data
- **Camera calibration** information
- **Transform trees** (tf/tf_static)

*Hello everybody! I'm looking for more people that can bring this package to the next level. If you'd like to help you can contact me via the email. I'll be happy for every news contribution!*

## 🚀 Quick Start

### Prerequisites

- **Python 3.6+**
- **ROS2** (Humble, Iron, or newer recommended)
- **Required Python packages** (see Installation section)

### Installation

1. **Install via pip (recommended):**
```bash
pip install kitti2bag
```

2. **Install from source:**
```bash
git clone https://github.com/Yutarop/kitti_to_ros2bag.git
cd kitti_to_ros2bag
pip install -e .
```

### Dependencies

The tool requires the following Python packages:
- `pykitti` - KITTI dataset Python library
- `progressbar2` - Progress visualization
- `transforms3d` - 3D transformations
- `opencv-python` - Computer vision operations
- `numpy` - Numerical computations
- `rclpy` - ROS2 Python client library
- `rosbag2_py` - ROS2 bag Python interface
- `sensor_msgs_py` - ROS2 sensor message utilities
- `cv_bridge` - OpenCV-ROS bridge

ROS2 packages:
- `geometry_msgs`
- `sensor_msgs`
- `tf2_msgs`
- `std_msgs`

## 📖 Usage

### Basic Usage

The general syntax is:
```bash
kitti2bag <dataset_type> [directory] [options]
```

### Dataset Types

The tool supports three main dataset types:

1. **`raw_synced`** - Raw synchronized KITTI data with all sensors
2. **`odom_color`** - Odometry dataset with color cameras
3. **`odom_gray`** - Odometry dataset with grayscale cameras

### Command Line Options

| Option | Long Form | Description | Required For |
|--------|-----------|-------------|--------------|
| `-t` | `--date` | Date of raw dataset (e.g., 2011_09_26) | Raw datasets |
| `-r` | `--drive` | Drive number (e.g., 0002) | Raw datasets |
| `-s` | `--sequence` | Sequence number (00-21) | Odometry datasets |

### Examples

#### 1. Raw Synchronized Dataset

```bash
# Download KITTI raw data
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0002/2011_09_26_drive_0002_sync.zip
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip

# Extract data
unzip 2011_09_26_drive_0002_sync.zip
unzip 2011_09_26_calib.zip

# Convert to ROS2 bag
kitti2bag -t 2011_09_26 -r 0002 raw_synced .
```

#### 2. Odometry Dataset (Color)

```bash
# Download KITTI odometry data (sequence 00)
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_color/dataset/sequences/00.zip

# Extract and convert
unzip 00.zip
kitti2bag -s 00 odom_color sequences/
```

#### 3. Odometry Dataset (Grayscale)

```bash
# Download KITTI odometry data (sequence 01)
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_gray/dataset/sequences/01.zip

# Extract and convert
unzip 01.zip
kitti2bag -s 01 odom_gray sequences/
```

### Sample Output

When converting, you'll see progress output like:
```bash
$ kitti2bag -t 2011_09_26 -r 0002 raw_synced .
Exporting static transformations
Exporting time dependent transformations
Exporting IMU
Exporting camera 0
100% (77 of 77) |##########################| Elapsed Time: 0:00:00 Time: 0:00:00
Exporting camera 1
100% (77 of 77) |##########################| Elapsed Time: 0:00:00 Time: 0:00:00
Exporting camera 2
100% (77 of 77) |##########################| Elapsed Time: 0:00:01 Time: 0:00:01
Exporting camera 3
100% (77 of 77) |##########################| Elapsed Time: 0:00:01 Time: 0:00:01
Exporting velodyne data
100% (77 of 77) |##########################| Elapsed Time: 0:00:15 Time: 0:00:15
```

## 📊 Output Format

The tool generates ROS2 bag files with the following topic structure:

### Topics Generated

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/kitti/camera_gray_left/image_raw` | `sensor_msgs/Image` | Left grayscale camera |
| `/kitti/camera_gray_right/image_raw` | `sensor_msgs/Image` | Right grayscale camera |
| `/kitti/camera_color_left/image_raw` | `sensor_msgs/Image` | Left color camera |
| `/kitti/camera_color_right/image_raw` | `sensor_msgs/Image` | Right color camera |
| `/kitti/camera_*/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration data |
| `/kitti/velo/pointcloud` | `sensor_msgs/PointCloud2` | Velodyne LiDAR point cloud |
| `/kitti/oxts/imu` | `sensor_msgs/Imu` | IMU orientation and angular velocity |
| `/kitti/oxts/gps/fix` | `sensor_msgs/NavSatFix` | GPS position |
| `/kitti/oxts/gps/vel` | `geometry_msgs/TwistStamped` | GPS velocity |
| `/tf` | `tf2_msgs/TFMessage` | Dynamic coordinate transforms |
| `/tf_static` | `tf2_msgs/TFMessage` | Static coordinate transforms |

### Bag File Information

After conversion, you'll get a detailed overview:
```
## OVERVIEW ##
path:        kitti_2011_09_26_drive_0002_synced.bag
version:     2.0
duration:    7.8s
start:       Sep 26 2011 13:02:44.33 (1317042164.33)
end:         Sep 26 2011 13:02:52.16 (1317042172.16)
size:        417.2 MB
messages:    1078
compression: none [308/308 chunks]
```

## 🗂️ Dataset Information

### KITTI Raw Data

Raw synchronized datasets contain:
- **Stereo cameras** (4 cameras total: 2 grayscale, 2 color)
- **Velodyne HDL-64E** rotating 3D laser scanner
- **OXTS RT 3003** GPS/IMU system
- **Full calibration** data

Download from: [KITTI Raw Data](http://www.cvlibs.net/datasets/kitti/raw_data.php)

### KITTI Odometry Data

Odometry datasets contain:
- **Sequences 00-21** with ground truth poses
- **Stereo camera** images (color or grayscale)
- **Calibration** information
- **Ground truth** trajectories (sequences 00-10)

Download from: [KITTI Odometry](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)

## 🛠️ Troubleshooting

### Common Issues

#### 1. Command Not Found Error
```bash
# If you get "kitti2bag: command not found", try:
python -m kitti2bag -t 2011_09_26 -r 0002 raw_synced .
```

#### 2. Missing Dependencies
```bash
# Install missing Python packages:
pip install pykitti transforms3d opencv-python

# For ROS2 dependencies:
sudo apt install ros-<distro>-sensor-msgs ros-<distro>-geometry-msgs
```

#### 3. Python Path Issues
```bash
# Ensure your Python environment has access to ROS2:
source /opt/ros/<distro>/setup.bash
```

#### 4. Memory Issues with Large Datasets
- Process smaller sequences or drives
- Ensure sufficient disk space (datasets can be several GB)
- Close other applications to free memory

### Docker Alternative

If you encounter persistent environment issues, consider using Docker:
```bash
# Pull ROS2 image
docker pull ros:humble

# Run with mounted dataset directory
docker run -it -v /path/to/kitti:/data ros:humble bash
```

## 🤝 Contributing

We welcome contributions! Here's how you can help:

### Areas for Improvement
- **Additional sensor support** (more KITTI dataset variants)
- **Performance optimizations** for large datasets
- **Better error handling** and validation
- **Documentation improvements**
- **Testing framework** development
- **CI/CD pipeline** setup

### Development Setup
```bash
# Clone repository
git clone https://github.com/Yutarop/kitti_to_ros2bag.git
cd kitti_to_ros2bag

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install in development mode
pip install -e .

# Install development dependencies
pip install pytest black flake8
```

### Submitting Changes
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Add tests if applicable
5. Ensure code style compliance (`black kitti2bag/`)
6. Commit your changes (`git commit -m 'Add amazing feature'`)
7. Push to the branch (`git push origin feature/amazing-feature`)
8. Open a Pull Request

## 📋 Requirements Summary

- **Operating System**: Linux (Ubuntu 20.04+ recommended), macOS, Windows
- **Python**: 3.6 or higher
- **ROS2**: Humble Hawksbill or newer
- **Disk Space**: Varies by dataset (100MB - 50GB+)
- **Memory**: 4GB+ recommended for large datasets

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **KITTI Dataset**: Created by Andreas Geiger, Philip Lenz, and Raquel Urtasun
- **Original kitti2bag**: Based on the work by Tomas Krejci
- **ROS2 Community**: For the excellent robotics framework
- **Contributors**: Everyone who has contributed to this project

## 📚 Additional Resources

- [KITTI Dataset Homepage](http://www.cvlibs.net/datasets/kitti/)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [PyKITTI Library](https://github.com/utiasSTARS/pykitti)
- [ROS2 Bag Documentation](https://github.com/ros2/rosbag2)

