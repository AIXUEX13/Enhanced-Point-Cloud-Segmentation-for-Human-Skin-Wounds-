# Enhanced-Point-Cloud-Segmentation-for-Human-Skin-Wounds
This repository contains the source code for the manuscript Enhanced Point Cloud Segmentation for Human Skin Wounds Using Adaptive Region-Growing with Dynamic Thresholds, submitted to The Visual Computer.

# Compilation environment
Operating system: Windows 11
Qt version: Qt 6.6.3
Compiler: MSVC 2019, x86_64
Build tool: Qt Creator 13.0.1 (Community)
Dependency Library:
- PCL (Point Cloud Library) 1.12+ https://pointclouds.org/downloads/
- Eigen3 3.3+
- VTK 8.2+ https://vtk.org/download/
- Boost 1.74+

#  Compilation methods
1.Open the project.pro file.
2.Adjust the normal vector angle threshold and RGB threshold in the source code as needed.
3.Set the threshold reward factor and the CVS value.
4.Update the point cloud file path to your dataset.
5.In Qt Creator, select the Debug configuration, then click Build and Run.

# Usage Example
When running from Qt Creator, no arguments are passed by default. You need to enter them in Qt Creator → Projects → Run → Command line arguments, for example:
D:\QTPCL\data\test_input.pcd D:\QTPCL\data\seg_result.pcd
After execution, the segmentation result will be presented in a visualized form.

# Repository Structure

Repository Structure
├── src/                  # Source code
│   └── main.cpp
│
├── data/                 # Sample input point clouds
│
├── results/              # Segmentation outputs
│
├── docs/                 # Documentation, figures
│
├── project.pro           # Qt project configuration file
└── README.md             # Project description


# Parameters
Normal vector angle threshold: Used to control the normal angle threshold for surface segmentation.
RGB threshold: Used to control the similarity threshold of colors.
Threshold reward factor（km）: Used to balance the adjustment of thresholds during segmentation.
CVS (Current Value Score): Used to determine whether the segmentation process should be terminated.


