#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <chrono>

int PointCloudBoundary2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setSearchMethod(tree);
    normalEstimation.setRadiusSearch(1);  
    normalEstimation.compute(*normals);      

    pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>);
    boundaries->resize(cloud->size());  
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimation;
    boundary_estimation.setInputCloud(cloud);
    boundary_estimation.setInputNormals(normals);  
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    boundary_estimation.setSearchMethod(kdtree_ptr); 
    boundary_estimation.setKSearch(30);               
    boundary_estimation.setAngleThreshold(M_PI * 0.5);  
    boundary_estimation.compute(*boundaries);          

    
    int boundary_count = 0;
    for (size_t i = 0; i < boundaries->size(); i++)
    {
        if (boundaries->points[i].boundary_point != 0)
        {
            boundary_count++;
        }
    }
    std::cout << "边界点的数量: " << boundary_count << std::endl;

    // 3. 可视化和保存
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_visual->resize(cloud->size());

    for (size_t i = 0; i < cloud->size(); i++)
    {
        cloud_visual->points[i].x = cloud->points[i].x;
        cloud_visual->points[i].y = cloud->points[i].y;
        cloud_visual->points[i].z = cloud->points[i].z;
        if (boundaries->points[i].boundary_point != 0)  
        {
            cloud_visual->points[i].r = 255;
            cloud_visual->points[i].g = 0;
            cloud_visual->points[i].b = 0;
            cloud_boundary->push_back(cloud_visual->points[i]);  
        }
        else  // 非边界点
        {
            cloud_visual->points[i].r = 255;
            cloud_visual->points[i].g = 255;
            cloud_visual->points[i].b = 255;
        }
    }

    // 保存结果
    //pcl::io::savePCDFileBinaryCompressed("D:\\Desktop\\LS\\mox3\\bianjie_all_0.0.pcd", *cloud_visual);
    pcl::io::savePCDFileBinaryCompressed("D:\\Desktop\\LS\\mox3\\yige.0.pcd", *cloud_boundary);

    return 0;
}

void visualizePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);  
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);  
    viewer->initCameraParameters();

    
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char** argv)
{
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\Desktop\\LS\\mox3\\yige.pcd", *cloud) == -1)
    {
        PCL_ERROR("无法加载点云文件\n");
        return (-1);
    }

    
    PointCloudBoundary2(cloud);

    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::io::loadPCDFile<pcl::PointXYZRGB>("D:\\Desktop\\LS\\mox3\\yige_bj.0.pcd", *cloud_visual);

    
   // visualizePointCloud(cloud_visual);

    return 0;
}

