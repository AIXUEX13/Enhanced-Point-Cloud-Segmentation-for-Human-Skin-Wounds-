#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/principal_curvatures.h>
#include <vector>
#include <algorithm>
#include <pcl/io/pcd_io.h>  
#include <fstream>  

struct PointWithCurvature {
    pcl::PointXYZRGBNormal point;
    float curvature;

    bool operator<(const PointWithCurvature& other) const {
        return curvature < other.curvature; 
    }
};
std::stringstream ss;

void computeCurvaturesAndSort(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudXYZRGBNormal,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudXYZSorted
                            ) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
    for (size_t i = 0; i < cloudXYZRGBNormal->points.size(); ++i) {
        pcl::PointXYZ point;
        point.x = cloudXYZRGBNormal->points[i].x;
        point.y = cloudXYZRGBNormal->points[i].y;
        point.z = cloudXYZRGBNormal->points[i].z;
        cloudXYZ->points.push_back(point);
    }


    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> curvatureEstimation;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    for (size_t i = 0; i < cloudXYZRGBNormal->points.size(); ++i) {
        pcl::Normal normal;
        normal.normal_x = cloudXYZRGBNormal->points[i].normal_x;
        normal.normal_y = cloudXYZRGBNormal->points[i].normal_y;
        normal.normal_z = cloudXYZRGBNormal->points[i].normal_z;
        normals->points.push_back(normal);
    }

    pcl::PointCloud<pcl::PrincipalCurvatures> curvatures;
    curvatureEstimation.setInputCloud(cloudXYZ);
    curvatureEstimation.setInputNormals(normals);

    float radius = 1.2f;
    curvatureEstimation.setRadiusSearch(radius);
    curvatureEstimation.compute(curvatures);

    std::vector<PointWithCurvature> pointsWithCurvature;
    for (size_t i = 0; i < cloudXYZ->points.size(); ++i) {
        PointWithCurvature pwc;
        pwc.point = cloudXYZRGBNormal->points[i];

        pwc.curvature = std::min(fabs(curvatures.points[i].pc1), fabs(curvatures.points[i].pc2));  
        pointsWithCurvature.push_back(pwc);
    }


    std::sort(pointsWithCurvature.begin(), pointsWithCurvature.end());


    cloudXYZRGBNormal->points.clear();  
    for (const auto& pwc : pointsWithCurvature) {
        cloudXYZRGBNormal->points.push_back(pwc.point);
    }


    pcl::io::savePCDFileASCII(ss.str(), *cloudXYZRGBNormal);
    std::cout << "Sorted point cloud saved to 'sorted_cloud.pcd'." << std::endl;


    std::ofstream outfile("D:\\Desktop\\LS\\4.22\\pxh_.csv");
    for (const auto& pwc : pointsWithCurvature) {

        outfile << pwc.curvature<<std::endl;
    }
   }

int main() {
       std::stringstream ss1;
    std::string s1="D:\\Desktop\\LS\\4.22\\mox3\\chusd";
    ss << s1<<"_"  << ".pcd";
    ss1<<s1<<".pcd";
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudXYZRGBNormal(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (ss1.str(), *cloudXYZRGBNormal) == -1) { //* load the file
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZSorted(new pcl::PointCloud<pcl::PointXYZ>());
    computeCurvaturesAndSort(cloudXYZRGBNormal, cloudXYZSorted);


    return 0;
}
