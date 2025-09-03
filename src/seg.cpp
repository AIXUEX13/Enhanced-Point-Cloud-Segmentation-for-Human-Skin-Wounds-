#include "pcl/pcl_macros.h"
#include <iostream>
#include <random>
#include <set>
#include <vector>
#include <cmath>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <ctime>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>

float angleBetweenNormals(const pcl::PointXYZRGBNormal& a, const pcl::PointXYZRGBNormal& b) {
    Eigen::Vector3f normalA(a.normal_x, a.normal_y, a.normal_z);
    Eigen::Vector3f normalB(b.normal_x, b.normal_y, b.normal_z);


    float dotProduct = normalA.dot(normalB);


    float magnitudeA = normalA.norm();
    float magnitudeB = normalB.norm();


    return std::acos(dotProduct / (magnitudeA * magnitudeB));
}
float computeCurvature(const pcl::PointXYZRGBNormal& point, const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, float radius) {

    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
    kdtree.setInputCloud(cloud.makeShared());

    std::vector<int> pointIndices;
    std::vector<float> pointSquaredDistances;

    if (kdtree.radiusSearch(point, radius, pointIndices, pointSquaredDistances) <= 0) {
        return 0.0f;
    }

    
    Eigen::MatrixXd pointsMatrix(pointIndices.size(), 3);
    for (size_t i = 0; i < pointIndices.size(); ++i) {
        const pcl::PointXYZRGBNormal& p = cloud.points[pointIndices[i]];
        pointsMatrix(i, 0) = p.x;
        pointsMatrix(i, 1) = p.y;
        pointsMatrix(i, 2) = p.z;
    }

 
    Eigen::Vector3d centroid = pointsMatrix.colwise().mean();
    Eigen::MatrixXd centered = pointsMatrix.rowwise() - centroid.transpose();
    Eigen::MatrixXd covariance = (centered.transpose() * centered) / double(pointsMatrix.rows() - 1);


    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covariance);
    Eigen::Vector3d eigenValues = solver.eigenvalues();


    std::sort(eigenValues.data(), eigenValues.data() + eigenValues.size());


    float k1 = eigenValues[0];  
    float k2 = eigenValues[1];  

 
    float meanCurvature = (k1 + k2) / 2.0f;

    return meanCurvature;
}

float CCDistance(const pcl::PointXYZRGBNormal& a, const pcl::PointXYZRGBNormal& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) +
                     (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
}

float rgbDistance(const pcl::PointXYZRGBNormal& a, const pcl::PointXYZRGBNormal& b) {
    return std::sqrt((a.r - b.r) * (a.r - b.r) +
                     (a.g - b.g) * (a.g - b.g) +
                     (a.b - b.b) * (a.b - b.b));
}

float normalSimilarity(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, int idx1, int idx2) {

    Eigen::Vector3f normal1(cloud->points[idx1].normal_x, cloud->points[idx1].normal_y, cloud->points[idx1].normal_z);
    Eigen::Vector3f normal2(cloud->points[idx2].normal_x, cloud->points[idx2].normal_y, cloud->points[idx2].normal_z);


    float cosTheta = normal1.dot(normal2) / (normal1.norm() * normal2.norm());


    cosTheta = std::min(1.0f, std::max(-1.0f, cosTheta));


    return std::acos(cosTheta);  
}

void regionGrowing(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                   const std::vector<int>& seedIndices,
                   float distanceThreshold, float normalThreshold, float qlThreshold, float rgbThreshold,
                   const pcl::PointCloud<pcl::Normal>::Ptr& normals,
                   std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>& outputClouds,
                   std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>& outputClouds_w,
                   int minRegionSize, int maxAdditionalSeeds,float km,int k_1) {


    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
    kdtree.setInputCloud(cloud);


    std::vector<int> regionLabels(cloud->size(), -1);  
    int currentRegionId = 0;  


    std::vector<int> seedPoints;
    if (seedIndices.empty()) {
  
        for (int i = 0; i < 12&& i < cloud->size(); ++i) {
            seedPoints.push_back(i);
        }
    } else {
       
        seedPoints = seedIndices;
    }

  
    outputClouds.clear();
    outputClouds_w.clear();
    outputClouds.resize(seedPoints.size());
    outputClouds_w.resize(seedPoints.size());    

    
    for (int seedIdx : seedPoints) {
        if (regionLabels[seedIdx] != -1) continue; 

        
        regionLabels[seedIdx] = currentRegionId;
        std::set<int> toProcessLocal;
        toProcessLocal.insert(seedIdx);

        outputClouds[currentRegionId] = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        outputClouds_w[currentRegionId] = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

       
        while (!toProcessLocal.empty()) {
            int currentIdx = *toProcessLocal.begin();
            toProcessLocal.erase(toProcessLocal.begin());

            outputClouds[currentRegionId]->points.push_back(cloud->points[currentIdx]);

            std::vector<int> pointIdxKSearch(k_1);  
            std::vector<float> pointKSearchSquaredDistance(30);  

           
            int numNeighbors = kdtree.nearestKSearch(cloud->points[currentIdx], 10, pointIdxKSearch, pointKSearchSquaredDistance);

            if (numNeighbors > 0) {
                for (size_t i = 0; i < pointIdxKSearch.size(); ++i) {
                    int neighborIdx = pointIdxKSearch[i];
                    if (neighborIdx == currentIdx || regionLabels[neighborIdx] != -1) continue;

                    float normalSim = normalSimilarity(cloud, currentIdx, neighborIdx);
                    float colorRgbDist = rgbDistance(cloud->points[currentIdx], cloud->points[neighborIdx]);
                    //float curvature = computeCurvature(cloud->points[currentIdx], *cloud, 2.0f);

                    // bool isSimilar =(normalSim <= normalThreshold && curvature < qlThreshold) ||
                    //                  (normalSim > normalThreshold && colorRgbDist < rgbThreshold);
                    bool cdSimilar =(normalSim <= normalThreshold ) ;
                    bool rgbSimilar =(colorRgbDist <= rgbThreshold) ;
                    int m=cloud->size();

                    float th1 = (normalThreshold * km) / m;
                    float th2 = (normalThreshold)/(k_1*m);
                    float rgb1 = (rgbThreshold * km) / m;
                    float rgb2 = (rgbThreshold)/(k_1*m);
                    //cout<<th2<<endl;
                    //cout<<th1<<endl;


                    if (cdSimilar) {
                        regionLabels[neighborIdx] = currentRegionId;
                        toProcessLocal.insert(neighborIdx);
                        normalThreshold=normalThreshold+ th1;
                        //rgbThreshold=rgbThreshold+rgb1;


                        // outFile_1 << normalSim << "," << curvature << std::endl;
                    }

                    // else if(cdSimilar!=1 && rgbsimlar){
                    //     normalThreshold=normalThreshold-0.00001;
                    //     regionLabels[neighborIdx] = currentRegionId;
                    //     toProcessLocal.insert(neighborIdx);
                    //     rgbThreshold=rgbThreshold+0.0001;
                    //  }
                    // else if(cdSimilar!=1 && rgbsimlar !=1)
                    // {
                    //     normalThreshold=normalThreshold-0.00001;
                    //     rgbThreshold=rgbThreshold-0.0001;
                    // }
                    else{
                        normalThreshold=normalThreshold-th2;
                           //cout<<normalThreshold<<endl;
                        //rgbThreshold=rgbThreshold+rgb2;
                    }
                }
            }
        }
        ++currentRegionId;
    }

   
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> regionClouds(currentRegionId);
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (regionLabels[i] != -1) {
            int regionId = regionLabels[i];
            if (!regionClouds[regionId]) {
                regionClouds[regionId] = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
            }
            regionClouds[regionId]->points.push_back(cloud->points[i]);
        }
    }

    outputClouds = regionClouds;

    
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> regionClouds_w(currentRegionId + 1); 
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (regionLabels[i] == -1) {
           
            if (!regionClouds_w[0]) {  
                regionClouds_w[0] = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
            }
            regionClouds_w[0]->points.push_back(cloud->points[i]);
        }
    }
    outputClouds_w = regionClouds_w;

}
// void savePointCloudToTXT(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, const std::string& filename) {
//    
//     std::ofstream txtFile(filename);
//     if (!txtFile.is_open()) {
//         std::cerr << "Failed to open file: " << filename << std::endl;
//         return;
//     }

//     
//     for (const auto& point : cloud->points) {
//         txtFile << point.x << " "   
//                 << point.y << " "   
//                 << point.z << " "  
//                 << static_cast<int>(point.r) << " "  
//                 << static_cast<int>(point.g) << " "  
//                 << static_cast<int>(point.b) << " " 
//                 << point.normal_x << " "  
//                 << point.normal_y << " "  
//                 << point.normal_z << "\n";  
//     }

//     
//     txtFile.close();
//     std::cout << "Point cloud saved to " << filename << std::endl;
// }






int main() {
  
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> ("D:\\Desktop\\LS\\4.22\\mox3\\chusd.pcd", *cloud) == -1) { //* load the file
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    // std::ofstream txt_File("D:\\Desktop\\LS\\11.13\\lbpcd.txt");
    // savePointCloudToTXT(cloud, "D:\\Desktop\\LS\\11.13\\lbpcd.txt");
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normals->points.resize(cloud->points.size());
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        normals->points[i].normal_x = cloud->points[i].normal_x;
        normals->points[i].normal_y = cloud->points[i].normal_y;
        normals->points[i].normal_z = cloud->points[i].normal_z;
    }

    
    float distanceThreshold = 1.5;  
    int k_1=30;
    float km=0.5;
    float normalThreshold = ((2*M_PI)/180);     
    float qlThreshold = 2.0f;         
    float rgbThreshold = 3.0f;      
    std::vector<int> seedIndices = {}; 
    int minRegionSize = 200; 
    int maxAdditionalSeeds = 0;  


    
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> outputClouds;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> outputClouds_w;

    cout<<1<<endl;
  
    regionGrowing(cloud, seedIndices, distanceThreshold, normalThreshold, qlThreshold, rgbThreshold, normals, outputClouds,outputClouds_w,
                  minRegionSize, maxAdditionalSeeds,km,k_1);

    cout<<2<<endl;

    for (size_t i = 0; i < outputClouds.size(); ++i) {
        try {
            if (outputClouds[i] == nullptr) {
                cout << "Cloud " << i << " is null!" << endl;
                continue;
            }

            size_t point_count = outputClouds[i]->points.size();
            if (point_count == 0) {
                cout << "Cloud " << i << " has no points!" << endl;
                continue;
            }

            // 设置 width 和 height
            outputClouds[i]->width = point_count;  
            outputClouds[i]->height = 1;  

            std::stringstream ss;
            ss << "D:\\Desktop\\LS\\4.22\\mox3\\mox3_fg_1.0\\fgbf" << i << ".pcd";  

           
            pcl::io::savePCDFile(ss.str(), *outputClouds[i]);
            cout << "Saved cloud " << i << " to " << ss.str() << outputClouds[i]->size() <<endl;
        }
        catch (const std::exception& e) {
            cout << "Error saving cloud " << i << ": " << e.what() << endl;
        }
    }
    for (size_t i = 0; i < outputClouds_w.size(); ++i) {
        try {
            if (outputClouds_w[i] == nullptr) {
                cout << "Cloud " << i << " is null!" << endl;
                continue;
            }

            size_t point_count = outputClouds_w[i]->points.size();
            if (point_count == 0) {
                cout << "Cloud " << i << " has no points!" << endl;
                continue;
            }

         
            outputClouds_w[i]->width = point_count;  
            outputClouds_w[i]->height = 1;  

            std::stringstream ss;
            ss << "D:\\Desktop\\LS\\4.22\\mox3\\mox3_fg_1.0\\wfgbf" << i << ".pcd";  

           
            pcl::io::savePCDFile(ss.str(), *outputClouds_w[i]);
            cout << "Saved cloud " << i << " to " << ss.str() << outputClouds_w[i]->size() <<endl;
        }
        catch (const std::exception& e) {
            cout << "Error saving cloud " << i << ": " << e.what() << endl;
        }
    }
    return 0;
}

