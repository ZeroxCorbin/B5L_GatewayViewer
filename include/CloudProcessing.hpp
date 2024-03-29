#include <iostream>
#include <ostream>
#include <chrono>
#include <thread>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/bilateral.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "clsTimer.hpp"
using Point = pcl::PointXYZI;

void NormalEstimationOMP(pcl::PointCloud<Point>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals_cloud);
void BilateralFilter(pcl::PointCloud<Point>::Ptr cloud_in, pcl::PointCloud<Point>::Ptr bilateral_cloud);
void VoxelGrid(pcl::PointCloud<Point>::Ptr cloud_in, pcl::PointCloud<Point>::Ptr filtered_cloud);
void PassThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<Point>::Ptr filtered_cloud);
void SACSegmentation(pcl::PointCloud<Point>::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers);
void RandomSampleConsensus(pcl::PointCloud<Point>::Ptr cloud_in, pcl::PointCloud<Point>::Ptr filtered_cloud);