#include "CloudProcessing.hpp"

using Point = pcl::PointXYZ;

void NormalEstimationOMP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                         pcl::PointCloud<pcl::Normal>::Ptr normals_cloud)
{
    Timer t;
    t.start();
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> *ne (new pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal>);
    ne->setInputCloud(cloud_in);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne->setSearchMethod(tree);
    ne->setRadiusSearch(0.05);
    ne->compute(*normals_cloud);
    std::cout << "norm: " << t.elapsedMilliseconds() << std::endl;
}

void BilateralFilter(pcl::PointCloud<Point>::Ptr cloud_in,
                     pcl::PointCloud<Point>::Ptr bilateral_cloud)
{
    Timer t;
    t.start();
    // pcl::BilateralFilter<Point> bf;
    // bf.setInputCloud(cloud_in);
    // bf.setHalfSize(2.0);
    // pcl::search::KdTree<Point>::Ptr tree (new
    // pcl::search::KdTree<Point> ()); bf.setSearchMethod (tree);
    // bf.setStdDev(0.2);
    // bf.filter(*bilateral_cloud);
    std::cout << "bilt: " << t.elapsedMilliseconds() << std::endl;
}

void VoxelGrid(pcl::PointCloud<Point>::Ptr cloud_in,
               pcl::PointCloud<Point>::Ptr filtered_cloud)
{
    Timer t;
    t.start();
    pcl::VoxelGrid<Point> *sor (new pcl::VoxelGrid<Point>);
    sor->setInputCloud(
        cloud_in); // Set the point cloud to be filtered for the filtering object
    sor->setLeafSize(
        0.05f, 0.05f,
        0.05f); // Set the voxel size created when filtering is a 1cm cube
    // sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor->filter(*filtered_cloud); // Perform filtering processing and store the
                                 // output cloud_filtered
    std::cout << "voxl: " << t.elapsedMilliseconds() << std::endl;
}

void PassThroughFilter(pcl::PointCloud<Point>::Ptr cloud_in,
                       pcl::PointCloud<Point>::Ptr filtered_cloud)
{
    Timer t;
    t.start();
    pcl::PassThrough<Point> *pass (new pcl::PassThrough<Point>);
    pass->setInputCloud(cloud_in);
    pass->setFilterFieldName("z");
    pass->setFilterLimits(1.0, 6.0);
    // pass.setFilterLimitsNegative (true);
    pass->filter(*filtered_cloud);
    std::cout << "pass: " << t.elapsedMilliseconds() << std::endl;
}

void SACSegmentation(pcl::PointCloud<Point>::Ptr cloud_in,
                     pcl::ModelCoefficients::Ptr coefficients,
                     pcl::PointIndices::Ptr inliers)
{
    Timer t;
    t.start();
    pcl::SACSegmentation<Point> *seg (new pcl::SACSegmentation<Point>);

    seg->setOptimizeCoefficients(true);
    // Mandatory
    seg->setModelType(pcl::SACMODEL_PLANE);
    seg->setMethodType(pcl::SAC_RANSAC);
    seg->setDistanceThreshold(0.2);

    seg->setInputCloud(cloud_in);
    seg->segment(*inliers, *coefficients);

    std::cout << "plan: " << t.elapsedMilliseconds() << std::endl;
}

void RandomSampleConsensus(pcl::PointCloud<Point>::Ptr cloud_in,
                           pcl::PointCloud<Point>::Ptr filtered_cloud)
{
    Timer t;
    t.start();
    std::vector<int> *inliers(new std::vector<int>);

    pcl::SampleConsensusModelPlane<Point>::Ptr model_p(
        new pcl::SampleConsensusModelPlane<Point>(cloud_in));
    pcl::RandomSampleConsensus<Point> *ransac(new  pcl::RandomSampleConsensus<Point>(model_p));

    ransac->setDistanceThreshold(.1);
    ransac->computeModel();
    ransac->getInliers(*inliers);

    pcl::copyPointCloud<Point>(*cloud_in, *inliers, *filtered_cloud);

    std::cout << "rans: " << t.elapsedMilliseconds() << std::endl;
}