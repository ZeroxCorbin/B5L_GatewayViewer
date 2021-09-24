#include "b5l_gateway_viewer.hpp"

#if WIN32
const char *FilePath="D:\\file_in.pcd";
#else
const char *FilePath="/home/zeroxcorbin/file_in.pcd";
#endif

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
std::mutex updateModelMutex_;
bool update_;

bool SaveImageFile(std::vector<unsigned char> fileBytes){

    std::cout << "Writing file bytes: " << fileBytes.size() << std::endl;

    std::ofstream file(FilePath, std::ios::out|std::ios::binary);
    std::copy(fileBytes.cbegin(), fileBytes.cend(),
        std::ostream_iterator<unsigned char>(file));
    file.close();

	return true;
}

void
RecieveData(clsTCPSocket *client){
    while(true){
        client->Write("send\r\n");

        std::vector<unsigned char> vect;
        while(client->HasData()){
            int len = client->Read();
            if(len > 0){
                vect.insert(vect.end(), client->recBuffer, client->recBuffer+len);
            }else{
                break;
            }
            std::this_thread::sleep_for(1ms);
        }

        if(vect.size() > 0){
            SaveImageFile(vect);

            std::unique_lock<std::mutex> updateLock(updateModelMutex_);
            pcl::io::loadPCDFile<pcl::PointXYZ> (FilePath, *cloud_);
            update_ = true;
            updateLock.unlock();
        }

        //std::this_thread::sleep_for(1000ms);
    }
}

void
Connect(){
    clsTCPSocket client;

    client.NameIP = "192.168.0.123";
    client.Port = 8890;

	if(!client.Configure()){
		return;
	}

    if(client.Connect()){
        std::thread client1(RecieveData, &client);
        client1.join();
    }
}

void NormalEstimationOMP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals_cloud){
    Timer t;
    t.start();
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (cloud_in);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (0.05);
        ne.compute (*normals_cloud);
    std::cout <<  "norm: " << t.elapsedMilliseconds() << std::endl;
}

void BilateralFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr bilateral_cloud){
    Timer t;
    t.start();
        // pcl::BilateralFilter<pcl::PointXYZ> bf;
        // bf.setInputCloud(cloud_in);
        // bf.setHalfSize(2.0);
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        // bf.setSearchMethod (tree);
        // bf.setStdDev(0.2);
        // bf.filter(*bilateral_cloud);
    std::cout <<  "bilt: " << t.elapsedMilliseconds() << std::endl;
}

void VoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud){
    Timer t;
    t.start();
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_in);//Set the point cloud to be filtered for the filtering object
        sor.setLeafSize(0.05f, 0.05f, 0.05f);//Set the voxel size created when filtering is a 1cm cube
        //sor.setLeafSize(0.05f, 0.05f, 0.05f);
        sor.filter(*filtered_cloud);//Perform filtering processing and store the output cloud_filtered
    std::cout <<  "voxl: " << t.elapsedMilliseconds() << std::endl;
}

void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud){
    Timer t;
    t.start();
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud_in);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (1.0, 6.0);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*filtered_cloud);
    std::cout <<  "pass: " << t.elapsedMilliseconds() << std::endl;
}

void FindPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers){
    Timer t;
    t.start();
        pcl::SACSegmentation<pcl::PointXYZ> seg;

        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.1);

        seg.setInputCloud (cloud_in);
        seg.segment (*inliers, *coefficients);
    std::cout <<  "plan: " << t.elapsedMilliseconds() << std::endl;
}
void DisplayCloud()
{
    std::cout <<  "Waiting for initial data.\n";
    while(!update_){
        std::this_thread::sleep_for(10ms);
    }

    std::cout <<  "Visualizer starting...." << std::endl;
    pcl::visualization::PCLVisualizer viewer;
    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> handler(cloud_,"intensity");

    viewer.addPointCloud<pcl::PointXYZ>(cloud_,"id");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "id");

    viewer.initCameraParameters ();

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughCloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud (new pcl::PointCloud<pcl::PointXYZ>());;

    pcl::PointCloud<pcl::Normal>::Ptr normalsCloud (new pcl::PointCloud<pcl::Normal>);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    while (!viewer.wasStopped()){

        // Get lock on the boolean update and check if cloud was updated
        std::unique_lock<std::mutex> updateLock(updateModelMutex_);
        if(update_)
        {

            PassThroughFilter(cloud_, passThroughCloud);
            //VoxelGrid(passThroughCloud, voxelCloud);
            

            viewer.updatePointCloud< pcl::PointXYZ >(passThroughCloud,"id");

            // NormalEstimationOMP(voxelCloud, normalsCloud);
            // viewer.removePointCloud("normals");
            // viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(passThroughCloud, normalsCloud, 100, (0.1F), "normals");

            FindPlanes(passThroughCloud, coefficients, inliers);
            viewer.removeShape("plane1");
            viewer.addPlane(*coefficients, "plane1");

            update_ = false;
        }
        updateLock.unlock(); 
        
        viewer.spinOnce(100);
    }

    viewer.close();
}

int
main()
{
    std::thread client(Connect);
    std::thread display(DisplayCloud);

    display.join();
    
    client.join();

    return 0;
}