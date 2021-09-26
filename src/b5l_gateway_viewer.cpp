#include "b5l_gateway_viewer.hpp"

#if WIN32
const char *FilePath="C:\\Users\\jack\\Dropbox\\Projects\\Cpp\\B5L_GatewayViewer\\build\\Debug\\file_in.pcd";
#else
const char *FilePath="/home/zeroxcorbin/file_in.pcd";
#endif

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn_(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut_(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::Normal>::Ptr cloudNormals_(new pcl::PointCloud<pcl::Normal>);
std::vector<char> cloud_buffers;// (new std::vector<char>);

std::mutex updateModelMutex_;
bool update_;
bool exit_app_;

void
CopyBuffersToPointCloud()
{
	/* The result format should be 0x001 or 0x002 (PCD) */

	pcl::PointXYZ currPoint;

    cloudIn_->points.clear();

	short *pXYZ;//,*pAmplitude;
	int i;

	pXYZ = (short *)(&cloud_buffers[0]);
    //pAmplitude = (short *)(&cloud_buffers[0] + (6 * 76800));

	/* QVGA Loop */
	for (i = 0; i < 76800; i++)
	{
		currPoint.x = (float)*pXYZ++ / 1000;
		currPoint.y = (float)*pXYZ++ / 1000;
		currPoint.z = (float)*pXYZ++ / 1000;
		//currPoint.intensity = *pAmplitude++;

		cloudIn_->points.push_back(currPoint);
	}

}

bool SaveImageFile(std::vector<unsigned char> fileBytes){

    std::ofstream file(FilePath, std::ios::out|std::ios::binary);
    if(!file.fail()){
        std::copy(fileBytes.cbegin(), fileBytes.cend(), std::ostream_iterator<unsigned char>(file));
        file.close();

        return true;
    }else{
        std::cout << "Writing file failed!" << std::endl; 

        return false;
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

void SACSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers){
    Timer t;
    t.start();
        pcl::SACSegmentation<pcl::PointXYZ> seg;

        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (2.0);

        seg.setInputCloud (cloud_in);
        seg.segment (*inliers, *coefficients);
    std::cout <<  "plan: " << t.elapsedMilliseconds() << std::endl;
}

void RandomSampleConsensus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud){
    Timer t;
    t.start();    
        std::vector<int> *inliers (new std::vector<int>);
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_in));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
        
        ransac.setDistanceThreshold (.1);
        ransac.computeModel();
        ransac.getInliers(*inliers);
        pcl::copyPointCloud<pcl::PointXYZ>(*cloud_in, *inliers, *filtered_cloud);
    std::cout <<  "rans: " << t.elapsedMilliseconds() << std::endl;
}

bool ProcessNormals_ = false;

void ProcessPointCloud(){

    // pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughCloud (new pcl::PointCloud<pcl::PointXYZ>());

    // pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud (new pcl::PointCloud<pcl::PointXYZ>());;

    // pcl::PointCloud<pcl::Normal>::Ptr normalsCloud (new pcl::PointCloud<pcl::Normal>);

    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr inliersCloud (new pcl::PointCloud<pcl::PointXYZ>());

    // pcl::PointCloud<pcl::PointXYZ>::Ptr ransacCloud (new pcl::PointCloud<pcl::PointXYZ>());;

    PassThroughFilter(cloudIn_, cloudOut_);
    if(ProcessNormals_){
         NormalEstimationOMP(cloudOut_, cloudNormals_);   
    }else{
        cloudNormals_->points.clear();
    }


          //VoxelGrid(passThroughCloud, voxelCloud);
            

            // SACSegmentation(passThroughCloud, coefficients, inliers);
            // pcl::copyPointCloud<pcl::PointXYZ>(*passThroughCloud, *inliers, *inliersCloud);
            // viewer->removeShape("plane1");
            // viewer->addPlane(*coefficients, "plane1");

            //RandomSampleConsensus(passThroughCloud, ransacCloud);

}

void
RecieveData(clsTCPSocket *client){
    Timer t;
    bool sent = false;

    while(true){
        if(!sent){
            t.start();
            client->Write("send\r\n");
            cloud_buffers.clear();
            sent = true;
        }

        while(cloud_buffers.size() < 614400){
            int len = client->Read();
            if(len <= 0){
                exit_app_ = true;
                break;
            }
            cloud_buffers.insert(cloud_buffers.end(), client->recBuffer, client->recBuffer+len);
            sent = false;
        }
        if(exit_app_)
            break;

        if(cloud_buffers.size() > 0){
            
            std::cout <<  "sensor: " << t.elapsedMilliseconds() << std::endl;
            t.start();

            std::unique_lock<std::mutex> updateLock(updateModelMutex_);

            CopyBuffersToPointCloud();

            ProcessPointCloud();

            std::cout <<  "cloud proc: " << t.elapsedMilliseconds() << std::endl;

            updateLock.unlock();

            if(exit_app_)
                break;

            update_ = true;
            
        }

        if(exit_app_)
            break;
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



void DisplayCloud()
{
    std::cout <<  "Waiting for initial data.\n";
    while(!update_){
        std::this_thread::sleep_for(10ms);
    }

    std::cout <<  "Visualizer starting...." << std::endl;
    pcl::visualization::PCLVisualizer *viewer (new pcl::visualization::PCLVisualizer());
    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> handler(cloud_,"intensity");

    viewer->addPointCloud<pcl::PointXYZ>(cloudOut_,"id");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "id");

    viewer->initCameraParameters ();



viewer->spinOnce(100);

    while (!viewer->wasStopped()){

        if(update_)
        {
            std::unique_lock<std::mutex> updateLock(updateModelMutex_);
  
            viewer->updatePointCloud<pcl::PointXYZ>(cloudOut_, "id");

            if(cloudNormals_->points.size() > 0){
                viewer->removePointCloud("normals");
                viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloudOut_, cloudNormals_, 100, (0.1F), "normals");    
            }

            update_ = false;

            updateLock.unlock();
        }

        viewer->spinOnce(1);

        if(exit_app_)
            viewer->close();
    }

    exit_app_ = true;    
}

int
main()
{
    exit_app_ = false;
    update_ = false;

    std::thread client(Connect);
    std::thread display(DisplayCloud);

    display.join();
    client.join();

    return 0;
}