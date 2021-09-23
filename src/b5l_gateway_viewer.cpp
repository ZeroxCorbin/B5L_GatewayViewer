#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "clsTCPSocket.h"

#include <unistd.h>
#include <iostream>
#include <ostream>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

const char *FilePath="/home/zeroxcorbin/file_in.pcd";

int user_data;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
std::mutex updateModelMutex_;
bool update_;

void
viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    std::cout << "i only run once" << std::endl;

}

void
viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}

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
            pcl::io::loadPCDFile<pcl::PointXYZI> (FilePath, *cloud_);
            update_ = false;
            updateLock.unlock();
        }

        //std::this_thread::sleep_for(1000ms);
    }
}

void
Connect(){
    clsTCPSocket client;

    client.NameIP = "127.0.0.1";
    client.Port = 8890;

	if(!client.Configure()){
		return;
	}

    if(client.Connect()){
        std::thread client1(RecieveData, &client);
        client1.join();
    }
}
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;
  using CloudPtr = typename Cloud::Ptr;
  using CloudConstPtr = typename Cloud::ConstPtr;


int
main()
{

    // pcl::io::loadPCDFile("file.pcd", *cloud);
    // fprintf(stderr, "H:[%d] W:[%d]\n", cloud->height, cloud->width);
 pcl::visualization::CloudViewer viewer_ ("3D Viewer");
   
    // //blocks until the cloud is actually rendered
    // viewer.showCloud(cloud);
    
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer

    //This will only get called once
    //viewer.runOnVisualizationThreadOnce(viewerOneOff);

    //This will get called once per visualization iteration
    //viewer.runOnVisualizationThread(viewerPsycho);

    std::thread client(Connect);
 
    // prepare visualizer named "viewer"
    while (!viewer_.wasStopped ())
    {
        std::this_thread::sleep_for(100ms);

        // Get lock on the boolean update and check if cloud was updated
        std::unique_lock<std::mutex> updateLock(updateModelMutex_);
        if(update_)
        {
            // CloudPtr temp_cloud;
            // temp_cloud.swap (cloud_); //here we set cloud_ to null, so that
std::cout << "cloud update" << std::endl;
            viewer_.showCloud(cloud_);

            update_ = false;
        }
        updateLock.unlock();

    } 
    
    client.join();

    return 0;
}