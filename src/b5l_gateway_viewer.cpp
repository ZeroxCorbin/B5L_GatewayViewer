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

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
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
            pcl::io::loadPCDFile<pcl::PointXYZI> (FilePath, *cloud_);
            update_ = true;
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

void DisplayCloud()
{
    std::cout <<  "Waiting for initial data.\n";
    while(!update_){
        std::this_thread::sleep_for(10ms);
    }

    std::cout <<  "Visualizer statring...." << std::endl;
    pcl::visualization::PCLVisualizer viewer;
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler(cloud_,"intensity");

    viewer.addPointCloud<pcl::PointXYZI>(cloud_,handler,"id");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "id");

    viewer.initCameraParameters ();

    while (!viewer.wasStopped()){

        // Get lock on the boolean update and check if cloud was updated
        std::unique_lock<std::mutex> updateLock(updateModelMutex_);
        if(update_)
        {
            viewer.updatePointCloud< pcl::PointXYZI >(cloud_,handler,"id");
            update_ = false;
        }
        updateLock.unlock(); 
        
        viewer.spinOnce(10);
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