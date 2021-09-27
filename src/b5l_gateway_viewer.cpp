#include "b5l_gateway_viewer.hpp"

#if WIN32
const char *FilePath = "C:\\Users\\jack\\Dropbox\\Projects\\Cpp\\B5L_"
                       "GatewayViewer\\build\\Debug\\file_in.pcd";
#else
const char *FilePath = "/home/zeroxcorbin/file_in.pcd";
#endif

pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloudIn_(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    cloudOut_(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::Normal>::Ptr
    cloudNormals_(new pcl::PointCloud<pcl::Normal>);
std::vector<char> cloud_buffers; // (new std::vector<char>);
pcl::Indices
    *selectedIndices(new pcl::Indices);

std::mutex selectedIndicesMutex_;

std::mutex updateModelMutex_;
bool update_;
bool exit_app_;

void CopyBuffersToPointCloud()
{
    /* The result format should be 0x001 or 0x002 (PCD) */

    pcl::PointXYZ currPoint;

    cloudIn_->points.clear();

    short *pXYZ; //,*pAmplitude;
    int i;

    pXYZ = (short *)(&cloud_buffers[0]);
    // pAmplitude = (short *)(&cloud_buffers[0] + (6 * 76800));

    /* QVGA Loop */
    for (i = 0; i < 76800; i++)
    {
        currPoint.x = (float)*pXYZ++ / 1000;
        currPoint.y = (float)*pXYZ++ / 1000;
        currPoint.z = (float)*pXYZ++ / 1000;
        // currPoint.intensity = *pAmplitude++;

        cloudIn_->points.push_back(currPoint);
    }
}

bool SaveImageFile(std::vector<unsigned char> fileBytes)
{

    std::ofstream file(FilePath, std::ios::out | std::ios::binary);
    if (!file.fail())
    {
        std::copy(fileBytes.cbegin(), fileBytes.cend(),
                  std::ostream_iterator<unsigned char>(file));
        file.close();

        return true;
    }
    else
    {
        std::cout << "Writing file failed!" << std::endl;

        return false;
    }
}

bool ProcessNormals_ = false;

void ProcessPointCloud()
{


    // pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughCloud (
    //     new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud (new
    // pcl::PointCloud<pcl::PointXYZ>());;

    // pcl::PointCloud<pcl::Normal>::Ptr normalsCloud (new
    // pcl::PointCloud<pcl::Normal>);

    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr inliersCloud (new
    // pcl::PointCloud<pcl::PointXYZ>());

    // pcl::PointCloud<pcl::PointXYZ>::Ptr ransacCloud (new
    // pcl::PointCloud<pcl::PointXYZ>());;

    //PassThroughFilter(cloudIn_, passThroughCloud);


    //pcl::copyPointCloud(*passThroughCloud, *cloudOut_);




    cloudOut_->points.clear();
    for(int i =0; i < cloudIn_->points.size(); i++){
        pcl::PointXYZRGB p;
        p.x = cloudIn_->points[i].x;
        p.y = cloudIn_->points[i].y;
        p.z = cloudIn_->points[i].z;
        p.r = 255;
        p.g = 255;
        p.b = 255;

        cloudOut_->points.push_back(p);
    }

    std::unique_lock<std::mutex> updateLock(selectedIndicesMutex_);
    for(int i =0; i < selectedIndices->size(); i++){
        cloudOut_->points[(*selectedIndices)[i]].r = 255;
        cloudOut_->points[(*selectedIndices)[i]].g = 25;
        cloudOut_->points[(*selectedIndices)[i]].b = 25;
    }
    updateLock.unlock();

    if (ProcessNormals_)
    {
        NormalEstimationOMP(cloudOut_, cloudNormals_);
    }
    else
    {
        cloudNormals_->points.clear();
    }
    // updateLock.unlock();
    // VoxelGrid(passThroughCloud, voxelCloud);

    // SACSegmentation(passThroughCloud, coefficients, inliers);
    // pcl::copyPointCloud<pcl::PointXYZ>(*passThroughCloud, *inliers,
    // *inliersCloud); viewer->removeShape("plane1");
    // viewer->addPlane(*coefficients, "plane1");

    // RandomSampleConsensus(passThroughCloud, ransacCloud)

}

void RecieveData(clsTCPSocket *client)
{
    Timer t;
    bool sent = false;

    while (true)
    {
        if (!sent)
        {
            t.start();
            client->Write("send\r\n");
            cloud_buffers.clear();
            sent = true;
        }

        while (cloud_buffers.size() < 614400)
        {
            int len = client->Read();
            if (len <= 0)
            {
                exit_app_ = true;
                break;
            }
            cloud_buffers.insert(cloud_buffers.end(), client->recBuffer,
                                 client->recBuffer + len);
            sent = false;
        }
        
        if (exit_app_)
            break;

        if (cloud_buffers.size() > 0)
        {

            std::cout << "sensor: " << t.elapsedMilliseconds() << std::endl;
            t.start();

            std::unique_lock<std::mutex> updateLock(updateModelMutex_);

            CopyBuffersToPointCloud();

            ProcessPointCloud();

            std::cout << "cloud proc: " << t.elapsedMilliseconds() << std::endl;

            update_ = true;
            updateLock.unlock();
        }

        if (exit_app_)
            break;
    }
}

void Connect()
{
    clsTCPSocket *client(new clsTCPSocket);

    client->NameIP = "192.168.0.123";
    client->Port = 8890;

    if (!client->Configure())
    {
        return;
    }

    if (client->Connect())
    {
        std::thread client1(RecieveData, client);
        client1.join();
    }

    delete client;
}

unsigned int text_id = 0;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void *viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer =
        static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
    if (event.getKeySym() == "n" && event.keyDown())
    {
        ProcessNormals_ ? ProcessNormals_ = false : ProcessNormals_ = true;
    }
}

void mouseEventOccurred(const pcl::visualization::MouseEvent &event,
                        void *viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer =
        static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
    if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
        event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        std::cout << "Left mouse button released at position (" << event.getX()
                  << ", " << event.getY() << ")" << std::endl;

        //char str[512];
        //sprintf_s(str, "text#%03d", text_id++);
        //viewer->addText("clicked here", event.getX(), event.getY(), str);
    }
}

void pointPickingEventOccurred (const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
  float x, y, z;
  if (event.getPointIndex () == -1)
  {
     return;
  }
  event.getPoint(x, y, z);
  std::cout << "[INOF] Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;
}
void areaPickingEventOccurred (const pcl::visualization::AreaPickingEvent& event, void* viewer_void)
{
    std::unique_lock<std::mutex> updateLock(selectedIndicesMutex_);

    event.getPointsIndices(*selectedIndices);

    updateLock.unlock();
}

void DisplayCloud()
{
    std::cout << "Visualizer starting...." << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer());
    // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
    //     *handler (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>(cloudOut_,"intensity"));

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudOut_);

    viewer->initCameraParameters();
    viewer->setShowFPS(false);

    viewer->registerKeyboardCallback (keyboardEventOccurred, (void *)viewer.get());
    viewer->registerMouseCallback (mouseEventOccurred, (void *)viewer.get());
    viewer->registerPointPickingCallback (pointPickingEventOccurred, (void *)viewer.get());
    viewer->registerAreaPickingCallback (areaPickingEventOccurred, (void *)viewer.get());

    Timer t;
    t.start();

    while (!viewer->wasStopped())
    {

        if (update_)
        {
            std::unique_lock<std::mutex> updateLock(updateModelMutex_);

            if (!viewer->updatePointCloud<pcl::PointXYZRGB>(cloudOut_, rgb, "id"))
            {
                viewer->addPointCloud<pcl::PointXYZRGB>(cloudOut_, rgb, "id");
                viewer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "id");
            }

            viewer->removePointCloud("normals");
            if (cloudNormals_->points.size() > 0)
            {
                viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(
                    cloudOut_, cloudNormals_, 100, (0.1F), "normals");
            }

            std::string s = std::to_string(t.elapsedMilliseconds());
            viewer->removeShape("time");
            viewer->addText(s, 1, 1, "time");
            t.start();

            update_ = false;
            updateLock.unlock();
        }

        viewer->spinOnce(50);

        if (exit_app_)
            viewer->close();
    }

    exit_app_ = true;
}

int main()
{
    cloudOut_->width = 320;
    cloudOut_->height = 240;

    exit_app_ = false;
    update_ = false;

    std::thread client(Connect);
    std::thread display(DisplayCloud);

    display.join();
    client.join();

    return 0;
}