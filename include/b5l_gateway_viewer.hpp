
#include <iostream>
#include <ostream>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

#if WIN32
#include <mutex>
//#pragma comment(lib,"pthreadVC3.lib")
#else
#pragma comment(lib,"pthread")
#include <unistd.h>
#endif

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/bilateral.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "clsTCPSocket.hpp"
