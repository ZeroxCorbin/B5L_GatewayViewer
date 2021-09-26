
#include <iostream>
#include <ostream>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

#if WIN32
#include <mutex>
#else
#include <unistd.h>
#endif

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "clsTCPSocket.hpp"
#include "clsTimer.hpp"
#include "CloudProcessing.hpp"