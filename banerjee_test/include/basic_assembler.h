#include "ros/ros.h"

#include "tf/tf.h"
#include "tf/transform_listener.h"

#include "sensor_msgs/PointCloud2.h"

#include "pcl/io/io.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "boost/thread.hpp"                     // Threading class for subscriber
#include "signal.h"                             // To override ROS's SIGINT and use our own SIGINT handler
#include "exception"
#include "deque"

typedef pcl::PointXYZ						LaserPoint;
typedef pcl::PointCloud<LaserPoint>		 	LaserPointCloud;

bool                                        g_shutdown;
bool                                        g_merge_cloud_signal;
std::deque<LaserPointCloud>                 g_cloud_messages;
LaserPointCloud                             g_cloud_message;

// Constants
const std::string gc_point_cloud_topic      = "/laser/points2";      // Change the name of the topic here
const std::string gc_spindle_speed_param    = "/motor/speed";        // Change the name of the param here

void shutdown(int sig);
void threadSpinSubscriptions();
