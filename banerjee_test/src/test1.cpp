#include <ros/ros.h>

//#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl-1.6/pcl/ros/conversions.h"
#include "pcl-1.6/pcl/point_cloud.h"
#include "pcl-1.6/pcl/point_types.h"
#include "pcl-1.6/pcl/common/eigen.h"
//#include "pcl-1.6/pcl/common/io.h"
#include "pcl-1.6/pcl/filters/voxel_grid.h"
//#include "pcl-1.6/pcl/filters/filter.h"
//#include "pcl-1.6/pcl/filters/extract_indices.h"

//#include "pcl_ros/transforms.h"
//#include "pcl_ros/point_cloud.h"

ros::Publisher pub;

pcl::PointCloud<pcl::PointXYZRGB> PC;
pcl::PointCloud<pcl::PointXYZRGB> PC_filtered;

//using namespace pcl;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	sensor_msgs::PointCloud2 cloud_filtered;

	// Perform the actual filtering
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	sor.setInputCloud (input);
	sor.setLeafSize (0.01, 0.01, 0.01);
	sor.filter (cloud_filtered);



	// Publish the data
	pub.publish (cloud_filtered);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "pc_test");
	ros::NodeHandle nHandle;
	ros::Subscriber	pcSub;

	pcSub = nHandle.subscribe("/multisense_sl/camera/points2", 1, cloud_cb);
	pub   = nHandle.advertise<sensor_msgs::PointCloud2>("/banerjee_output", 1);

	ros::spin();

	return 0;
}
