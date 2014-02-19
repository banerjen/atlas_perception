////////////////////////////////////////
// Laser point cloud assembler        //
// Sensor : Hokuyo LIDAR (ATLAS Head) //
// By     : Nandan Banerjee           //
// Date   : 18th of February, 2014    //
////////////////////////////////////////

// Write a subscriber on topic /laser/points2
// Get the spindle speed by reading from the rosparam /multisense_sl/motor_speed

// Test case - spindle speed is 3.14
// Number of scans in one complete rotation - 80

#include "basic_assembler.h"

void shutdown(int sig)
{
    g_shutdown = true;
}

void threadSpinSubscriptions()
{
    ros::Rate r(100);    // Nyquist Sampling frequency f[=100] >= 2*BW[=(2*scan_rate)=(2*40)=80]
    while (ros::ok())
    {
        ros::spinOnce();                   // Handle ROS events
        r.sleep();
    }
}

void storeCloudMessages(sensor_msgs::PointCloud2 msg_cloud)
{
    LaserPointCloud*    msg_pcl;

    try
    {
        ROS_INFO_ONCE("Started!");
        msg_pcl = new LaserPointCloud();
        pcl::fromROSMsg(msg_cloud, *msg_pcl);
        g_cloud_message = *msg_pcl;
        g_merge_cloud_signal = true;
    }
    catch (std::exception ex)
    {
        ROS_WARN_STREAM("Error getting messages. Dropping scans!" << ex.what());
    }

    if (g_shutdown == true)
        exit(0);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "basic_assembler");
    ros::NodeHandle             n_Handle;
    ros::Subscriber             subscriber_point_cloud;
    ros::Publisher              publisher_merged_point_cloud;
    LaserPointCloud             merged_point_cloud;
    sensor_msgs::PointCloud2    msg_point_cloud_output;
    unsigned int                         cloud_count = 0;

    g_shutdown = false;
    g_merge_cloud_signal = false;
    publisher_merged_point_cloud = n_Handle.advertise<sensor_msgs::PointCloud2>("/banerjee/points2", 5);
    subscriber_point_cloud = n_Handle.subscribe("/laser/points2", 1, storeCloudMessages);
    boost::thread spinning_thread(threadSpinSubscriptions);

    signal(SIGINT, shutdown);

    ROS_INFO("Starting!");

    ros::Rate loop_rate(10000);

    merged_point_cloud.header.frame_id = "/left_camera_optical_frame";
    merged_point_cloud.height = 1;
    merged_point_cloud.width = 1081*400;
    merged_point_cloud.points.resize(400 * 1081);

    cloud_count = 0;

    while (g_shutdown == false)
    {
        if (g_merge_cloud_signal == true)
        {
            g_merge_cloud_signal = false;

            for (int i = 0; i < 1081; i++)
            {
                merged_point_cloud.points.at(cloud_count*1081 + i).x = g_cloud_message.points.at(i).x;
                merged_point_cloud.points.at(cloud_count*1081 + i).y = g_cloud_message.points.at(i).y;
                merged_point_cloud.points.at(cloud_count*1081 + i).z = g_cloud_message.points.at(i).z;
            }

            cloud_count++;
            if (cloud_count >= 400)
                cloud_count = 0;

            pcl::toROSMsg(merged_point_cloud, msg_point_cloud_output);
            publisher_merged_point_cloud.publish(msg_point_cloud_output);
        }
        loop_rate.sleep();
    }

    ros::shutdown();

    return 0;
}
