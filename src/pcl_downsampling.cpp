#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub_targ = nh.subscribe("pcl_filtered_targ", 10, &cloudHandler::cloudCB_targ, this);
        pcl_sub_scan = nh.subscribe("pcl_filtered_scan", 10, &cloudHandler::cloudCB_scan, this);
        pcl_pub_targ = nh.advertise<sensor_msgs::PointCloud2>("pcl_downsampled_targ", 1);
        pcl_pub_scan = nh.advertise<sensor_msgs::PointCloud2>("pcl_downsampled_scan", 1);
    }

    void cloudCB_targ(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_downsampled;
        sensor_msgs::PointCloud2 output;

        pcl::fromROSMsg(input, cloud);

        pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
        voxelSampler.setInputCloud(cloud.makeShared());
        // voxelSampler.setLeafSize(0.05f, 0.05f, 0.05f);
        voxelSampler.setLeafSize(0.1f, 0.1f, 0.1f);
        voxelSampler.filter(cloud_downsampled);

        pcl::toROSMsg(cloud_downsampled, output);
        pcl_pub_targ.publish(output);

    }

    void cloudCB_scan(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_downsampled;
        sensor_msgs::PointCloud2 output;

        pcl::fromROSMsg(input, cloud);

        pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
        voxelSampler.setInputCloud(cloud.makeShared());
        voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
        voxelSampler.filter(cloud_downsampled);

        pcl::toROSMsg(cloud_downsampled, output);
        pcl_pub_scan.publish(output);

    }
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub_targ;
    ros::Subscriber pcl_sub_scan;
    ros::Publisher pcl_pub_targ;
    ros::Publisher pcl_pub_scan;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_downsampling");

    cloudHandler handler;

    ros::spin();

    return 0;
}

