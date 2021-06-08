#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub_targ = nh.subscribe("pcl_target", 10, &cloudHandler::cloudCB_targ, this);
        pcl_sub_scan = nh.subscribe("pcl_scan", 10, &cloudHandler::cloudCB_scan, this);
        pcl_pub_targ = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered_targ", 1);
        pcl_pub_scan = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered_scan", 1);
    }

    void cloudCB_targ(const sensor_msgs::PointCloud2& input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        sensor_msgs::PointCloud2 output;

        pcl::fromROSMsg(input, cloud);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
        statFilter.setInputCloud(cloud.makeShared());
        statFilter.setMeanK(10);
        statFilter.setStddevMulThresh(0.2);
        statFilter.filter(cloud_filtered);

        pcl::toROSMsg(cloud_filtered, output);
        pcl_pub_targ.publish(output);
    }

    void cloudCB_scan(const sensor_msgs::PointCloud2& input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        sensor_msgs::PointCloud2 output;

        pcl::fromROSMsg(input, cloud);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
        statFilter.setInputCloud(cloud.makeShared());
        statFilter.setMeanK(10);
        statFilter.setStddevMulThresh(0.2);
        statFilter.filter(cloud_filtered);

        pcl::toROSMsg(cloud_filtered, output);
        pcl_pub_scan.publish(output);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub_targ;
    ros::Subscriber pcl_sub_scan;
    ros::Publisher pcl_pub_targ;
    ros::Publisher pcl_pub_scan;
};

main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_filter");

    cloudHandler handler;

    ros::spin();

    return 0;
}

