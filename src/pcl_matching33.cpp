#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <iostream>
using namespace std;

pcl::PointCloud<pcl::PointXYZ> cloud_targ;
pcl::PointCloud<pcl::PointXYZ> cloud_scan;
pcl::PointCloud<pcl::PointXYZ> cloud_aligned;

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub_targ = nh.subscribe("pcl_downsampled_targ", 10, &cloudHandler::cloudCB_targ, this);
        pcl_sub_scan = nh.subscribe("pcl_downsampled_scan", 10, &cloudHandler::cloudCB_scan, this);
        pcl_pub_scan = nh.advertise<sensor_msgs::PointCloud2>("pcl_scan_altered", 1);
        pcl_pub_aligned = nh.advertise<sensor_msgs::PointCloud2>("pcl_aligned", 1);
    }

    void cloudCB_targ(const sensor_msgs::PointCloud2 &input)
    {


    // pcl::PointCloud<pcl::PointXYZ> MapCloud;
    // pcl::PointCloud<pcl::PointXYZ> ScanCloud;

        pcl::io::loadPCDFile ("/home/pranavkdas/Libraries/libpointmatcher/build/seg_pure_11.pcd", cloud_targ);

        // pcl::fromROSMsg(input, cloud_targ);

    }

    void cloudCB_scan(const sensor_msgs::PointCloud2 &input)
    {
        sensor_msgs::PointCloud2 output;

        pcl::io::loadPCDFile ("/home/pranavkdas/Libraries/libpointmatcher/build/seg_pure_10.pcd", cloud_scan);
        // pcl::fromROSMsg(input, cloud_scan);

        for (size_t i = 0; i < cloud_scan.points.size (); ++i)
        {
            cloud_scan.points[i].x = cloud_scan.points[i].x + 10;
            // cloud_scan.points[i].x = cloud_scan.points[i].y + 5;
        }

        pcl::toROSMsg(cloud_scan, output);
        pcl_pub_scan.publish(output);

        if ((cloud_scan.points.size()>0) && (cloud_targ.points.size()>0)){
            cloudHandler::perform_icp();
        }
    }

    void perform_icp()        
    {   
        sensor_msgs::PointCloud2 output;

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_scan.makeShared());
        icp.setInputTarget(cloud_targ.makeShared());
 
        icp.setMaxCorrespondenceDistance(30);
        icp.setMaximumIterations(100000);
        icp.setTransformationEpsilon (1e-10);
        icp.setEuclideanFitnessEpsilon(0.1);

        icp.align(cloud_aligned);

        Eigen::Matrix4f transformation = icp.getFinalTransformation ();
        cout<<transformation;
        pcl::toROSMsg(cloud_aligned, output);
        pcl_pub_aligned.publish(output);
    } 

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub_targ;
    ros::Subscriber pcl_sub_scan;
    ros::Publisher pcl_pub_scan;
    ros::Publisher pcl_pub_aligned;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_matching");

    cloudHandler handler;

    ros::spin();

    return 0;
}
