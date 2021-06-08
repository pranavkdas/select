#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

class gpsHandler
{
public:
    gpsHandler()
    {
        gps_sub = nh.subscribe("/kitti/oxts/gps/fix", 10, &gpsHandler::gpsCB, this);
        // coords_sub = nh.subscribe("/kitti/oxts/gps/fix", 10, &gpsHandler::gpsCB, this);
        // pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_partitioned", 1);
    }

    void gpsCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_partitioned;
        
        pcl::fromROSMsg(input, cloud);
        
        sensor_msgs::PointCloud2 output;

        float resolution = 128.0f;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

        octree.setInputCloud (cloud.makeShared());
        octree.addPointsFromInputCloud ();

        pcl::PointXYZ center_point;
        center_point.x = 0 ;
        center_point.y = -1;
        center_point.z = 50;

        float radius = 40;
        std::vector<int> radiusIdx;
        std::vector<float> radiusSQDist;
        if (octree.radiusSearch (center_point, radius, radiusIdx, radiusSQDist) > 0)
        {
            for (size_t i = 0; i < radiusIdx.size (); ++i)
            {
                cloud_partitioned.points.push_back(cloud.points[radiusIdx[i]]);
            }
        }

        pcl::toROSMsg(cloud_partitioned, output);
        output.header.frame_id = "/camera_init";
        pcl_pub.publish(output);
    }


protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

main (int argc, char **argv)
{
    ros::init (argc, argv, "gps_dict");

    gpsHandler handler;

    ros::spin();

    return 0;
}
