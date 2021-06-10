#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <select_pcd/updated_coord.h>

float a,b,c;
int p = 0;
class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("pcl_map", 10, &cloudHandler::cloudCB, this);
        coords_sub = nh.subscribe("updated_coord", 10, &cloudHandler::coordsCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_target", 1);
    }

    void coordsCB(const select_pcd::updated_coord &input){
        a = input.x;
        b = input.y;
        c = input.z;
        
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
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
        if (p==0){
            center_point.x = 0;
            center_point.y = -1;
            center_point.z = 40; 
            p=1;
        }
        else{
            center_point.x = a;
            center_point.y = b;
            center_point.z = c; // Just for testing.
            // center_point.z = 50;
            std::cout << center_point.x << "\n";
        }

        

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
    ros::Subscriber coords_sub;
    ros::Publisher pcl_pub;
};

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_partition");

    cloudHandler handler;

    ros::spin();

    return 0;
}
