#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

//main(int argc, char **argv)
//{
    //ros::init (argc, argv, "pcl_read");

    

class pcl_reader
{    
public:
    pcl_reader(){
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("os1_points", 1);
    ros::Subscriber pcl_sub = nh.subscribe("/os1_cloud_node/points", 1, &pcl_reader::cloud_CB, this);
    
    }

    //CALLBACK FUNCTION FOR PCL_SUB
    void cloud_CB(const sensor_msgs::PointCloud2 &input){
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(input, cloud);

        sensor_msgs::PointCloud2 read_output;

        pcl::toROSMsg(cloud, read_output);

        read_output2.header.frame_id = "/base_link";

        pcl_pub.publish(read_output);



    }
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
    ros::Publisher pcl_pub_2;
};

//}

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_read");
    pcl_reader reader;
    ros::spin();

    return 0;
}


