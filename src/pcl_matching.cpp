#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include "pointmatcher/PointMatcher.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"
// pcl::PointCloud<pcl::PointXYZ> cloud_targ;
// pcl::PointCloud<pcl::PointXYZ> cloud_scan;
// pcl::PointCloud<pcl::PointXYZ> cloud_aligned;
#include <visualization_msgs/Marker.h>
#include <select_pcd/updated_coord.h>

typedef PointMatcher<float> PM;

PM::DataPoints scene;
PM::DataPoints object;
pcl::PointCloud<pcl::PointXYZ> cloud_targ;

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub_targ = nh.subscribe("pcl_target", 1, &cloudHandler::cloudCB_targ, this);
        pcl_sub_scan = nh.subscribe("pcl_scan", 1, &cloudHandler::cloudCB_scan, this);
        pcl_pub_scan = nh.advertise<sensor_msgs::PointCloud2>("pcl_scan_altered", 1);
        pcl_pub_aligned = nh.advertise<sensor_msgs::PointCloud2>("pcl_aligned", 1);
        marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        coords_pub = nh.advertise<select_pcd::updated_coord>("updated_coord",1);
    }

    void cloudCB_targ(const sensor_msgs::PointCloud2 &input)
    {

        pcl::fromROSMsg(input, cloud_targ);
        scene= PointMatcher_ros::rosMsgToPointMatcherCloud<float>(input, false);


    }

    void cloudCB_scan(const sensor_msgs::PointCloud2 &input)
    {
        sensor_msgs::PointCloud2 output;

        pcl::PointCloud<pcl::PointXYZ> cloud_scan;
        
        pcl::fromROSMsg(input, cloud_scan);

        for (size_t i = 0; i < cloud_scan.points.size (); ++i)
        {
            cloud_scan.points[i].x = cloud_scan.points[i].x + 10;
            // cloud_scan.points[i].x = cloud_scan.points[i].y + 5;
        }

        pcl::toROSMsg(cloud_scan, output);
        pcl_pub_scan.publish(output);

        object= PointMatcher_ros::rosMsgToPointMatcherCloud<float>(input, false);

        

        // pcl::toROSMsg(cloud_scan, output);
        // pcl_pub_scan.publish(output);

        // if ((cloud_scan.points.size()>0)){
        //     cloudHandler::perform_icp();
        // }
    // }

    // void perform_icp()        
    // {   
        // sensor_msgs::PointCloud2 transformed_pcd = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(::object, "/camera_init", ros::Time::now());
        // pcl_pub_aligned.publish(transformed_pcd);
    if(cloud_scan.points.size() > 0 && cloud_targ.points.size()>0)
    {
        PM::ICP icp;
        

        PointMatcherSupport::Parametrizable::Parameters params;
        std::string name;
        
        // Uncomment for console outputs
        setLogger(PM::get().LoggerRegistrar.create("FileLogger"));

        // Prepare reading filters
        name = "MinDistDataPointsFilter";
        params["minDist"] = "1.0";
        std::shared_ptr<PM::DataPointsFilter> minDist_read =
            PM::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();

        name = "RandomSamplingDataPointsFilter";
        params["prob"] = "0.05";
        std::shared_ptr<PM::DataPointsFilter> rand_read =
            PM::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();

        // Prepare reference filters
        name = "MinDistDataPointsFilter";
        params["minDist"] = "1.0";
        std::shared_ptr<PM::DataPointsFilter> minDist_ref =
            PM::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();

        name = "RandomSamplingDataPointsFilter";
        params["prob"] = "0.05";
        std::shared_ptr<PM::DataPointsFilter> rand_ref =
            PM::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();

        // Prepare matching function
        name = "KDTreeMatcher";
        params["knn"] = "1";
        params["epsilon"] = "3.16";
        std::shared_ptr<PM::Matcher> kdtree =
            PM::get().MatcherRegistrar.create(name, params);
        params.clear();

        // Prepare outlier filters
        name = "TrimmedDistOutlierFilter";
        params["ratio"] = "0.75";
        std::shared_ptr<PM::OutlierFilter> trim =
            PM::get().OutlierFilterRegistrar.create(name, params);
        params.clear();

        // Prepare error minimization
        name = "PointToPointErrorMinimizer";
        std::shared_ptr<PM::ErrorMinimizer> pointToPoint =
            PM::get().ErrorMinimizerRegistrar.create(name);

        // Prepare transformation checker filters
        name = "CounterTransformationChecker";
        params["maxIterationCount"] = "150";
        std::shared_ptr<PM::TransformationChecker> maxIter =
            PM::get().TransformationCheckerRegistrar.create(name, params);
        params.clear();

        name = "DifferentialTransformationChecker";
        params["minDiffRotErr"] = "0.001";
        params["minDiffTransErr"] = "0.01";
        params["smoothLength"] = "4";
        std::shared_ptr<PM::TransformationChecker> diff =
            PM::get().TransformationCheckerRegistrar.create(name, params);
        params.clear();

        // Prepare inspector
        std::shared_ptr<PM::Inspector> nullInspect =
            PM::get().InspectorRegistrar.create("NullInspector");

        //  name = "VTKFileInspector";
        //  params["dumpDataLinks"] = "1";
        //  params["dumpReading"] = "1";
        //  params["dumpReference"] = "1";
        //  std::shared_ptr<PM::Inspector> vtkInspect =
        //      PM::get().InspectorRegistrar.create(name, params);
        //  params.clear();

        // Prepare transformation
        std::shared_ptr<PM::Transformation> rigidTrans =
            PM::get().TransformationRegistrar.create("RigidTransformation");
        
        // Build ICP solution
        icp.readingDataPointsFilters.push_back(minDist_read);
        icp.readingDataPointsFilters.push_back(rand_read);

        icp.referenceDataPointsFilters.push_back(minDist_ref);
        icp.referenceDataPointsFilters.push_back(rand_ref);

        icp.matcher = kdtree;
        
        icp.outlierFilters.push_back(trim);
        
        icp.errorMinimizer = pointToPoint;

        icp.transformationCheckers.push_back(maxIter);
        icp.transformationCheckers.push_back(diff);
        
        // toggle to write vtk files per iteration
        icp.inspector = nullInspect;
        //icp.inspector = vtkInspect;

        icp.transformations.push_back(rigidTrans);

        
        PM::TransformationParameters T = icp(object, scene);

        // Eigen::Transform<float, 3, Eigen::Affine> tROTA(T);

        // float x, y, z, roll, pitch, yaw;

        // pcl::getTranslationAndEulerAngles(tROTA, x, y, z, roll, pitch, yaw);

        std::cout << "Transformation Matrix = \n" << T << std::endl;
        // std::cout << tROTA << " " << x << std::endl;
        PM::DataPoints transformed_object(object);
        icp.transformations.apply(transformed_object, T);

        sensor_msgs::PointCloud2 transformed_pcd = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(transformed_object, "/camera_init", ros::Time::now());
        
        // To find current position
        pcl::PointCloud<pcl::PointXYZ> hope;
        
        pcl::fromROSMsg(transformed_pcd, hope);
        Eigen::Vector4f centroid;
    
        pcl::compute3DCentroid(hope, centroid);
        std::cout << "The XYZ coordinates of the centroid are: ("
        << centroid[0] << ", "
        << centroid[1] << ", "
        << centroid[2] << ")." << std::endl;

        select_pcd::updated_coord new_msg;
        new_msg.x = centroid[0];
        new_msg.y = centroid[1];
        new_msg.z = centroid[2];

        coords_pub.publish(new_msg);
          

        // Set our initial shape type to be a cube
        uint32_t shape = visualization_msgs::Marker::CUBE;

        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/camera_init";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = 0;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = centroid[0];
        marker.pose.position.y = centroid[1];
        marker.pose.position.z = centroid[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        marker_pub.publish(marker);
        
        // Another method to find centroid ///

        // float a,b,c;
        // for (size_t i = 0; i < hope.points.size (); ++i)
        // {
        //     a += hope.points[i].x;
        //     b += hope.points[i].y;
        //     c += hope.points[i].z;
        //     // cloud_scan.points[i].x = cloud_scan.points[i].y + 5;
        // }

        // std::cout << a/(hope.points.size()) << " " << b/(hope.points.size()) << " " << c/(hope.points.size()) << std::endl;
        pcl_pub_aligned.publish(transformed_pcd);
    }

        ///////////////////////////////////////////
        // sensor_msgs::PointCloud2 output;

        // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        // icp.setInputSource(cloud_scan.makeShared());
        // icp.setInputTarget(cloud_targ.makeShared());
 
        // icp.setMaxCorrespondenceDistance(30);
        // icp.setMaximumIterations(10000);
        // icp.setTransformationEpsilon (1e-12);
        // icp.setEuclideanFitnessEpsilon(0.1);

        // icp.align(cloud_aligned);

        // pcl::toROSMsg(cloud_aligned, output);
        // pcl_pub_aligned.publish(output);
    } 

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub_targ;
    ros::Subscriber pcl_sub_scan;
    ros::Publisher pcl_pub_scan;
    ros::Publisher pcl_pub_aligned;
    ros::Publisher marker_pub;
    ros::Publisher coords_pub;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_matching");

    cloudHandler handler;

    ros::spin();

    return 0;
}







