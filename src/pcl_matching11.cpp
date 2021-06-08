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

typedef PointMatcher<float> PM;

class listener
{
public:
    PM::DataPoints scene;
    PM::DataPoints object;
    sensor_msgs::PointCloud2 transformed_pcd;
    
    void targCB(const sensor_msgs::PointCloud2 &input);
    void scanCB(const sensor_msgs::PointCloud2 &input);
    void perform_icp();
};

void listener::targCB(const sensor_msgs::PointCloud2 &input)
{
    scene = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(input, false);
    
    pcl::PointCloud<pcl::PointXYZ> cloud_scan;
    pcl::fromROSMsg(input, cloud_scan);
    pcl::io::savePCDFileASCII ("2.pcd", cloud_scan);
}

void listener::scanCB(const sensor_msgs::PointCloud2 &input)
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
    pcl::io::savePCDFileASCII ("1.pcd", cloud_scan);
    object = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(output, false);

}

void listener::perform_icp()        
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

        
        PM::TransformationParameters T = icp(object,scene);

        std::cout << "Transformation Matrix = \n" << T << std::endl;
        PM::DataPoints transformed_object(object);
        icp.transformations.apply(transformed_object, T);

        transformed_pcd = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(transformed_object, "/camera_init", ros::Time::now());
        // pcl_pub_aligned.publish(transformed_pcd);

}

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_matching");
    ros::NodeHandle nh;

    listener L;

    ros::Rate loop_rate(30);

    ros::Subscriber pcl_sub_targ = nh.subscribe("pcl_target", 10, &listener::targCB, &L);
    ros::Subscriber pcl_sub_scan = nh.subscribe("pcl_scan", 10, &listener::scanCB, &L);

    ros::Publisher pcl_pub_aligned = nh.advertise<sensor_msgs::PointCloud2>("pcl_aligned", 1);

    while(ros::ok())
    {
        ros::spinOnce();
        L.perform_icp();
        pcl_pub_aligned.publish(L.transformed_pcd);
        ROS_INFO("This is theta1f:");
        // ROS_INFO("This is theta2f: %.2f", L.object);
        loop_rate.sleep();
    }

    return 0;
}
