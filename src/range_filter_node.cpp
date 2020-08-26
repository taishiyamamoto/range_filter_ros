#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

class RangeFilter{
    public:
        RangeFilter(){
            sub_ = nh_.subscribe<sensor_msgs::PointCloud2> ("velodyne_points", 1, &RangeFilter::cloud_cb,this);
            pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("filter_cloud", 1);
        }
    private: 
        void cloud_cb(const sensor_msgs::PointCloud2 cloud_msg);
        void pass_through_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,bool LimitsNegative,const std::string &FieldName,float min_Limits,float max_Limits);
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
};

void 
RangeFilter::cloud_cb(const sensor_msgs::PointCloud2 cloud_msg){
    
    //pcl to rosmsg
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(cloud_msg, *cloud);

    pass_through_filter(cloud, false, "x", -2.0,2.0);
    pass_through_filter(cloud, false, "y", -2.0,2.0);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_filter_cloud;
    pcl::toROSMsg(*cloud, output_filter_cloud);
    output_filter_cloud.header = cloud_msg.header;

    // Publish the data
    pub_.publish (output_filter_cloud);

}

void
RangeFilter::pass_through_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,bool LimitsNegative,const std::string &FieldName,float min_Limits,float max_Limits){
    pcl::PassThrough<pcl::PointXYZI> pass_;
	pass_.setInputCloud(cloud);
	pass_.setFilterLimitsNegative(LimitsNegative);
	pass_.setFilterFieldName(FieldName);
	pass_.setFilterLimits(min_Limits,max_Limits);
	pass_.filter(*cloud);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"range_filter");

    RangeFilter rf;

    ros::spin();
    return 0;
}