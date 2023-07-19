#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <cti_msgs/CarryBoxInfo.h>

struct passThroughRange
{
    std::string axis;
    bool drop;
    bool enableWithBox;
    double min;
    double max;
};

struct CropBoxRange
{
    std::string name;
    bool drop;
    bool enableWithBox;
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
};

ros::Publisher livoxFilteredPub;
std::vector<passThroughRange> passFilterRanges;
std::vector<CropBoxRange> cropBoxRanges;
bool boxAttached = true;

void readFilterRanges()
{

  // 读取YAML文件
  std::string yamlFile = ros::package::getPath("livox_ros_driver2") + "/param/range.yaml";
  try
  {
    YAML::Node config = YAML::LoadFile(yamlFile);

    // 提取过滤范围
    for (const auto& rangeNode : config["passThroughRanges"])
    {
      passThroughRange range;
      range.axis = rangeNode["axis"].as<std::string>();
      range.min = rangeNode["min"].as<double>();
      range.max = rangeNode["max"].as<double>();
      range.drop = rangeNode["drop"].as<bool>();
      range.enableWithBox = rangeNode["enableWithBox"].as<bool>();
      passFilterRanges.push_back(range);
    }
    for (const auto& rangeNode : config["cropBoxRanges"])
    {
      CropBoxRange range;
      range.name = rangeNode["range"]["name"].as<std::string>();
      range.minX = rangeNode["range"]["min"]["x"].as<double>();
      range.minY = rangeNode["range"]["min"]["y"].as<double>();
      range.minZ = rangeNode["range"]["min"]["z"].as<double>();
      range.maxX = rangeNode["range"]["max"]["x"].as<double>();
      range.maxY = rangeNode["range"]["max"]["y"].as<double>();
      range.maxZ = rangeNode["range"]["max"]["z"].as<double>();
      range.drop = rangeNode["range"]["drop"].as<bool>();
      range.enableWithBox = rangeNode["range"]["enableWithBox"].as<bool>();
      cropBoxRanges.push_back(range);
    }
    ROS_INFO_STREAM("passThroughRanges: ");
    for (const auto& range : passFilterRanges)
    {
      ROS_INFO_STREAM("axis: " << range.axis << ", min: " << range.min << ", max: " << range.max << ", drop: " << range.drop << ", enableWithBox: " << range.enableWithBox);
    }
    ROS_INFO_STREAM("cropBoxRanges: ");
    for (const auto& range : cropBoxRanges)
    {
      ROS_INFO_STREAM("name: " << range.name << ", minX: " << range.minX << ", minY: " << range.minY << ", minZ: " << range.minZ << ", maxX: " << range.maxX << ", maxY: " << range.maxY << ", maxZ: " << range.maxZ << ", drop: " << range.drop << ", enableWithBox: " << range.enableWithBox);
    }
  }

  catch (const YAML::Exception& e)
  {
    ROS_ERROR_STREAM("Failed to read filter range YAML file: " << e.msg);
  }
}

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // 转换为PCL点云数据类型
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  *cloudFiltered = *cloud;

  for (const auto& range : passFilterRanges)
  {
    if (range.enableWithBox && !boxAttached)
    {
      continue;
    }
    pcl::PassThrough<pcl::PointXYZ> passThroughFilter;
    passThroughFilter.setInputCloud(cloudFiltered);
    passThroughFilter.setFilterLimitsNegative(range.drop);
    passThroughFilter.setFilterFieldName(range.axis);
    passThroughFilter.setFilterLimits(range.min, range.max);
    passThroughFilter.filter(*cloudFiltered);
  }
  for (const auto& range : cropBoxRanges)
  {
    if (range.enableWithBox && !boxAttached)
    {
      continue;
    }
    pcl::CropBox<pcl::PointXYZ> cropBoxFilter;
    cropBoxFilter.setInputCloud(cloudFiltered);
    cropBoxFilter.setNegative(range.drop);
    cropBoxFilter.setMin(Eigen::Vector4f(range.minX, range.minY, range.minZ, 1.0));
    cropBoxFilter.setMax(Eigen::Vector4f(range.maxX, range.maxY, range.maxZ, 1.0));
    cropBoxFilter.filter(*cloudFiltered);
  }

  // 将滤波后的数据发布到'/livox/lidar_filter'话题
  sensor_msgs::PointCloud2 filteredMsg;
  pcl::toROSMsg(*cloudFiltered, filteredMsg);
  filteredMsg.header = msg->header;
  livoxFilteredPub.publish(filteredMsg);
}

void carryBoxCallback(const cti_msgs::CarryBoxInfo::ConstPtr& msg)
{
  if (msg->carbox_comm == cti_msgs::CarryBoxInfo::CAR_BOX_DISCONNECT)
  {
    boxAttached = false;
  }
  else
  {
    boxAttached = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_filter_node");
  ros::NodeHandle nh;

  ros::Subscriber lidarSub = nh.subscribe("/livox/lidar", 10, pointcloudCallback);
  ros::Subscriber carryBoxSub = nh.subscribe("/robot_base/carry_box_info", 1, carryBoxCallback);
  livoxFilteredPub = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_filter", 10);
  readFilterRanges();

  ros::spin();

  return 0;
}
