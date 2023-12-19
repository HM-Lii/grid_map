#include <nav_msgs/OccupancyGrid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;
struct Point {
  int x;
  int y;
};
class GridMap {
 public:
  GridMap() : nh_("~") {
    cloud.reset(new PointCloud);
    // 订阅点云主题

    nh_.param("/grid_map/cloud_topic", cloud_topic, string("cloud_registered"));
    nh_.param("/grid_map/map_topic_name", map_topic_name, string("map"));
    cout << "subscribe:" << cloud_topic << endl;
    cout << "subscribe:" << cloud_topic.c_str() << endl;
    // 设置栅格地图参数
    nh_.param("/grid_map/map_width", map_width_, 10.0);
    nh_.param("/grid_map/map_height", map_height_, 10.0);
    nh_.param("/grid_map/thre_z_min", thre_z_min, -0.2);
    nh_.param("/grid_map/thre_z_max", thre_z_max, 1.0);
    nh_.param("/grid_map/filter_radius", filter_radius, 0.5);
    nh_.param("/grid_map/filter_num", filter_num, 3);
    nh_.param("/grid_map/map_resolution", resolution_, 0.05);
    nh_.param("/grid_map/orgin_x", orgin_x, 0.5);
    nh_.param("/grid_map/orgin_y", orgin_y, 0.5);

    pointcloud_sub_ = nh_.subscribe(cloud_topic.c_str(), 2,
                                    &GridMap::pointcloudCallback, this);
    occupancy_grid_pub_ =
        nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_name.c_str(), 1, true);

    grid_width_ = map_width_ / resolution_;  // 栅格地图宽度，单位：栅格
    grid_height_ = map_height_ / resolution_;  // 栅格地图高度，单位：栅格

    // 初始化栅格地图
    grid_map_ = new int16_t[grid_width_ * grid_height_];
    for (int i = 0; i < grid_width_ * grid_height_; ++i) {
      grid_map_[i] = -1;
    }
    // 将栅格地图中所有栅格状态设置为灰色
    // 设置激光雷达参数
    scan_angle_min_ = -M_PI / 6;  // 扫描角度最小值，单位：度
    scan_angle_max_ = M_PI / 6;   // 扫描角度最大值，单位：度
    scan_range_ = 40;             // 扫描距离，单位：米
  }
  ~GridMap() {
    // 清理动态分配的数组
    delete[] grid_map_;
  }
  void pointcloudCallback(const PointCloud::ConstPtr& msg) {
    // 获取点云
    *cloud += *msg;
    pointCloudCount++;
    if (pointCloudCount == 5) {
      // 高度滤波
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(thre_z_min, thre_z_max);
      pass.filter(*cloud);

      // 半径滤波
      //创建滤波器
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
      //设置输入点云
      radiusoutlier.setInputCloud(cloud);
      //设置半径,在该范围内找临近点
      radiusoutlier.setRadiusSearch(filter_radius);
      //设置查询点的邻域点集数，小于该阈值的删除
      radiusoutlier.setMinNeighborsInRadius(filter_num);
      radiusoutlier.filter(*cloud);
      // 获取tf变换
      tf::StampedTransform transform;
      try {
        listener_.lookupTransform("camera_init", "body", ros::Time(0),
                                  transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return;
      }
      double center_x_ =
          (transform.getOrigin().getX() + map_width_ / 2) / resolution_;
      double center_y_ =
          (transform.getOrigin().getY() + map_height_ / 2) / resolution_;

      // 将点从车体坐标系转换为地图坐标系
      pcl_ros::transformPointCloud("camera_init", *cloud, *cloud, listener_);

      // 遍历点云中的每个点
      for (const auto& point : cloud->points) {
        // 计算点所在的栅格坐标
        int grid_x = (point.x + map_width_ / 2) / resolution_;
        int grid_y = (point.y + map_height_ / 2) / resolution_;

        // 判断栅格坐标是否在地图范围内
        if (grid_x >= 0 && grid_x < grid_width_ && grid_y >= 0 &&
            grid_y < grid_height_) {
          int start_x = center_x_;
          int start_y = center_y_;

          int end_x = grid_x;
          int end_y = grid_y;
          // 使用 Bresenham算法计算从该点到射线终点之间的所有栅格
          int dx = abs(end_x - start_x);
          int dy = abs(end_y - start_y);
          int sx = (start_x < end_x) ? 1 : -1;
          int sy = (start_y < end_y) ? 1 : -1;
          int err = dx - dy;

          while (true) {
            if (start_x == end_x && start_y == end_y) {
              break;
            }
            //防止把终点的栅格设置为空闲
            // 检查当前栅格是否在地图范围内
            if (start_x >= 0 && start_x < grid_width_ && start_y >= 0 &&
                start_y < grid_height_) {
              int index = start_y * grid_width_ + start_x;
              grid_map_[index] -= 1;
              // 减小占据可能性
              if (grid_map_[index] < 0) {
                grid_map_[index] = 0;
              }
            }
            int e2 = 2 * err;
            if (e2 > -dy) {
              err -= dy;
              start_x += sx;
            }
            if (e2 < dx) {
              err += dx;
              start_y += sy;
            }
          }
          //增加占据可能性
          int index = grid_y * grid_width_ + grid_x;
          grid_map_[index] += 1;
          if (grid_map_[index] < 0) {
            grid_map_[index] = 0;
          } else if (grid_map_[index] > 9) {
            grid_map_[index] = 10;
          }
        }
      }
      nav_msgs::OccupancyGrid occupancy_grid;
      occupancy_grid.header.frame_id = "camera_init";
      occupancy_grid.info.width = grid_width_;
      occupancy_grid.info.height = grid_height_;
      occupancy_grid.info.resolution = resolution_;
      occupancy_grid.info.origin.position.x = -map_width_ * orgin_x;
      occupancy_grid.info.origin.position.y = -map_height_ * orgin_y;

      // 将grid_map_中的栅格状态转换为OccupancyGrid数据
      occupancy_grid.data.resize(grid_width_ * grid_height_, -1);
      for (int i = 0; i < grid_width_ * grid_height_; ++i) {
        if (grid_map_[i] > -1) {
          if (grid_map_[i] < 4) {
            occupancy_grid.data[i] = 0;  // 无障碍栅格
          } else if (grid_map_[i] < 11) {
            occupancy_grid.data[i] = 100;  // 占用栅格
          } else {
            cout << "error,gridmap:" << grid_map_[i] << endl;
          }
        }
      }
      // 发布OccupancyGrid消息
      occupancy_grid_pub_.publish(occupancy_grid);
      cloud->clear();
      pointCloudCount = 0;
    }
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_sub_;
  tf::TransformListener listener_;

  string cloud_topic;
  string map_topic_name;

  double thre_z_min;
  double thre_z_max;
  double filter_radius;
  int filter_num;
  double map_width_;
  double map_height_;
  double resolution_;
  double orgin_x;
  double orgin_y;
  int grid_width_;
  int grid_height_;
  int16_t* grid_map_;
  ros::Publisher occupancy_grid_pub_;
  int pointCloudCount = 0;
  PointCloud::Ptr cloud;  // 用于合并两次点云的变量
  double scan_angle_min_;
  double scan_angle_max_;
  double scan_range_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "grid_map");
  GridMap grid_map;
  ros::spin();
  return 0;
}
