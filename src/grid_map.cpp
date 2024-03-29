#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

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
    grid_depth_ =
        (thre_z_max - thre_z_min) / resolution_;  // 栅格地图深度，单位：栅格
    // 初始化栅格地图
    grid_map_.resize(grid_width_ * grid_height_, vector<int>(grid_depth_, -1));
    // 将栅格地图中所有栅格状态设置为灰色
  }
  ~GridMap() {}
  void pointcloudCallback(const PointCloud::ConstPtr& msg) {
    // 获取点云
    *cloud += *msg;
    // 增加计数器
    pointCloudCount++;
    if (pointCloudCount == filter_num) {
      // 高度滤波
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(thre_z_min, thre_z_max);
      pass.filter(*cloud);
      // 获取tf变换
      tf::StampedTransform transform;
      try {
        listener_.lookupTransform("camera_init", "body", ros::Time(0),
                                  transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("lookupTransform meet error:%s", ex.what());
        return;
      }
      double center_x_ =
          (transform.getOrigin().getX() + map_width_ / 2) / resolution_;
      double center_y_ =
          (transform.getOrigin().getY() + map_height_ / 2) / resolution_;
      double center_z_ =
          transform.getOrigin().getZ() / resolution_;  // 增加z轴的处理
      for (const auto& point : cloud->points) {
        // 计算点所在的栅格坐标
        int grid_x = (point.x + map_width_ / 2) / resolution_;
        int grid_y = (point.y + map_height_ / 2) / resolution_;
        int grid_z = point.z / resolution_;  // 增加z轴的处理

        // 判断栅格坐标是否在地图范围内
        if (grid_x >= 0 && grid_x < grid_width_ && grid_y >= 0 &&
            grid_y < grid_height_) {  // 增加z轴的处理
          bresenham3D(center_x_, center_y_, center_z_, grid_x, grid_y, grid_z);
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
        for (auto grid : grid_map_[i]) {
          bool first=true;
          if (grid > -1) {          
            if (grid < 4&&first) {
              occupancy_grid.data[i] = 0;  // 未占用栅格
              first=false;
            } 
            else if (grid < 11) {
              occupancy_grid.data[i] = 100;  // 占用栅格
              break;
            } 
            else {
              cout << "error,gridmap:" << grid << endl;
            }
          }
        }       
      }
      // 发布OccupancyGrid消息
      occupancy_grid_pub_.publish(occupancy_grid);
      cloud->clear();
      pointCloudCount = 0;
    }
  }
  void bresenham3D(int x1, int y1, int z1, int x2, int y2, int z2) {
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int dz = abs(z2 - z1);

    int xs = (x2 - x1) > 0 ? 1 : -1;
    int ys = (y2 - y1) > 0 ? 1 : -1;
    int zs = (z2 - z1) > 0 ? 1 : -1;

    if (dx >= dy && dx >= dz) {
      int p1 = 2 * dy - dx;
      int p2 = 2 * dz - dx;
      while (x1 != x2) {
        declineProbability(x1, y1, z1);
        x1 += xs;
        if (p1 >= 0) {
          y1 += ys;
          p1 -= 2 * dx;
        }
        if (p2 >= 0) {
          z1 += zs;
          p2 -= 2 * dx;
        }
        p1 += 2 * dy;
        p2 += 2 * dz;      
      }
    } else if (dy >= dx && dy >= dz) {
      int p1 = 2 * dx - dy;
      int p2 = 2 * dz - dy;
      while (y1 != y2) {
        declineProbability(x1, y1, z1);
        y1 += ys;
        if (p1 >= 0) {
          x1 += xs;
          p1 -= 2 * dy;
        }
        if (p2 >= 0) {
          z1 += zs;
          p2 -= 2 * dy;
        }
        p1 += 2 * dx;
        p2 += 2 * dz;
      }
    } else {
      int p1 = 2 * dy - dz;
      int p2 = 2 * dx - dz;
      while (z1 != z2) {
        declineProbability(x1, y1, z1);
        z1 += zs;
        if (p1 >= 0) {
          y1 += ys;
          p1 -= 2 * dz;
        }
        if (p2 >= 0) {
          x1 += xs;
          p2 -= 2 * dz;
        }
        p1 += 2 * dy;
        p2 += 2 * dx;
      }
    }
    increaseProbability(x2, y2, z2);
  }
  void declineProbability(int x, int y, int z) {
    if (x >= 0 && x < grid_width_ && y >= 0 && y < grid_height_ &&
        z >= thre_z_min / resolution_ && z < thre_z_max / resolution_) {
      int index = y * grid_width_ + x;
      z -= thre_z_min / resolution_;
      grid_map_[index][z] -= 1;
      if (grid_map_[index][z] < 0) {
        grid_map_[index][z] = 0;
      }
    }
  }
  void increaseProbability(int x, int y, int z) {
    int index = y * grid_width_ + x;
    z -= thre_z_min / resolution_;
    grid_map_[index][z] += 1;
    if (grid_map_[index][z] < 0) {
      grid_map_[index][z] = 0;
    } else if (grid_map_[index][z] > 9) {
      grid_map_[index][z] = 10;
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
  int grid_depth_;
  vector<vector<int> > grid_map_;
  ros::Publisher occupancy_grid_pub_;
  int pointCloudCount = 0;
  PointCloud::Ptr cloud;  // 用于合并两次点云的变量
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "grid_map");
  GridMap grid_map;
  ros::spin();
  return 0;
}
