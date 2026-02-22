#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 定义一个简单的栅格地图类
class SimpleGridMap {
private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher map_pub; // 发送给 RViz 检查

    // 地图参数
    double resolution = 0.1; // 分辨率：10cm 一个格子
    
    // 这里我们用一个极其简化的方式存储地图：std::set
    // 实际项目中会用 std::vector<int> (一维数组模拟三维) 来提高速度
    // 但为了让你今天就能看懂，我们先用 Set 来存“占用的格子坐标”
    // string 格式: "x_index,y_index,z_index"

public:
    SimpleGridMap() {
        // 1. 订阅 Fast-LIO 的点云 (请确认你的话题名是否是 /cloud_registered)
        cloud_sub = nh.subscribe("/cloud_registered", 1, &SimpleGridMap::cloudCallback, this);
        
        // 2. 发布 Marker 给 RViz 显示我们的格子
        map_pub = nh.advertise<visualization_msgs::Marker>("my_grid_map", 1);
        
        ROS_INFO("GridMap Node Started! Waiting for PointCloud...");
    }

    // 核心回调函数：只要有新的点云进来，就会执行这里
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // --- 第一步：格式转换 (ROS -> PCL) ---
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        if (cloud.points.empty()) return;

        // --- 准备工作 ---
        // 使用 set 来去重，防止同一个格子里生成几百个方块卡死 RViz
        std::set<std::string> current_frame_occupied; 
        
        visualization_msgs::Marker cubes;
        cubes.header.frame_id = "camera_init"; // 必须和 Fast-LIO 的坐标系一致
        cubes.header.stamp = ros::Time::now();
        cubes.id = 0;
        cubes.type = visualization_msgs::Marker::CUBE_LIST;
        cubes.action = visualization_msgs::Marker::ADD;
        cubes.scale.x = resolution; 
        cubes.scale.y = resolution;
        cubes.scale.z = resolution;
        cubes.color.a = 0.8; //稍微透明一点，更有科技感
        cubes.color.r = 1.0; cubes.color.g = 0.0; cubes.color.b = 0.0; 

        // --- 第二步：遍历每个点 ---
        for (const auto& pt : cloud.points) {//范围 for 循环，专门用来遍历容器。 类型为auto
            // 核心公式：坐标 -> 索引
            int idx_x = std::floor(pt.x / resolution);
            int idx_y = std::floor(pt.y / resolution);
            int idx_z = std::floor(pt.z / resolution);

            // 生成唯一 Key 这里根据xyz坐标生成唯一key 如果点云很近那么会用到下方去重
            std::string key = std::to_string(idx_x) + "_" + 
                              std::to_string(idx_y) + "_" + 
                              std::to_string(idx_z);

            // ⚡️ 核心去重逻辑：如果这个格子已经加过方块了，就跳过 // 检查集合里有没有？目前是空的，没有。
            if (current_frame_occupied.count(key)) {
                continue;
            }

            // 没加过，标记为已占用
            current_frame_occupied.insert(key);// 把 "元素" 放进集合。
            
            // 计算中心坐标并加入列表
            geometry_msgs::Point center;
            center.x = idx_x * resolution + resolution / 2.0;
            center.y = idx_y * resolution + resolution / 2.0;
            center.z = idx_z * resolution + resolution / 2.0;
            
            cubes.points.push_back(center);
        }

        // --- 第三步：发布出去 ---
        // 只有当有方块时才发布
        if (!cubes.points.empty()) {
            map_pub.publish(cubes);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_grid_map_node");
    SimpleGridMap map;
    ros::spin();
    return 0;
}