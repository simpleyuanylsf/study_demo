#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>
#include <queue> 

#define MAP_X_SIZE 10.0 
#define MAP_Y_SIZE 10.0 
#define MAP_Z_SIZE 3.0  
#define RESOLUTION 0.1  

// 定义膨胀半径 (比如 20cm，即 2个格子)
// 根据你的无人机实际大小调整，mid360 很小，通常膨胀 2-3 格足够了
const int INFLATE_RADIUS = 2;

const int GLX_SIZE = std::ceil(MAP_X_SIZE / RESOLUTION);
const int GLY_SIZE = std::ceil(MAP_Y_SIZE / RESOLUTION);
const int GLZ_SIZE = std::ceil(MAP_Z_SIZE / RESOLUTION);

struct GridNode {
    Eigen::Vector3i index;
    double gScore;
    double fScore;
    GridNode* parent;
    
    GridNode(Eigen::Vector3i _idx) {
        index = _idx;
        gScore = INFINITY;
        fScore = INFINITY;
        parent = NULL;
    }
};

struct CompareNode {
    bool operator()(GridNode* a, GridNode* b) {
        return a->fScore > b->fScore; 
    }
};

class SimpleGridMap {
private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Subscriber goal_sub;
    ros::Publisher path_pub; // 新增：路径发布者
    ros::Publisher map_pub;      // 原来的 Marker 发布者 (可以留着对比，也可以删掉)
    ros::Publisher pcl_map_pub;  // 【新增】: 发布 PointCloud2 地图

    std::vector<int> grid_map_data; 
    Eigen::Vector3d start_pt, goal_pt;
    bool has_goal = false;

public:
    SimpleGridMap() {
        cloud_sub = nh.subscribe("/cloud_registered", 1, &SimpleGridMap::cloudCallback, this);
        goal_sub = nh.subscribe("/move_base_simple/goal", 1, &SimpleGridMap::goalCallback, this);
        
        //这些发布的后缀是在rviz中显示需要添加的模块名称
        map_pub = nh.advertise<visualization_msgs::Marker>("my_grid_map", 1);
        pcl_map_pub = nh.advertise<sensor_msgs::PointCloud2>("my_grid_map_pcl", 1);
        path_pub = nh.advertise<visualization_msgs::Marker>("my_astar_path", 1); // 新增astar显示

        grid_map_data.resize(GLX_SIZE * GLY_SIZE * GLZ_SIZE, 0);

        ROS_INFO("Map Initialized: Size [%d x %d x %d]", GLX_SIZE, GLY_SIZE, GLZ_SIZE);
    }

    bool isIndexValid(int x, int y, int z) {
        return (x >= 0 && x < GLX_SIZE && y >= 0 && y < GLY_SIZE && z >= 0 && z < GLZ_SIZE);
    }

    Eigen::Vector3i posToIndex(double x, double y, double z) {
        Eigen::Vector3i idx;
        idx(0) = std::floor((x + MAP_X_SIZE / 2.0) / RESOLUTION);
        idx(1) = std::floor((y + MAP_Y_SIZE / 2.0) / RESOLUTION);
        idx(2) = std::floor((z + MAP_Z_SIZE / 2.0) / RESOLUTION);
        return idx;
    }
    
    // 索引转坐标（画路径要用）
    Eigen::Vector3d indexToPos(Eigen::Vector3i idx) {
        Eigen::Vector3d pos;
        pos(0) = (idx(0) * RESOLUTION) - (MAP_X_SIZE / 2.0) + (RESOLUTION / 2.0);
        pos(1) = (idx(1) * RESOLUTION) - (MAP_Y_SIZE / 2.0) + (RESOLUTION / 2.0);
        pos(2) = (idx(2) * RESOLUTION) - (MAP_Z_SIZE / 2.0) + (RESOLUTION / 2.0);
        return pos;
    }

    double getHeuristic(Eigen::Vector3i node_idx, Eigen::Vector3i goal_idx) {
        return (node_idx - goal_idx).cast<double>().norm() * RESOLUTION;
    }

//***********************************************************************************************
// //以下为路径规划

    // B样条核心公式：给定 4 个控制点和 t，算出曲线上的点
    Eigen::Vector3d getPos(Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, double t) {
        // 均匀 B 样条基函数矩阵 (Matrix M)
        // 这一坨数字是数学家算好的，不用纠结，照抄即可
        // 它的作用是把 t 混合进 4 个控制点里
        double b0 = (1.0 - 3.0*t + 3.0*t*t - t*t*t) / 6.0;
        double b1 = (4.0 - 6.0*t*t + 3.0*t*t*t) / 6.0;
        double b2 = (1.0 + 3.0*t + 3.0*t*t - 3.0*t*t*t) / 6.0;
        double b3 = (t*t*t) / 6.0;

        return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
    }

    //b样条曲线生成函数
    std::vector<Eigen::Vector3d> generateBSpline(std::vector<Eigen::Vector3d> control_points) {
        std::vector<Eigen::Vector3d> smooth_path;

        if (control_points.empty()) return smooth_path;

        // 【修复点】：删掉了之前那个 if (size < 4) return 的判断
        // 现在哪怕只有 2 个点，我们也可以通过下面的补点逻辑生成曲线

        // 核心循环：每 4 个控制点，生成一段曲线
        // 比如 P0, P1, P2, P3 生成第一段
        // P1, P2, P3, P4 生成第二段...
        // 这就是为什么它叫 "Local Control" (局部控制)

        std::vector<Eigen::Vector3d> cpts = control_points;

        // 1. 首尾各重复 2 次
        // 这样：
        // 2个点 (A,B) -> 变成 6个点 (A,A,A, B,B,B) -> 可以生成
        // 3个点 (A,B,C) -> 变成 7个点 (A,A,A, B, C,C,C) -> 可以生成
        cpts.insert(cpts.begin(), control_points[0]); 
        cpts.insert(cpts.begin(), control_points[0]);
        cpts.push_back(control_points.back());
        cpts.push_back(control_points.back());

        // 2. 生成曲线
        double ts = 0.05; // 密度：越小越平滑
        for (int i = 0; i < cpts.size() - 3; i++) {
            for (double t = 0.0; t < 1.0; t += ts) {
                Eigen::Vector3d pos = getPos(cpts[i], cpts[i+1], cpts[i+2], cpts[i+3], t);
                smooth_path.push_back(pos);
            }
        }
        return smooth_path;
    }  
        
// 检查两个位置之间是否有障碍物
    bool isLineFree(Eigen::Vector3d p1, Eigen::Vector3d p2) {
        double dist = (p1 - p2).norm();
        if (dist < RESOLUTION) return true; // 太近了，认为无障碍

        // 步长：分辨率的一半，防止漏掉薄墙
        double step = RESOLUTION / 2.0; 
        int n_steps = std::floor(dist / step);

        Eigen::Vector3d direction = (p2 - p1).normalized();

        for (int i = 1; i < n_steps; i++) {
            Eigen::Vector3d check_pt = p1 + direction * (i * step);
            Eigen::Vector3i idx = posToIndex(check_pt(0), check_pt(1), check_pt(2));
            
            // 检查是否越界
            if (!isIndexValid(idx(0), idx(1), idx(2))) return false;

            // 检查是否撞墙
            int array_idx = idx(0) + idx(1) * GLX_SIZE + idx(2) * GLX_SIZE * GLY_SIZE;
            if (grid_map_data[array_idx] == 1) {
                return false; // 有障碍物！
            }
        }
        return true; // 一路畅通
    }

// 核心：路径剪枝
    std::vector<Eigen::Vector3d> simplifyPath(GridNode* end_node) {
        std::vector<Eigen::Vector3d> raw_path;
        
        // 1. 先从链表提取出原始路径 (从终点回溯到起点)
        GridNode* curr = end_node;
        while (curr != NULL) {
            raw_path.push_back(indexToPos(curr->index));
            curr = curr->parent;
        }
        // 现在的 raw_path 是：终点 -> ... -> 起点
        // 我们把它反转一下：起点 -> ... -> 终点
        std::reverse(raw_path.begin(), raw_path.end());

        if (raw_path.size() < 3) return raw_path; // 只有两个点就不需要剪枝了

        // 2. 开始剪枝 (Floyd Algorithm 简化版)
        std::vector<Eigen::Vector3d> optimized_path;
        optimized_path.push_back(raw_path[0]); // 起点肯定要保留

        int current_idx = 0;
        while (current_idx < raw_path.size() - 1) {
            // 贪心搜索：从当前点开始，尽可能往后找，直到被挡住
            for (int next_idx = raw_path.size() - 1; next_idx > current_idx; next_idx--) {
                // 如果当前点可以直接连到 next_idx，且中间无障碍
                if (isLineFree(raw_path[current_idx], raw_path[next_idx])) {
                    optimized_path.push_back(raw_path[next_idx]); // 保留这个远处的点
                    current_idx = next_idx; // 直接跳过去，中间的都删掉
                    break; 
                }
            }
        }
        return optimized_path;
    }

    // -----------------------------------------------------
    // 地图回调 这段代码其实是使用marker组件显示 由于根据高度生成不同颜色的区域需要手动赋值 所以此处使用pointcloud2组件
    // -----------------------------------------------------
    // void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    //     pcl::PointCloud<pcl::PointXYZ> cloud;
    //     pcl::fromROSMsg(*msg, cloud);
    //     if (cloud.points.empty()) return;

    //     start_pt << 0.0, 0.0, 1.0; 

    //     visualization_msgs::Marker cubes;
    //     cubes.header.frame_id = "camera_init";
    //     cubes.header.stamp = ros::Time::now();
    //     cubes.type = visualization_msgs::Marker::CUBE_LIST;
    //     cubes.action = visualization_msgs::Marker::ADD;
    //     cubes.scale.x = RESOLUTION; cubes.scale.y = RESOLUTION; cubes.scale.z = RESOLUTION;
        
    //     // 【修复 1：初始化四元数】---------------------------
    //     cubes.pose.orientation.w = 1.0;
    //     cubes.pose.orientation.x = 0.0;
    //     cubes.pose.orientation.y = 0.0;
    //     cubes.pose.orientation.z = 0.0;
    //     // --------------------------------------------------

    //     cubes.color.a = 0.6; cubes.color.r = 0.0; cubes.color.g = 1.0; cubes.color.b = 0.0;
        
    //     // 膨胀半径 (如果你不需要膨胀，就设为 0)
    //     const int INFLATE_RADIUS = 3; 

    //     for (const auto& pt : cloud.points) {
    //         Eigen::Vector3i idx = posToIndex(pt.x, pt.y, pt.z);

    //         // 膨胀循环
    //         for (int x = -INFLATE_RADIUS; x <= INFLATE_RADIUS; x++) {
    //             for (int y = -INFLATE_RADIUS; y <= INFLATE_RADIUS; y++) {
    //                 for (int z = -1; z <= 1; z++) { // z轴稍微膨胀一点即可
                        
    //                     int neighbor_x = idx(0) + x;
    //                     int neighbor_y = idx(1) + y;
    //                     int neighbor_z = idx(2) + z;

    //                     if (!isIndexValid(neighbor_x, neighbor_y, neighbor_z)) continue;

    //                     int array_idx = neighbor_x + neighbor_y * GLX_SIZE + neighbor_z * GLX_SIZE * GLY_SIZE;

    //                     // 标记障碍物
    //                     if(grid_map_data[array_idx] == 0) {
    //                         grid_map_data[array_idx] = 1; 
    //                     }

    //                     // 加入显示列表 (仅显示本次扫描到的中心点，防止膨胀后显示太乱)
    //                     if (x == 0 && y == 0 && z == 0) {
    //                         geometry_msgs::Point p;
    //                         // 计算坐标中心
    //                         p.x = neighbor_x * RESOLUTION - MAP_X_SIZE / 2.0 + RESOLUTION/2;
    //                         p.y = neighbor_y * RESOLUTION - MAP_Y_SIZE / 2.0 + RESOLUTION/2;
    //                         p.z = neighbor_z * RESOLUTION - MAP_Z_SIZE / 2.0 + RESOLUTION/2;
    //                         cubes.points.push_back(p);
    //                     }
    //                 }
    //             }
    //         }
    //     }
        
    //     // 【修复 2：判空】防止空列表报错
    //     if (!cubes.points.empty()) {
    //         map_pub.publish(cubes);
    //     }
    // }

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        if (cloud.points.empty()) return;

        start_pt << 0.0, 0.0, 1.0; 

        // -------------------------------------------------------
        // 【新增】: 准备一个用于显示的点云容器
        // -------------------------------------------------------
        pcl::PointCloud<pcl::PointXYZ> display_cloud; 
        
        // 膨胀半径 (之前定义的)
        const int INFLATE_RADIUS = 2; 

        for (const auto& pt : cloud.points) {
            Eigen::Vector3i idx = posToIndex(pt.x, pt.y, pt.z);

            // 膨胀循环
            for (int x = -INFLATE_RADIUS; x <= INFLATE_RADIUS; x++) {
                for (int y = -INFLATE_RADIUS; y <= INFLATE_RADIUS; y++) {
                    for (int z = -1; z <= 1; z++) { 
                        
                        int neighbor_x = idx(0) + x;
                        int neighbor_y = idx(1) + y;
                        int neighbor_z = idx(2) + z;

                        if (!isIndexValid(neighbor_x, neighbor_y, neighbor_z)) continue;

                        int array_idx = neighbor_x + neighbor_y * GLX_SIZE + neighbor_z * GLX_SIZE * GLY_SIZE;

                        // 1. 更新内部地图数据 (逻辑不变)
                        if(grid_map_data[array_idx] == 0) {
                            grid_map_data[array_idx] = 1; 
                        }

                        // 2. 【核心修改】: 加入到显示点云中
                        // 我们只显示中心点 (x=0,y=0,z=0) 或者你可以显示全部
                        if (x == 0 && y == 0 && z == 0) {
                            pcl::PointXYZ p;
                            // 计算物理坐标
                            p.x = neighbor_x * RESOLUTION - MAP_X_SIZE / 2.0 + RESOLUTION/2;
                            p.y = neighbor_y * RESOLUTION - MAP_Y_SIZE / 2.0 + RESOLUTION/2;
                            p.z = neighbor_z * RESOLUTION - MAP_Z_SIZE / 2.0 + RESOLUTION/2;
                            
                            display_cloud.points.push_back(p);
                        }
                    }
                }
            }
        }
        
        // -------------------------------------------------------
        // 【新增】: 转换为 ROS 消息并发布
        // -------------------------------------------------------
        if (!display_cloud.points.empty()) {
            sensor_msgs::PointCloud2 output_msg;
            pcl::toROSMsg(display_cloud, output_msg);
            
            output_msg.header.frame_id = "camera_init"; // 必须和 Fast-LIO 一致
            output_msg.header.stamp = ros::Time::now();
            
            pcl_map_pub.publish(output_msg);
        }
    }

    //目标点回调函数处理
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        goal_pt << x, y, 1.0; 
        has_goal = true;
        ROS_INFO("Get Goal: [%f, %f, 1.0], Start A* Search...", x, y);
        AstarSearch();
    }
    
    // 修改后的可视化函数：支持自定义 ID 和 颜色
    void visualizePath(const std::vector<Eigen::Vector3d>& path, int id, double r, double g, double b) {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "camera_init";
        line_strip.header.stamp = ros::Time::now();
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.scale.x = 0.05; // 线宽
        
        // 关键：不同的路径需要不同的 ID，否则会被覆盖！
        line_strip.id = id; 
        
        // 设置颜色
        line_strip.color.a = 1.0;
        line_strip.color.r = r;
        line_strip.color.g = g;
        line_strip.color.b = b;
        
        // 设置姿态（防报错）
        line_strip.pose.orientation.w = 1.0;

        for (const auto& pt : path) {
            geometry_msgs::Point p;
            p.x = pt(0); p.y = pt(1); p.z = pt(2);
            line_strip.points.push_back(p);
        }
        
        if (!line_strip.points.empty()) {
            path_pub.publish(line_strip);
        }
    }

    //a*算法实现
    void AstarSearch() {
        if (!has_goal) return;
        Eigen::Vector3i start_idx = posToIndex(start_pt(0), start_pt(1), start_pt(2));
        Eigen::Vector3i goal_idx = posToIndex(goal_pt(0), goal_pt(1), goal_pt(2));

        if (!isIndexValid(start_idx(0), start_idx(1), start_idx(2)) || 
            !isIndexValid(goal_idx(0), goal_idx(1), goal_idx(2))) {
            ROS_WARN("Start or Goal is outside of the map!");
            return;
        }

// 【解决卡顿 1】: 如果终点本身就是障碍物，直接放弃
        int goal_flat_idx = goal_idx(0) + goal_idx(1) * GLX_SIZE + goal_idx(2) * GLX_SIZE * GLY_SIZE;
        if (grid_map_data[goal_flat_idx] == 1) {
            ROS_WARN("Goal is inside Obstacle! Quit Search.");
            return;
        }

        std::priority_queue<GridNode*, std::vector<GridNode*>, CompareNode> open_list;
        std::vector<bool> visited(GLX_SIZE * GLY_SIZE * GLZ_SIZE, false);
        std::vector<GridNode*> node_pool(GLX_SIZE * GLY_SIZE * GLZ_SIZE, NULL);

        GridNode* start_node = new GridNode(start_idx);
        start_node->gScore = 0;
        start_node->fScore = getHeuristic(start_idx, goal_idx);
        open_list.push(start_node);
        node_pool[start_idx(0) + start_idx(1)*GLX_SIZE + start_idx(2)*GLX_SIZE*GLY_SIZE] = start_node;

        // --- A* 核心循环 ---
        GridNode* current_node = NULL;
        GridNode* neighbor_node = NULL;

        while (!open_list.empty()) {
            current_node = open_list.top();
            open_list.pop();

            // 判断是否到达终点
            if ((current_node->index - goal_idx).cast<double>().norm() < 2.0) { 
                ROS_INFO("Goal Reached! Optimizing Path...");
                
                // 1. 剪枝 (得到折线)
                std::vector<Eigen::Vector3d> simple_path = simplifyPath(current_node);
                
                // 2. B样条生成 (得到曲线)
                std::vector<Eigen::Vector3d> smooth_path = generateBSpline(simple_path);

                // --- 3. 显示两条路进行对比 ---
                
                // 显示剪枝后的折线 (ID=0, 蓝色)
                visualizePath(simple_path, 0, 0.0, 0.0, 1.0); 

                // 显示 B样条曲线 (ID=1, 红色)
                visualizePath(smooth_path, 1, 1.0, 0.0, 0.0); 
                
                return;
            }

            int dx[6] = {1, -1, 0, 0, 0, 0};
            int dy[6] = {0, 0, 1, -1, 0, 0};
            int dz[6] = {0, 0, 0, 0, 1, -1};

            for (int i = 0; i < 6; i++) {
                Eigen::Vector3i neighbor_idx;
                neighbor_idx(0) = current_node->index(0) + dx[i];
                neighbor_idx(1) = current_node->index(1) + dy[i];
                neighbor_idx(2) = current_node->index(2) + dz[i];

                if (!isIndexValid(neighbor_idx(0), neighbor_idx(1), neighbor_idx(2))) continue;

                int neighbor_flat_idx = neighbor_idx(0) + neighbor_idx(1) * GLX_SIZE + neighbor_idx(2) * GLX_SIZE * GLY_SIZE;

                if (grid_map_data[neighbor_flat_idx] == 1) continue; // 撞墙了
                if (visited[neighbor_flat_idx]) continue; // 走过了

                if (node_pool[neighbor_flat_idx] == NULL) {
                    node_pool[neighbor_flat_idx] = new GridNode(neighbor_idx);
                }
                neighbor_node = node_pool[neighbor_flat_idx];

                double provisional_gScore = current_node->gScore + RESOLUTION;

                if (provisional_gScore < neighbor_node->gScore) {
                    neighbor_node->parent = current_node; // 记录路径！
                    neighbor_node->gScore = provisional_gScore;
                    neighbor_node->fScore = provisional_gScore + getHeuristic(neighbor_idx, goal_idx);
                    
                    open_list.push(neighbor_node);
                    visited[neighbor_flat_idx] = true;
                }
            }
        }
        
        ROS_WARN("Failed to find path!");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "nav_map_node");
    SimpleGridMap map;
    ros::spin();
    return 0;
}