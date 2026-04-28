// include/agv_corridor_layer/corridor_layer.hpp
#ifndef AGV_CORRIDOR_LAYER__CORRIDOR_LAYER_HPP_
#define AGV_CORRIDOR_LAYER__CORRIDOR_LAYER_HPP_

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agv_corridor_layer
{

/**
 * @brief 一条被预约的走廊,由一条中心路径 + 时间戳组成
 */
struct ReservedCorridor
{
  std::string owner_id;              // 占用这条走廊的车辆 ID,例如 "agv_01"
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  // 为啥我们这里不可以使用时间戳呀,这个是在节点端的时间吗？
  rclcpp::Time received_at;          // 收到时的本地时间,用于过期判断
};

/**
 * @brief 把其他车辆的预约路径膨胀成占用区,注入 global_costmap。
 *
 * 参数:
 *   - self_id:            本车 ID,收到的走廊如果 owner_id 等于 self_id 会被忽略
 *   - topic:              订阅的走廊话题 (默认 /reserved_corridors)
 *   - corridor_radius:    路径膨胀半径 (米)
 *   - corridor_cost:      走廊内格子写入的 cost (0-254),推荐 253 (LETHAL-1)
 *                         留出 LETHAL 给静态障碍,避免和 inflation_layer 语义混淆
 *   - corridor_ttl:       走廊有效期秒数,超时自动清除,避免调度器挂后卡死
 *   - enabled:            运行时开关
 */
class CorridorLayer : public nav2_costmap_2d::Layer
{
public:
  CorridorLayer() = default;
  ~CorridorLayer() override = default;

  // ---- nav2_costmap_2d::Layer 接口 ----
  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;

  bool isClearable() override { return true; }

  void onFootprintChanged() override {}

private:
  // 订阅回调:单条走廊消息
  void corridorCallback(const nav_msgs::msg::Path::SharedPtr msg);

  // 根据 TTL 清理过期走廊,返回是否有变动
  bool pruneExpired();

  // 把一条走廊光栅化到 dirty cells,同时扩展 bounds
  void rasterizeCorridor(
    const ReservedCorridor & corridor,
    double * min_x, double * min_y,
    double * max_x, double * max_y);

  // ---- 参数 ----
  std::string self_id_;
  std::string topic_;
  double corridor_radius_{0.6};
  unsigned char corridor_cost_{253};
  double corridor_ttl_{30.0};

  // ---- 状态 ----
  // key 是 owner_id,每台车只保留最新一条走廊
  std::unordered_map<std::string, ReservedCorridor> corridors_;
  std::mutex corridor_mutex_;

  // 上一次 updateBounds 时刷过的格子包围盒,用于下次"擦除"
  double last_min_x_{0.0}, last_min_y_{0.0};
  double last_max_x_{0.0}, last_max_y_{0.0};
  bool has_last_bounds_{false};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr corridor_sub_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace agv_corridor_layer

#endif  // AGV_CORRIDOR_LAYER__CORRIDOR_LAYER_HPP_
