// src/corridor_layer.cpp
#include "agv_corridor_layer/corridor_layer.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace agv_corridor_layer
{

void CorridorLayer::onInitialize()
{
  // nav2_costmap_2d::Layer 给了一个 weak_ptr 到 LifecycleNode
  // 确保引用对象不被销毁，这里的lock方法是提升弱指针到我们的强指针的
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("CorridorLayer: failed to lock parent node");
  }

  clock_ = node->get_clock();

  // 参数声明。name_ 是 Layer 基类自带的,对应 yaml 里这个插件实例的名字
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("self_id", rclcpp::ParameterValue(std::string("")));
  declareParameter("topic",
    rclcpp::ParameterValue(std::string("/reserved_corridors")));
  declareParameter("corridor_radius", rclcpp::ParameterValue(0.6));
  declareParameter("corridor_cost", rclcpp::ParameterValue(253));
  declareParameter("corridor_ttl", rclcpp::ParameterValue(30.0));

  bool enabled = true;
  node->get_parameter(name_ + ".enabled", enabled);
  enabled_ = enabled;
  node->get_parameter(name_ + ".self_id", self_id_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".corridor_radius", corridor_radius_);

  int cost_int = 253;
  node->get_parameter(name_ + ".corridor_cost", cost_int);
  corridor_cost_ =
    static_cast<unsigned char>(std::clamp(cost_int, 0, 254));

  node->get_parameter(name_ + ".corridor_ttl", corridor_ttl_);

  if (self_id_.empty()) {
    RCLCPP_WARN(
      node->get_logger(),
      "CorridorLayer[%s]: self_id is empty. This layer will treat ALL "
      "received corridors as foreign. Set it to the robot's namespace "
      "(e.g. 'agv_01') to avoid inflating its own path.",
      name_.c_str());
  }

  // 走廊话题是全局的 (调度器是单例,两台车都订阅同一话题)
  // QoS: 用 reliable + keep_last(10),调度器侧应该匹配
  corridor_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    topic_, rclcpp::QoS(10),
    std::bind(&CorridorLayer::corridorCallback, this, std::placeholders::_1));

  current_ = true;  // 告诉 layered_costmap 本层已可用

  RCLCPP_INFO(
    node->get_logger(),
    "CorridorLayer[%s] initialized. self_id='%s' topic='%s' "
    "radius=%.2fm cost=%u ttl=%.1fs",
    name_.c_str(), self_id_.c_str(), topic_.c_str(),
    corridor_radius_, corridor_cost_, corridor_ttl_);
}

void CorridorLayer::corridorCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  // 约定:调度器把 owner_id 塞在 header.frame_id 后面用 "|" 分隔,
  //   例如 "map|agv_01"。这样能复用 nav_msgs/Path 不用自定义 msg。
  // 实际 frame_id (给 TF 用的) 和 owner 会在此拆分。
  std::string frame_id = msg->header.frame_id;
  std::string owner;
  auto sep = frame_id.find('|');
  if (sep != std::string::npos) {
    owner = frame_id.substr(sep + 1);
    frame_id = frame_id.substr(0, sep);
  }

  if (owner.empty()) {
    // 调度器没带 owner,保守起见忽略 (否则本车会把自己的路径当障碍)
    return;
  }

  if (owner == self_id_) {
    return;  // 忽略自己的路径
  }

  ReservedCorridor corridor;
  corridor.owner_id = owner;
  corridor.poses = msg->poses;
  corridor.received_at = clock_->now();

  // 统一覆盖该 owner 上次的走廊 (每台车只保留最新一条)
  {
    std::lock_guard<std::mutex> lock(corridor_mutex_);
    corridors_[owner] = std::move(corridor);
  }
}

bool CorridorLayer::pruneExpired()
{
  if (corridor_ttl_ <= 0.0) {
    return false;
  }
  auto now = clock_->now();
  bool changed = false;
  std::lock_guard<std::mutex> lock(corridor_mutex_);
  for (auto it = corridors_.begin(); it != corridors_.end(); ) {
    const double age = (now - it->second.received_at).seconds();
    if (age > corridor_ttl_) {
      it = corridors_.erase(it);
      changed = true;
    } else {
      ++it;
    }
  }
  return changed;
}

void CorridorLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  pruneExpired();

  // 关键:把上一次刷过的区域也加进 bounds,这样走廊更新或删除时
  // layered_costmap 会调 updateCosts 覆盖掉旧格子 (擦除逻辑)
  if (has_last_bounds_) {
    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);
  }

  double new_min_x = std::numeric_limits<double>::max();
  double new_min_y = std::numeric_limits<double>::max();
  double new_max_x = std::numeric_limits<double>::lowest();
  double new_max_y = std::numeric_limits<double>::lowest();

  std::lock_guard<std::mutex> lock(corridor_mutex_);
  for (const auto & kv : corridors_) {
    for (const auto & pose : kv.second.poses) {
      const double x = pose.pose.position.x;
      const double y = pose.pose.position.y;
      new_min_x = std::min(new_min_x, x - corridor_radius_);
      new_min_y = std::min(new_min_y, y - corridor_radius_);
      new_max_x = std::max(new_max_x, x + corridor_radius_);
      new_max_y = std::max(new_max_y, y + corridor_radius_);
    }
  }

  if (new_min_x <= new_max_x) {
    *min_x = std::min(*min_x, new_min_x);
    *min_y = std::min(*min_y, new_min_y);
    *max_x = std::max(*max_x, new_max_x);
    *max_y = std::max(*max_y, new_max_y);

    last_min_x_ = new_min_x;
    last_min_y_ = new_min_y;
    last_max_x_ = new_max_x;
    last_max_y_ = new_max_y;
    has_last_bounds_ = true;
  } else {
    // 没有任何走廊了,下次不需要再 include 旧 bounds
    has_last_bounds_ = false;
  }
}

void CorridorLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  // 为了光栅化方便,用 master_grid 的 world->map 转换
  const double resolution = master_grid.getResolution();
  // 采样步长取分辨率的一半,避免线段太稀留缝
  const double step = resolution * 0.5;
  // 半径向 costmap 格子数
  const int radius_cells =
    static_cast<int>(std::ceil(corridor_radius_ / resolution));

  std::lock_guard<std::mutex> lock(corridor_mutex_);

  for (const auto & kv : corridors_) {
    const auto & poses = kv.second.poses;
    if (poses.size() < 2) {
      continue;
    }

    // 沿路径相邻两点之间插值采样
    for (size_t i = 1; i < poses.size(); ++i) {
      const double x0 = poses[i - 1].pose.position.x;
      const double y0 = poses[i - 1].pose.position.y;
      const double x1 = poses[i].pose.position.x;
      const double y1 = poses[i].pose.position.y;
      const double dx = x1 - x0;
      const double dy = y1 - y0;
      const double seg_len = std::hypot(dx, dy);
      const int samples =
        std::max(1, static_cast<int>(std::ceil(seg_len / step)));

      for (int s = 0; s <= samples; ++s) {
        const double t = static_cast<double>(s) / samples;
        const double cx = x0 + t * dx;
        const double cy = y0 + t * dy;

        unsigned int mx, my;
        if (!master_grid.worldToMap(cx, cy, mx, my)) {
          continue;  // 采样点在 costmap 外
        }

        // 以 (mx, my) 为圆心画实心圆
        const int mx_i = static_cast<int>(mx);
        const int my_i = static_cast<int>(my);
        for (int dj = -radius_cells; dj <= radius_cells; ++dj) {
          for (int di = -radius_cells; di <= radius_cells; ++di) {
            if (di * di + dj * dj > radius_cells * radius_cells) {
              continue;
            }
            const int ci = mx_i + di;
            const int cj = my_i + dj;
            // 必须落在 updateBounds 给出的窗口内,这是 Layer 契约
            if (ci < min_i || ci >= max_i || cj < min_j || cj >= max_j) {
              continue;
            }
            const unsigned char old_cost =
              master_grid.getCost(ci, cj);
            // 不覆盖已有的 LETHAL (静态障碍),只抬高 free / 低 cost 的格子
            if (old_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
              continue;
            }
            if (corridor_cost_ > old_cost) {
              master_grid.setCost(ci, cj, corridor_cost_);
            }
          }
        }
      }
    }
  }
}

void CorridorLayer::reset()
{
  std::lock_guard<std::mutex> lock(corridor_mutex_);
  corridors_.clear();
  has_last_bounds_ = false;
  current_ = true;
}

}  // namespace agv_corridor_layer

// pluginlib 注册
PLUGINLIB_EXPORT_CLASS(
  agv_corridor_layer::CorridorLayer,
  nav2_costmap_2d::Layer)
