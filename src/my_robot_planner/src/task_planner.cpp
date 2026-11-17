#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <my_robot_interface/action/move_to_named.hpp>
#include <my_robot_interface/action/pick_place.hpp>
#include <my_robot_interface/action/sort_objects.hpp>
#include <my_robot_interface/msg/detected_object_array.hpp>
#include <my_robot_interface/msg/grasp_candidate.hpp>
#include <my_robot_perception/object_catalog.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class TaskPlanner : public rclcpp::Node
{
  using Sort = my_robot_interface::action::SortObjects;
  using Pick = my_robot_interface::action::PickPlace;
  using Named = my_robot_interface::action::MoveToNamed;
  using Object = my_robot_interface::msg::DetectedObject;
  using Objects = my_robot_interface::msg::DetectedObjectArray;
  using Candidate = my_robot_interface::msg::GraspCandidate;
  using Trigger = std_srvs::srv::Trigger;
  using SortHandle = rclcpp_action::ServerGoalHandle<Sort>;
  using PickHandle = rclcpp_action::ClientGoalHandle<Pick>;
  using NamedHandle = rclcpp_action::ClientGoalHandle<Named>;
  using Clock = std::chrono::steady_clock;
  using Class = uint16_t;
  using Slot = std::pair<Class, size_t>;
  enum class Phase {WAITING, OBSERVING, SENDING, EXECUTING, RECOVERING};
  enum class RuleState {READY, RESERVED, PLACED};
  enum class Shortage {NONE, OCCUPIED, REJECTED};
  // 区域规则编译成绝对候选列表；点规则（绝对或相对）按物体冻结目标并保留尺寸预留。
  enum class RuleKind {SLOT, POINT};
  struct RuleTarget {Class class_id; tf2::Vector3 goal, extent; RuleState state = RuleState::READY;};
  struct Attempt {int count = 0; rclcpp::Time retry_after{0, 0, RCL_ROS_TIME};};
  // 候选因预检失败被拒；按候选记录次数与最近原因，条件变化后可有限次重评。
  struct CandidateFailure {int attempts = 0; uint8_t reason = 0; int reevaluations = 0;};
  struct Verification {Object source; tf2::Vector3 target; Slot slot;};
  struct PlacementCheck {const Object * destination = nullptr; bool at_source = false, reliable = false;};
  struct Reservation {tf2::Vector3 goal, extent;};
  // 每个物体只保留最新一条候选：旧候选被覆盖丢弃，不做历史回查。
  // center 是候选到达时该物体的观测中心，用于判断物体是否已被移动。
  struct CachedCandidate {Candidate msg; rclcpp::Time received; tf2::Vector3 center; bool has_center = false;};

public:
  TaskPlanner() : Node("task_planner"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    frame_ = declare_parameter<std::string>("reference_frame", "base_link");
    tool_frame_ = declare_parameter<std::string>("end_effector_frame", "link6");
    tool_to_tcp_ = xyz(declare_parameter<std::vector<double>>("tool_to_tcp", {0, 0, 0.132}));
    for (const auto & definition : my_robot_perception::loadCatalog(*this)) {
      classes_.emplace(definition.id, definition);
      default_order_.push_back(definition.id);
    }
    default_strategy_ = declare_parameter<std::string>("default_strategy", "class_order");
    const auto requested_order = declare_parameter<std::vector<int64_t>>("default_class_order", std::vector<int64_t>{});
    if (!requested_order.empty()) {
      default_order_.clear();
      for (const auto id : requested_order) {
        if (id > 0 && id <= 255 && classes_.count(static_cast<Class>(id))) {
          default_order_.push_back(static_cast<Class>(id));
        } else {RCLCPP_WARN(get_logger(), "Ignoring undefined priority class %ld", id);}
      }
    }
    finish_mode_ = declare_parameter<std::string>("finish_mode", "batch");
    retry_policy_ = declare_parameter<std::string>("retry_policy", "priority");
    exhausted_policy_ = declare_parameter<std::string>("on_attempts_exhausted", "continue");
    slots_full_policy_ = declare_parameter<std::string>("on_slots_full", "wait");
    verification_ = declare_parameter<std::string>("placement_verification", "visual");
    max_attempts_ = declare_parameter<int>("max_attempts", 3);
    max_candidate_attempts_ = declare_parameter<int>("max_candidate_attempts", 2);
    max_candidate_reevaluations_ = declare_parameter<int>("max_candidate_reevaluations", 1);
    max_objects_ = declare_parameter<int>("max_objects", 0);
    idle_timeout_ = declare_parameter<double>("idle_timeout", 3.0);
    cooldown_ = declare_parameter<double>("retry_cooldown", 0.0);
    verify_timeout_ = declare_parameter<double>("verification_timeout", 3.0);
    slot_tolerance_ = declare_parameter<double>("slot_tolerance", 0.012);
    height_tolerance_ = declare_parameter<double>("height_tolerance", 0.006);
    slot_clearance_ = declare_parameter<double>("slot_clearance", 0.025);
    max_age_ = declare_parameter<double>("max_observation_age", 2.0);
    // 候选新鲜度上限必须容得下推理节拍（节流 5 s + 进程往返约 4.4 s，观测时戳天然滞后
    // 4.4–9.4 s），否则候选会周期性被当作过期。不另做健康检查：
    // 候选过期或缺席即等于传统链路，降级由数据的缺席产生。
    candidate_max_age_ = declare_parameter<double>("grasp_candidate_max_age", 20.0);
    candidate_max_drift_ = declare_parameter<double>("grasp_candidate_max_drift", 0.02);
    timeout_ = declare_parameter<double>("observation_timeout", 5.0);
    action_timeout_ = declare_parameter<double>("action_timeout", 180.0);
    recovery_timeout_ = declare_parameter<double>("recovery_timeout", 90.0);
    observation_target_ = declare_parameter<std::string>("observation_target", "pose_1");
    settle_time_ = declare_parameter<double>("observation_settle_time", 0.3);
    workspace_min_ = declare_parameter<std::vector<double>>("workspace_min", std::vector<double>{});
    workspace_max_ = declare_parameter<std::vector<double>>("workspace_max", std::vector<double>{});
    if (!workspace_min_.empty() || !workspace_max_.empty()) {
      const auto lo = xyz(workspace_min_), hi = xyz(workspace_max_);
      for (int i = 0; i < 3; ++i) {
        if (lo[i] >= hi[i]) {throw std::invalid_argument("workspace_min must be below max");}
      }
    }
    position_modes_ = declare_parameter<std::vector<std::string>>(
      "place_position_modes", {"absolute", "absolute", "absolute"});
    if (position_modes_.size() != 3 || !std::all_of(position_modes_.begin(), position_modes_.end(),
      [](const auto & mode) {return mode == "absolute" || mode == "relative";}))
    {throw std::invalid_argument("place_position_modes requires X/Y/Z relative or absolute");}
    explicit_rules_ = declare_parameter<bool>("use_position_rules", false);
    // Discover explicit rules even when every referenced ID is unknown. Never
    // fall back to region coordinates after an explicit rule set was supplied.
    for (const auto & entry : get_node_parameters_interface()->get_parameter_overrides()) {
      const auto & name = entry.first;
      const bool point = name.rfind("place_positions.", 0) == 0;
      const bool region = name.rfind("place_regions.", 0) == 0;
      if (point || region) {explicit_rules_ = true;}
      if (point || region || name.rfind("classes.", 0) == 0) {
        try {
          const auto start = name.find('.') + 1;
          const auto id = name.substr(start, name.find('.', start) - start);
          const auto value = std::stoul(id);
          if (std::to_string(value) != id || value > 255 || !classes_.count(static_cast<Class>(value))) {
            RCLCPP_WARN(get_logger(), "Ignoring placement for undefined class: %s", name.c_str());
          }
        } catch (const std::exception &) {RCLCPP_WARN(get_logger(), "Ignoring invalid placement key: %s", name.c_str());}
      }
    }
    const auto step = declare_parameter<std::vector<double>>("placement_step", {0.04, 0.06});
    if (step.size() != 2 || !std::isfinite(step[0]) || !std::isfinite(step[1]) || step[0] <= 0 || step[1] <= 0) {
      throw std::invalid_argument("placement_step requires positive finite x/y spacing");
    }
    const tf2::Vector3 spacing(step[0], step[1], 0.0);
    std::string conflict;
    for (const auto & entry : classes_) {
      const auto id = entry.first;
      const auto key = std::to_string(id);
      try {
        const auto point = declare_parameter<std::vector<double>>("place_positions." + key, std::vector<double>{});
        const auto region = declare_parameter<std::vector<double>>("place_regions." + key, std::vector<double>{});
        if (explicit_rules_) {
          // 点与区域同时配置属于配置错误：记录后在循环外失败启动，不静默禁用类别。
          if (!point.empty() && !region.empty()) {conflict = key; continue;}
          if (!region.empty()) {
            slots_[id] = regionGrid(region, regionOrigin(region), spacing, true);
            rule_kind_[id] = RuleKind::SLOT;
          } else if (!point.empty()) {
            position_values_[id] = xyz(point);
            rule_kind_[id] = RuleKind::POINT;
          }
          continue;
        }
        const auto prefix = "classes." + key;
        const auto origin_values = declare_parameter<std::vector<double>>(prefix + ".place_position", std::vector<double>{});
        auto values = declare_parameter<std::vector<double>>(prefix + ".place_slots", std::vector<double>{});
        const auto region_rule = declare_parameter<std::vector<double>>(prefix + ".place_region", std::vector<double>{});
        if (values.empty() && !region_rule.empty()) {
          const auto origin = origin_values.empty() ? regionOrigin(region_rule) : xyz(origin_values);
          const auto points = regionGrid(region_rule, origin, spacing, false);
          for (const auto & point : points) {values.insert(values.end(), {point.x(), point.y(), point.z()});}
        }
        if (values.empty()) {values = origin_values;}
        if (values.size() % 3 != 0) {throw std::invalid_argument("slots require xyz triples");}
        for (size_t i = 0; i < values.size(); i += 3) {slots_[id].push_back(xyz({values[i], values[i + 1], values[i + 2]}));}
        if (!values.empty()) {set_parameter(rclcpp::Parameter(prefix + ".place_slots", values));}
        if (slots_[id].empty()) {slots_.erase(id);}
        else {rule_kind_[id] = RuleKind::SLOT;}
      } catch (const std::exception & error) {
        slots_.erase(id); position_values_.erase(id); rule_kind_.erase(id);
        RCLCPP_WARN(get_logger(), "Class %u placement disabled: %s", id, error.what());
      }
    }
    if (!conflict.empty()) {
      throw std::invalid_argument("Class " + conflict + " defines both place_positions and place_regions");
    }
    if (!validPolicy(default_strategy_, default_order_) || frame_.empty() || tool_frame_.empty() ||
      (finish_mode_ != "batch" && finish_mode_ != "watch") ||
      (retry_policy_ != "priority" && retry_policy_ != "immediate" && retry_policy_ != "disabled") ||
      (exhausted_policy_ != "continue" && exhausted_policy_ != "stop") ||
      (slots_full_policy_ != "wait" && slots_full_policy_ != "finish") ||
      (verification_ != "visual" && verification_ != "motion") || max_attempts_ < 1 ||
      max_candidate_attempts_ < 1 || max_candidate_reevaluations_ < 0 || max_objects_ < 0)
    {throw std::invalid_argument("Invalid task policy");}
    for (const double value : {max_age_, timeout_, action_timeout_, recovery_timeout_,
      verify_timeout_, slot_tolerance_, height_tolerance_})
    {
      if (!std::isfinite(value) || value <= 0) {throw std::invalid_argument("Invalid timeout/tolerance");}
    }
    if (!std::isfinite(candidate_max_age_) || candidate_max_age_ <= 0) {
      throw std::invalid_argument("grasp_candidate_max_age must be positive");
    }
    if (!std::isfinite(candidate_max_drift_) || candidate_max_drift_ < 0) {
      throw std::invalid_argument("grasp_candidate_max_drift must be non-negative");
    }
    for (const double value : {idle_timeout_, cooldown_, settle_time_, slot_clearance_}) {
      if (!std::isfinite(value) || value < 0) {throw std::invalid_argument("Invalid wait/clearance");}
    }
    subscription_ = create_subscription<Objects>(
      declare_parameter<std::string>("objects_topic", "/perception/objects"), rclcpp::SensorDataQoS(),
      [this](Objects::ConstSharedPtr objects) {observations_ = objects; last_received_ = now(); ++sequence_;});
    // 可选通道：没有发布者或数据过期时缓存为空，派发自动退回传统抓取。
    candidates_subscription_ = create_subscription<Candidate>(
      declare_parameter<std::string>("grasp_candidates_topic", "/grasp/candidates"), 10,
      [this](Candidate::ConstSharedPtr candidate) {
        if (candidate->object_id.empty()) {return;}
        auto & entry = candidates_[candidate->object_id];
        if (entry.received.nanoseconds() != 0 &&
          rclcpp::Time(candidate->header.stamp) < rclcpp::Time(entry.msg.header.stamp))
        {// 乱序到达的旧候选不得覆盖较新的那条。
          return;
        }
        entry.msg = *candidate; entry.received = now();
        // 记下候选到达时该物体的观测中心：派发时用它判断物体是否已被移动。
        entry.has_center = false;
        if (observations_) {
          for (const auto & object : observations_->objects) {
            if (object.object_id == candidate->object_id) {
              entry.center = position(object); entry.has_center = true; break;
            }
          }
        }
      });
    pick_client_ = rclcpp_action::create_client<Pick>(this,
      declare_parameter<std::string>("pick_place_action", "/manipulation/pick_place"));
    named_client_ = rclcpp_action::create_client<Named>(this,
      declare_parameter<std::string>("move_to_named_action", "/manipulation/move_to_named"));
    recovery_client_ = create_client<Trigger>(declare_parameter<std::string>(
      "recovery_service", "/manipulation/release_held_object"));
    server_ = rclcpp_action::create_server<Sort>(this,
      declare_parameter<std::string>("action_name", "/sorting/sort_objects"),
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Sort::Goal> goal) {
        if (busy_ || (!goal->class_order.empty() && !goal->color_order.empty()) || !validPolicy(goal->strategy.empty() ? default_strategy_ : goal->strategy,
          goalOrder(*goal)))
        {return rclcpp_action::GoalResponse::REJECT;}
        busy_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<SortHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<SortHandle> handle) {
        task_ = handle;
        strategy_ = handle->get_goal()->strategy.empty() ? default_strategy_ : handle->get_goal()->strategy;
        order_ = goalOrder(*handle->get_goal());
        attempts_.clear();
        pending_verifications_.clear();
        skipped_.clear();
        slot_failures_.clear();
        rule_failures_.clear();
        precondition_failures_.clear();
        rejection_counts_.clear();
        scene_signature_.clear();
        shortage_ = Shortage::NONE;
        rule_targets_.clear();
        completed_ = 0;
        verifying_ = idle_ = cancel_sent_ = false;
        last_failed_.clear();
        restartObservation();
        feedback("");
      });
    // 单 executor 更新全部任务状态；运动和恢复均异步，不新增调度线程或 outbox。
    timer_ = create_wall_timer(100ms, [this]() {tick();});
    RCLCPP_INFO(get_logger(), "Adaptive planner ready (%s, retries=%d)", finish_mode_.c_str(), max_attempts_);
  }

private:
  static tf2::Vector3 xyz(const std::vector<double> & v)
  {
    if (v.size() != 3 || !std::all_of(v.begin(), v.end(), [](double x) {return std::isfinite(x);})) {
      throw std::invalid_argument("Expected three finite xyz values");
    }
    return {v[0], v[1], v[2]};
  }
  static tf2::Vector3 position(const Object & o)
  {return {o.pose.position.x, o.pose.position.y, o.pose.position.z};}
  // 区域规则按间距生成候选；区域坐标始终是 base_link 下的绝对值。
  // 绝对规则先试半径小的候选：固定向下姿态下远点更容易先撞 joint5 限位。
  // 先校验长度与边界再取元素：短数组直接抛错，不能越界访问。
  static tf2::Vector3 regionOrigin(const std::vector<double> & region)
  {
    if (region.size() != 5 || !std::all_of(region.begin(), region.end(), [](double v) {return std::isfinite(v);}) ||
      region[0] > region[2] || region[1] > region[3])
    {throw std::invalid_argument("region requires [xmin,ymin,xmax,ymax,z] with min <= max");}
    return {region[0], region[1], region[4]};
  }
  static std::vector<tf2::Vector3> regionGrid(const std::vector<double> & region,
    const tf2::Vector3 & origin, const tf2::Vector3 & step, bool nearest_first)
  {
    if (region.size() != 5 || !std::all_of(region.begin(), region.end(), [](double v) {return std::isfinite(v);}) ||
      region[0] > region[2] || region[1] > region[3])
    {throw std::invalid_argument("region requires [xmin,ymin,xmax,ymax,z] with min <= max");}
    std::vector<tf2::Vector3> points;
    const int xmin = static_cast<int>(std::ceil((region[0] - origin.x()) / step.x() - 1e-9));
    const int xmax = static_cast<int>(std::floor((region[2] - origin.x()) / step.x() + 1e-9));
    const int ymin = static_cast<int>(std::ceil((region[1] - origin.y()) / step.y() - 1e-9));
    const int ymax = static_cast<int>(std::floor((region[3] - origin.y()) / step.y() + 1e-9));
    for (int y = ymin; y <= ymax; ++y) {
      for (int x = xmin; x <= xmax; ++x) {
        points.emplace_back(origin.x() + x * step.x(), origin.y() + y * step.y(), region[4]);
      }
    }
    if (points.empty()) {throw std::invalid_argument("region produced no candidate points");}
    if (nearest_first) {
      std::stable_sort(points.begin(), points.end(),
        [](const auto & a, const auto & b) {return std::hypot(a.x(), a.y()) < std::hypot(b.x(), b.y());});
    } else {
      std::stable_sort(points.begin(), points.end(),
        [&](const auto & a, const auto & b) {return (a - origin).length2() < (b - origin).length2();});
    }
    return points;
  }
  static bool validPolicy(const std::string & strategy, const std::vector<Class> & order)
  {
    const std::set<Class> ids(order.begin(), order.end());
    return (strategy == "class_order" || strategy == "color_order" || strategy == "nearest") &&
           ids.size() == order.size() && (ids.empty() || *ids.begin() > 0);
  }
  std::vector<Class> goalOrder(const Sort::Goal & goal) const
  {
    if (!goal.class_order.empty()) {return goal.class_order;}
    if (goal.color_order.empty()) {return default_order_;}
    std::vector<Class> result;
    for (const auto color : goal.color_order) {
      for (const auto & entry : classes_) {if (entry.second.color == color) {result.push_back(entry.first);}}
    }
    return result;
  }
  bool hasPlacement(Class id) const
  {return pointRule(id) ? position_values_.count(id) > 0 : slots_.count(id) > 0;}
  bool pointRule(Class id) const
  {
    const auto rule = rule_kind_.find(id);
    return rule != rule_kind_.end() && rule->second == RuleKind::POINT;
  }
  void skip(const Object & object, const std::string & reason)
  {
    if (skipped_.emplace(object.object_id, reason).second) {
      RCLCPP_WARN(get_logger(), "Skipping %s class=%u: %s", object.object_id.c_str(), object.class_id, reason.c_str());
    }
  }
  static const char * precheckName(uint8_t stage)
  {
    switch (stage) {
      case 1: return "pre_grasp_ik";
      case 2: return "place_ik";
      case 3: return "release_space";
      case 4: return "precondition";
      default: return "none";
    }
  }
  void noteCandidateFailure(uint8_t reason)
  {
    CandidateFailure & failure = pointRule(active_object_.class_id) ?
      rule_failures_[active_object_.object_id] : slot_failures_[active_slot_];
    ++failure.attempts;
    failure.reason = reason;
    ++rejection_counts_[reason];
    RCLCPP_WARN(get_logger(), "Placement candidate for %s rejected (%s, %d/%d)",
      active_object_.object_id.c_str(), precheckName(reason), failure.attempts, max_candidate_attempts_);
  }
  // 场景变化只让每个候选复活有限次：目标冻结时无限重评会形成活锁。
  template<typename Map>
  int rearmCandidateFailures(Map & failures)
  {
    int rearmed = 0;
    for (auto & entry : failures) {
      auto & failure = entry.second;
      if (failure.attempts < max_candidate_attempts_ ||
        failure.reevaluations >= max_candidate_reevaluations_) {continue;}
      failure.attempts = 0;
      ++failure.reevaluations;
      ++rearmed;
    }
    return rearmed;
  }
  // 物体位置量化到 1 cm；有意义的环境变化才允许重新评估被拒候选。
  static std::string sceneSignature(const std::vector<Object> & objects)
  {
    std::vector<std::string> parts;
    for (const auto & o : objects) {
      if (o.tracking_uncertain) {continue;}
      const auto & p = o.pose.position;
      parts.push_back(o.object_id + "@" + std::to_string(std::lround(p.x * 100)) + "," +
        std::to_string(std::lround(p.y * 100)) + "," + std::to_string(std::lround(p.z * 100)));
    }
    std::sort(parts.begin(), parts.end());
    std::string result;
    for (const auto & part : parts) {result += part + ";";}
    return result;
  }
  std::string shortageText() const
  {
    if (shortage_ == Shortage::REJECTED) {
      return "Placement candidates rejected by precheck; no alternative target";
    }
    return "Placement area full";
  }
  static tf2::Vector3 extent(const Object & object)
  {
    tf2::Quaternion q; tf2::fromMsg(object.pose.orientation, q); q.normalize();
    const tf2::Matrix3x3 rotation(q);
    const auto & d = object.dimensions;
    tf2::Vector3 result;
    for (int i = 0; i < 3; ++i) {
      result[i] = (std::abs(rotation[i][0]) * d.x + std::abs(rotation[i][1]) * d.y + std::abs(rotation[i][2]) * d.z) / 2;
    }
    return result;
  }
  bool inWorkspace(const Object & o) const
  {
    if (workspace_min_.empty()) {return true;}
    const auto p = position(o);
    for (int i = 0; i < 3; ++i) {
      if (p[i] < workspace_min_[i] || p[i] > workspace_max_[i]) {return false;}
    }
    return true;
  }
  tf2::Vector3 ruleGoal(const Object & o) const
  {
    const auto existing = rule_targets_.find(o.object_id);
    if (existing != rule_targets_.end()) {return existing->second->goal;}
    auto goal = position_values_.at(o.class_id);
    const auto source = position(o);
    for (int axis = 0; axis < 3; ++axis) {
      if (position_modes_[axis] == "relative") {goal[axis] += source[axis];}
    }
    return goal;
  }
  bool near(const tf2::Vector3 & a, const tf2::Vector3 & b) const
  {
    return std::hypot(a.x() - b.x(), a.y() - b.y()) <= slot_tolerance_ &&
           std::abs(a.z() - b.z()) <= height_tolerance_;
  }
  bool classIssue(Class id) const
  {
    const auto prefix = "class " + std::to_string(id) + ":";
    return std::any_of(observations_->issues.begin(), observations_->issues.end(),
      [&](const auto & issue) {return issue.rfind(prefix, 0) == 0;});
  }
  void markPlaced(const Object & source, Slot slot, const std::string & observed_id)
  {
    if (pointRule(slot.first)) {
      auto target = rule_targets_.at(source.object_id);
      target->state = RuleState::PLACED;
      if (!observed_id.empty()) {rule_targets_[observed_id] = target;}
    } else {
      occupied_[slot] = observed_id;
      occupied_extent_[slot] = extent(source);
    }
  }
  void confirmPlacement(const Object & source, Slot slot, const std::string & observed_id)
  {
    markPlaced(source, slot, observed_id);
    attempts_.erase(source.object_id);
    ++completed_;
    feedback(source.object_id);
    RCLCPP_INFO(get_logger(), "Placement verified: %s", source.object_id.c_str());
  }
  void reserveSlot(const Slot & slot)
  {
    occupied_[slot] = "";
    occupied_extent_[slot] = extent(active_object_);
  }
  void releaseReservation(const std::string & id, Slot slot)
  {
    if (pointRule(slot.first)) {rule_targets_.at(id)->state = RuleState::READY;}
    else {
      occupied_.erase(slot);
      occupied_extent_.erase(slot);
    }
  }
  bool sorted(const Object & o) const
  {
    if (!hasPlacement(o.class_id)) {return false;}
    if (pointRule(o.class_id)) {
      if (near(position(o), ruleGoal(o))) {return true;}
      return std::any_of(rule_targets_.begin(), rule_targets_.end(), [&](const auto & entry) {
        const auto & target = *entry.second;
        return target.class_id == o.class_id && target.state == RuleState::PLACED && near(position(o), target.goal);
      });
    }
    return std::any_of(slots_.at(o.class_id).begin(), slots_.at(o.class_id).end(), [&](const auto & slot) {
      return near(position(o), slot);
    });
  }
  std::vector<Object> objectsInBase()
  {
    const auto & header = observations_->header;
    if (header.frame_id.empty()) {throw std::runtime_error("Observation has no frame_id");}
    geometry_msgs::msg::TransformStamped transform;
    if (header.frame_id != frame_) {
      transform = tf_buffer_.lookupTransform(frame_, header.frame_id, rclcpp::Time(header.stamp));
    }
    std::map<std::string, int> counts;
    for (const auto & o : observations_->objects) {++counts[o.object_id];}
    std::vector<Object> objects;
    for (auto o : observations_->objects) {
      const auto & d = o.dimensions;
      tf2::Quaternion q;
      tf2::fromMsg(o.pose.orientation, q);
      const auto p = position(o);
      if (o.object_id.empty() || counts[o.object_id] != 1 ||
        !std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()) ||
        !std::isfinite(d.x) || !std::isfinite(d.y) || !std::isfinite(d.z) ||
        d.x <= 0 || d.y <= 0 || d.z <= 0 || !std::isfinite(q.length2()) || q.length2() < 1e-12)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring invalid/duplicate object observation");
        continue;
      }
      q.normalize();
      o.pose.orientation = tf2::toMsg(q);
      if (header.frame_id != frame_) {
        geometry_msgs::msg::PoseStamped source, result;
        source.header = header; source.pose = o.pose;
        tf2::doTransform(source, result, transform);
        o.pose = result.pose;
      }
      objects.push_back(o);
    }
    if (objects.empty() && !observations_->objects.empty()) {
      throw std::runtime_error("No valid entries in nonempty observation");
    }
    return objects;
  }
  void feedback(const std::string & id)
  {
    auto update = std::make_shared<Sort::Feedback>();
    update->current_object_id = id;
    update->completed_count = completed_;
    task_->publish_feedback(update);
  }
  void finish(bool success, const std::string & message, bool canceled = false)
  {
    auto result = std::make_shared<Sort::Result>();
    result->success = success;
    result->message = message + "; verified/completed=" + std::to_string(completed_) +
      ", unresolved_attempted_targets=" + std::to_string(attempts_.size());
    for (const auto & entry : attempts_) {
      result->message += " [" + entry.first + ":" + std::to_string(entry.second.count) + " attempts]";
    }
    for (const auto & entry : rejection_counts_) {
      result->message += " [candidate " + std::string(precheckName(entry.first)) + ":" +
        std::to_string(entry.second) + "]";
    }
    result->message += ", skipped_targets=" + std::to_string(skipped_.size());
    for (const auto & entry : skipped_) {result->message += " [" + entry.first + ":" + entry.second + "]";}
    if (observations_) {
      for (const auto & issue : observations_->issues) {result->message += " [perception:" + issue + "]";}
    }
    if (canceled) {task_->canceled(result);} else if (success) {task_->succeed(result);} else {task_->abort(result);}
    RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
    task_.reset(); active_pick_.reset(); active_named_.reset(); busy_ = false;
  }
  void waiting(const std::string & reason)
  {
    idle_ = false;  // 故障或无数据不能充当“健康空场景”。默认等待恢复，不结束任务。
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), static_cast<int64_t>(timeout_ * 1000), "%s", reason.c_str());
  }
  void restartObservation()
  {
    phase_ = Phase::WAITING;
    needs_observation_ = !observation_target_.empty();
    observations_.reset();
    observations_after_ = now() + rclcpp::Duration::from_seconds(settle_time_);
    idle_ = cancel_sent_ = false;
    next_try_ = Clock::now();
    verification_since_ = now() + rclcpp::Duration::from_seconds(settle_time_);
  }
  void observe()
  {
    phase_ = Phase::OBSERVING;
    action_since_ = Clock::now();
    cancel_sent_ = false;
    const auto task = task_;
    rclcpp_action::Client<Named>::SendGoalOptions options;
    options.goal_response_callback = [this, task](NamedHandle::SharedPtr handle) {
      if (task_ != task) {return;}
      if (!handle) {
        if (task->is_canceling()) {finish(false, "Canceled", true); return;}
        phase_ = Phase::WAITING;
        next_try_ = Clock::now() + 1s;
        waiting("Observation motion rejected; retrying");
      } else {active_named_ = handle;}
    };
    options.result_callback = [this, task](const NamedHandle::WrappedResult & result) {
      if (task_ != task) {return;}
      active_named_.reset();
      if (task->is_canceling()) {finish(false, "Canceled after observation motion", true); return;}
      restartObservation();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result && result.result->success) {
        needs_observation_ = false;
        verification_since_ = now();
      } else {
        next_try_ = Clock::now() + 1s;
        waiting("Observation motion failed; retrying");
      }
    };
    Named::Goal goal; goal.target = observation_target_;
    named_client_->async_send_goal(goal, options);
  }
  void failed(const std::string & message)
  {
    last_failed_ = active_object_.object_id;
    auto & attempt = attempts_[last_failed_];
    if (retry_policy_ == "disabled") {attempt.count = max_attempts_;}
    attempt.retry_after = now() + rclcpp::Duration::from_seconds(cooldown_);
    RCLCPP_WARN(get_logger(), "Target %s attempt %d/%d: %s", last_failed_.c_str(),
      attempt.count, max_attempts_, message.c_str());
    if (attempt.count >= max_attempts_ && exhausted_policy_ == "stop") {
      finish(false, "Attempt limit: " + last_failed_);
    }
  }
  void startRecovery(bool holding)
  {
    phase_ = Phase::RECOVERING;
    recovery_holding_ = holding;
    recovery_since_ = Clock::now();
    recovery_pending_ = false;
    next_try_ = Clock::now();
  }
  void recover()
  {
    if (task_->is_canceling() && !recovery_pending_) {
      finish(false, "Canceled; recovery stopped and load retained if present", true);
      return;
    }
    if (Clock::now() - recovery_since_ > std::chrono::duration<double>(recovery_timeout_)) {
      if (recovery_pending_) {recovery_client_->remove_pending_request(recovery_request_id_);}
      finish(false, "Recovery not confirmed; executor state unknown (request may still be active)", task_->is_canceling());
      return;
    }
    if (recovery_pending_ || Clock::now() < next_try_) {return;}
    if (!recovery_client_->service_is_ready()) {waiting("Waiting for recovery service"); return;}
    const auto task = task_;
    recovery_pending_ = true;
    const auto request = recovery_client_->async_send_request(std::make_shared<Trigger::Request>(),
      [this, task](rclcpp::Client<Trigger>::SharedFuture future) {
        if (task_ != task) {return;}
        recovery_pending_ = false;
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_WARN(get_logger(), "Recovery retry: %s", response->message.c_str());
          next_try_ = Clock::now() + 1s;
          return;
        }
        if (task->is_canceling()) {finish(false, "Canceled after recovery", true); return;}
        verifying_ = recovery_holding_;
        if (verifying_) {
          if (pointRule(active_object_.class_id)) {rule_targets_.at(active_object_.object_id)->state = RuleState::RESERVED;}
          else {reserveSlot(active_slot_);}
        }
        if (!verifying_) {failed("Recovered; reacquiring target"); if (!task_) {return;}}
        restartObservation();
      });
    recovery_request_id_ = request.request_id;
  }
  bool blocked(const Object & object, const tf2::Vector3 & target, const std::vector<Object> & objects) const
  {
    const auto own = extent(object);
    return std::any_of(objects.begin(), objects.end(), [&](const Object & other) {
      if (other.object_id == object.object_id) {return false;}
      const auto delta = position(other) - target;
      const auto total = own + extent(other);
      return std::abs(delta.x()) < total.x() + slot_clearance_ &&
             std::abs(delta.y()) < total.y() + slot_clearance_ &&
             std::abs(delta.z()) < total.z() + slot_clearance_;
    });
  }
  // 汇总已预留、但当前可能漏检的目标：点规则用冻结目标，槽位/区域规则用占用槽位。
  std::vector<Reservation> reservations(const Object & object) const
  {
    std::vector<Reservation> result;
    const auto own_target = rule_targets_.find(object.object_id);
    for (const auto & entry : rule_targets_) {
      const auto & target = *entry.second;
      if (target.state == RuleState::READY ||
        (own_target != rule_targets_.end() && own_target->second == entry.second)) {continue;}
      result.push_back({target.goal, target.extent});
    }
    for (const auto & entry : occupied_) {
      const auto known = occupied_extent_.find(entry.first);
      if (known == occupied_extent_.end()) {continue;}
      result.push_back({slots_.at(entry.first.first)[entry.first.second], known->second});
    }
    return result;
  }
  bool overlaps(const Object & object, const tf2::Vector3 & goal,
    const std::vector<Reservation> & reserved) const
  {
    const auto own = extent(object);
    return std::any_of(reserved.begin(), reserved.end(), [&](const auto & r) {
      return std::abs(goal.x() - r.goal.x()) < own.x() + r.extent.x() + slot_clearance_ &&
             std::abs(goal.y() - r.goal.y()) < own.y() + r.extent.y() + slot_clearance_ &&
             std::abs(goal.z() - r.goal.z()) < own.z() + r.extent.z() + slot_clearance_;
    });
  }
  int freeSlot(const Object & object, const std::vector<Object> & objects,
    Shortage * shortage = nullptr)
  {
    if (shortage) {*shortage = Shortage::NONE;}
    const auto reserved = reservations(object);
    if (pointRule(object.class_id)) {
      const auto failed = rule_failures_.find(object.object_id);
      if (failed != rule_failures_.end() &&
        failed->second.attempts >= max_candidate_attempts_)
      {
        if (shortage) {*shortage = Shortage::REJECTED;}
        return -1;
      }
      const auto goal = ruleGoal(object);
      if (blocked(object, goal, objects) || overlaps(object, goal, reserved)) {
        if (shortage) {*shortage = Shortage::OCCUPIED;}
        return -1;
      }
      // 单点规则没有其他候选；被拒但未达上限时重试本目标。
      return 0;
    }
    int retry = -1;
    for (size_t i = 0; i < slots_.at(object.class_id).size(); ++i) {
      const Slot slot{object.class_id, i};
      if (occupied_.count(slot)) {
        if (shortage) {*shortage = Shortage::OCCUPIED;}
        continue;
      }
      if (blocked(object, slots_.at(object.class_id)[i], objects) ||
        overlaps(object, slots_.at(object.class_id)[i], reserved)) {
        if (shortage) {*shortage = Shortage::OCCUPIED;}
        continue;
      }
      const auto failed = slot_failures_.find(slot);
      if (failed != slot_failures_.end()) {
        if (failed->second.attempts >= max_candidate_attempts_) {
          if (shortage) {*shortage = Shortage::REJECTED;}
          continue;
        }
        // 先尝试从未失败的候选；都试过后再按上限重试被拒候选。
        if (retry < 0) {retry = static_cast<int>(i);}
        continue;
      }
      return static_cast<int>(i);
    }
    if (retry >= 0) {return retry;}
    return -1;
  }
  void updateOccupancy(const std::vector<Object> & objects)
  {
    // 相对规则：目标随物体冻结，按目标位维护状态与跨 ID 关联。
    for (const auto & o : objects) {
      if (o.tracking_uncertain || !hasPlacement(o.class_id) || !pointRule(o.class_id)) {continue;}
      const auto known = rule_targets_.find(o.object_id);
      if (known != rule_targets_.end() && known->second->state == RuleState::PLACED &&
        !near(position(o), known->second->goal)) {known->second->state = RuleState::READY;}
      // 视觉 ID 变化时，把已核验目标位的新 ID 关联到同一条规则记录。
      for (const auto & entry : rule_targets_) {
        if (entry.second->state == RuleState::PLACED && entry.second->class_id == o.class_id &&
          near(position(o), entry.second->goal)) {
          rule_targets_[o.object_id] = entry.second;
          break;
        }
      }
    }
    // 槽位/绝对规则：只有已关联物体明确移开才释放缓存；漏检/离开视野不等于槽位已空。
    for (auto it = occupied_.begin(); it != occupied_.end();) {
      const auto moved = std::find_if(objects.begin(), objects.end(), [&](const auto & o) {
        return !o.tracking_uncertain && o.object_id == it->second &&
               !near(position(o), slots_.at(it->first.first)[it->first.second]);
      });
      if (moved != objects.end()) {
        occupied_extent_.erase(it->first);
        it = occupied_.erase(it);
      } else {++it;}
    }
    for (const auto & o : objects) {
      if (o.tracking_uncertain || !hasPlacement(o.class_id) || pointRule(o.class_id)) {continue;}
      for (size_t i = 0; i < slots_.at(o.class_id).size(); ++i) {
        if (near(position(o), slots_.at(o.class_id)[i])) {
          occupied_[{o.class_id, i}] = o.object_id;
          occupied_extent_[{o.class_id, i}] = extent(o);
          if (!verifying_ || o.object_id != active_object_.object_id) {attempts_.erase(o.object_id);}
        }
      }
    }
  }
  PlacementCheck checkPlacement(const Object & source, const tf2::Vector3 & target,
    const std::vector<Object> & objects) const
  {
    PlacementCheck result;
    if (classIssue(source.class_id)) {return result;}
    for (const auto & o : objects) {
      if (o.class_id != source.class_id || o.tracking_uncertain) {continue;}
      if (near(position(o), target)) {result.destination = &o;}
      if (!sorted(o) && near(position(o), position(source))) {result.at_source = true;}
      if (o.object_id == source.object_id &&
        (std::abs(o.pose.position.z - target.z()) <= height_tolerance_ ||
        std::abs(o.pose.position.z - source.pose.position.z) <= height_tolerance_))
      {result.reliable = true;}
    }
    return result;
  }
  void verifyPending(const std::vector<Object> & objects)
  {
    for (auto it = pending_verifications_.begin(); it != pending_verifications_.end();) {
      const auto & pending = it->second;
      const auto check = checkPlacement(pending.source, pending.target, objects);
      if (check.destination && !check.at_source) {
        confirmPlacement(pending.source, pending.slot, check.destination->object_id);
      } else if (!check.at_source && !check.reliable) {++it; continue;}
      else if (check.at_source) {releaseReservation(it->first, pending.slot);}
      it = pending_verifications_.erase(it);
    }
  }
  bool verify(const std::vector<Object> & objects)
  {
    if (!verifying_) {return false;}
    const auto check = checkPlacement(active_object_, active_destination_, objects);
    if (check.destination && !check.at_source) {
      confirmPlacement(active_object_, active_slot_, check.destination->object_id);
      verifying_ = false;
    } else if ((now() - verification_since_).seconds() >= verify_timeout_) {
      verifying_ = false;
      if (!check.at_source && !check.reliable) {
        pending_verifications_[active_object_.object_id] = {active_object_, active_destination_, active_slot_};
        restartObservation();
        waiting("Placement observation uncertain; retaining reservation and observing again");
        return true;
      }
      // 来源仍有物块、目标没有：很可能抓空，重新获取位置并有限重试。
      if (check.at_source && !check.destination) {releaseReservation(active_object_.object_id, active_slot_);}
      failed(check.at_source ? "Object remains at source" : "Placement unconfirmed; reserved slot retained");
    } else {return true;}
    return false;
  }
  Object selectTarget(const std::vector<Object> & candidates)
  {
    tf2::Vector3 tcp(0, 0, 0);
    try {
      const auto transform = tf_buffer_.lookupTransform(frame_, tool_frame_, tf2::TimePointZero);
      tf2::Transform tool; tf2::fromMsg(transform.transform, tool); tcp = tool * tool_to_tcp_;
    } catch (const tf2::TransformException &) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TCP unavailable; using base distance");
    }
    const auto rank = [&](const Object & o) {
      return strategy_ != "nearest" ? std::find(order_.begin(), order_.end(), o.class_id) - order_.begin() : 0;
    };
    return *std::min_element(candidates.begin(), candidates.end(), [&](const auto & a, const auto & b) {
      if (retry_policy_ == "immediate" && (a.object_id == last_failed_) != (b.object_id == last_failed_)) {
        return a.object_id == last_failed_;
      }
      if (rank(a) != rank(b)) {return rank(a) < rank(b);}
      const double da = (position(a) - tcp).length2(), db = (position(b) - tcp).length2();
      return da != db ? da < db : a.object_id < b.object_id;
    });
  }
  // 新鲜候选：时戳在阈值内、坐标系有效且物体自候选到达后未被移动，否则当作无候选。
  // 物体位移判据取代了单纯的时戳代理：物体被推开后就不再抓它的旧位置。
  // 不做模型健康检查、不重试推理：降级是数据的缺席，不是代码的分支。
  const Candidate * freshCandidate(const Object & object)
  {
    const auto it = candidates_.find(object.object_id);
    if (it == candidates_.end()) {return nullptr;}
    const auto & entry = it->second;
    if (entry.msg.tcp_pose.header.frame_id.empty()) {return nullptr;}
    const double age = (now() - rclcpp::Time(entry.msg.header.stamp)).seconds();
    // 时钟回跳或候选盖在未来时戳上：age 为负不可信，不当作新鲜。
    if (!std::isfinite(age) || age < 0.0 || age > candidate_max_age_) {return nullptr;}
    if (entry.has_center && (position(object) - entry.center).length() > candidate_max_drift_) {
      return nullptr;
    }
    return &entry.msg;
  }
  void dispatch(const Object & object, size_t slot)
  {
    active_object_ = object;
    active_slot_ = {object.class_id, slot};
    Pick::Goal goal;
    goal.object_id = object.object_id;
    goal.class_id = object.class_id;
    goal.object_pose.header = observations_->header;
    goal.object_pose.header.frame_id = frame_;
    goal.object_pose.pose = object.pose;
    goal.object_dimensions = object.dimensions;
    goal.place_pose.header.frame_id = frame_;
    if (pointRule(object.class_id)) {
      const auto target = ruleGoal(object);
      if (!rule_targets_.count(object.object_id)) {
        rule_targets_[object.object_id] = std::make_shared<RuleTarget>(RuleTarget{object.class_id, target, extent(object)});
      }
      active_destination_ = target;
      RCLCPP_INFO(get_logger(), "Frozen placement %s class=%u modes=[%s,%s,%s] in %s: [%.4f, %.4f, %.4f]",
        object.object_id.c_str(), object.class_id, position_modes_[0].c_str(), position_modes_[1].c_str(),
        position_modes_[2].c_str(), frame_.c_str(), target.x(), target.y(), target.z());
    } else {active_destination_ = slots_.at(object.class_id)[slot];}
    const auto p = active_destination_;
    goal.place_pose.pose.position.x = p.x(); goal.place_pose.pose.position.y = p.y();
    goal.place_pose.pose.position.z = p.z(); goal.place_pose.pose.orientation = object.pose.orientation;
    // 候选只做查表透传：planner 不解释姿态，也不向模型提问。
    const auto * candidate = freshCandidate(object);
    if (candidate) {
      goal.grasp_pose = candidate->tcp_pose;
      goal.grasp_pose.header.frame_id = frame_;
      goal.grasp_width = candidate->width;
      goal.grasp_source = "graspnet";
    }
    feedback(object.object_id);
    RCLCPP_INFO(get_logger(), "Pick %s class=%u (%.4f, %.4f, %.4f) -> slot %zu (%.4f, %.4f, %.4f)",
      object.object_id.c_str(), object.class_id, object.pose.position.x, object.pose.position.y, object.pose.position.z,
      slot, p.x(), p.y(), p.z());
    if (candidate) {
      const auto * pose = &candidate->tcp_pose.pose;
      RCLCPP_INFO(get_logger(), "Grasp candidate for %s: tcp=(%.4f, %.4f, %.4f) width=%.3f age=%.2fs",
        object.object_id.c_str(), pose->position.x, pose->position.y, pose->position.z,
        candidate->width, (now() - rclcpp::Time(candidate->header.stamp)).seconds());
    } else {
      RCLCPP_INFO(get_logger(), "No fresh grasp candidate for %s; using traditional descent",
        object.object_id.c_str());
    }
    phase_ = Phase::SENDING;
    idle_ = cancel_sent_ = false;
    action_since_ = Clock::now();
    const auto task = task_;
    rclcpp_action::Client<Pick>::SendGoalOptions options;
    options.goal_response_callback = [this, task](PickHandle::SharedPtr handle) {
      if (task_ != task) {return;}
      if (!handle) {
        if (task->is_canceling()) {finish(false, "Canceled", true); return;}
        restartObservation(); next_try_ = Clock::now() + 1s;
        waiting("PickPlace rejected; reacquiring without consuming attempt");
      } else {
        active_pick_ = handle; phase_ = Phase::EXECUTING;
        ++attempts_[active_object_.object_id].count;
      }
    };
    options.result_callback = [this, task](const PickHandle::WrappedResult & result) {
      if (task_ != task) {return;}
      active_pick_.reset();
      if (task->is_canceling()) {
        finish(false, "Canceled; executor load retained if present", true); return;
      }
      if (!result.result) {startRecovery(false); return;}
      const auto & r = *result.result;
      if (r.holding) {startRecovery(true); return;}
      if (r.precheck_failure == Pick::Result::PRECHECK_PRECONDITION) {
        // 观测、尺寸、朝向等前提错误：不消耗抓取尝试，也不判候选不可用；有限次后跳过该物体。
        const auto failures = ++precondition_failures_[active_object_.object_id];
        auto & count = attempts_[active_object_.object_id].count;
        if (count > 0 && --count == 0) {attempts_.erase(active_object_.object_id);}
        if (failures >= max_attempts_) {
          skip(active_object_, "precondition: " + r.message);
        } else {
          RCLCPP_WARN(get_logger(), "Precondition failure for %s (%d/%d): %s",
            active_object_.object_id.c_str(), failures, max_attempts_, r.message.c_str());
        }
        restartObservation();
        return;
      }
      if (!r.ready_for_next_pick) {startRecovery(false); return;}
      if (r.unreachable_placement) {
        noteCandidateFailure(r.precheck_failure);
        // 放置预检失败发生在抓取前，退款物体尝试次数；候选按自身上限重试或切换。
        auto & count = attempts_[active_object_.object_id].count;
        if (--count <= 0) {attempts_.erase(active_object_.object_id);}
      } else if (r.released) {
        if (pointRule(active_object_.class_id)) {rule_targets_.at(active_object_.object_id)->state = RuleState::RESERVED;}
        else {reserveSlot(active_slot_);}
        if (verification_ == "motion") {
          markPlaced(active_object_, active_slot_, "");
          attempts_.erase(active_object_.object_id);
          ++completed_; feedback(active_object_.object_id);
        } else {verifying_ = true;}
      } else {failed(r.message); if (!task_) {return;}}
      restartObservation();
    };
    pick_client_->async_send_goal(goal, options);
  }
  void tick()
  {
    if (!task_) {return;}
    if (phase_ == Phase::RECOVERING) {recover(); return;}
    if (task_->is_canceling()) {
      if (phase_ == Phase::WAITING) {finish(false, "Canceled", true);}
      else if (active_pick_ && !cancel_sent_) {cancel_sent_ = true; pick_client_->async_cancel_goal(active_pick_);}
      else if (active_named_ && !cancel_sent_) {cancel_sent_ = true; named_client_->async_cancel_goal(active_named_);}
      return;
    }
    if (phase_ != Phase::WAITING) {
      if (Clock::now() - action_since_ > std::chrono::duration<double>(action_timeout_) && !cancel_sent_) {
        if (active_pick_) {pick_client_->async_cancel_goal(active_pick_); cancel_sent_ = true;}
        if (active_named_) {named_client_->async_cancel_goal(active_named_); cancel_sent_ = true;}
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Action timeout; requesting cancel and waiting for executor state");
      }
      return;
    }
    if (Clock::now() < next_try_) {return;}
    try {
      if (needs_observation_) {
        if (named_client_->action_server_is_ready()) {observe();}
        else {waiting("Waiting for observation server");}
        return;
      }
      if (!observations_ || (now() - last_received_).seconds() > max_age_) {
        waiting("Waiting for fresh perception; task retained"); return;
      }
      const auto stamp = rclcpp::Time(observations_->header.stamp, get_clock()->get_clock_type());
      if (last_received_ < observations_after_ || (stamp.nanoseconds() != 0 && stamp < observations_after_)) {return;}
      if (stamp.nanoseconds() != 0 && ((now() - stamp).seconds() < -0.1 || (now() - stamp).seconds() > max_age_)) {
        waiting("Waiting for current observation timestamps"); return;
      }
      if (processed_sequence_ == sequence_) {return;}
      processed_sequence_ = sequence_;
      const auto objects = objectsInBase();
      // 有意义的环境变化（物体位置量化到 1 cm）允许有限次重评被拒候选。
      const auto signature = sceneSignature(objects);
      if (!scene_signature_.empty() && signature != scene_signature_) {
        const int rearmed = rearmCandidateFailures(slot_failures_) + rearmCandidateFailures(rule_failures_);
        if (rearmed > 0) {
          RCLCPP_INFO(get_logger(), "Scene changed; re-evaluating %d rejected placement candidate(s)", rearmed);
        }
      }
      scene_signature_ = signature;
      shortage_ = Shortage::NONE;
      verifyPending(objects);
      updateOccupancy(objects);
      if (verification_since_.nanoseconds() == 0 && verifying_) {verification_since_ = now();}
      if (verify(objects) || !task_) {return;}
      if (max_objects_ > 0 && completed_ >= static_cast<uint32_t>(max_objects_)) {
        finish(attempts_.empty(), "Object limit reached"); return;
      }
      std::vector<Object> candidates;
      bool deferred = !pending_verifications_.empty() || std::any_of(order_.begin(), order_.end(),
        [&](Class id) {return classIssue(id);}), no_slots = false;
      for (const auto & o : objects) {
        if (!inWorkspace(o)) {continue;}
        if (!classes_.count(o.class_id)) {skip(o, "undefined class"); continue;}
        if (std::find(order_.begin(), order_.end(), o.class_id) == order_.end()) {continue;}
        if (!hasPlacement(o.class_id)) {skip(o, "no applicable placement rule"); continue;}
        if (pending_verifications_.count(o.object_id) || classIssue(o.class_id)) {deferred = true; continue;}
        if (sorted(o)) {continue;}
        if (o.tracking_uncertain) {deferred = true; continue;}
        const auto attempt = attempts_.find(o.object_id);
        if (attempt != attempts_.end()) {
          if (attempt->second.count >= max_attempts_) {continue;}
          if (now() < attempt->second.retry_after) {deferred = true; continue;}
        }
        const auto precondition = precondition_failures_.find(o.object_id);
        if (precondition != precondition_failures_.end() && precondition->second >= max_attempts_) {
          continue;
        }
        Shortage shortage = Shortage::NONE;
        if (freeSlot(o, objects, &shortage) < 0) {
          if (shortage == Shortage::REJECTED) {shortage_ = Shortage::REJECTED;}
          else if (shortage_ == Shortage::NONE) {shortage_ = shortage;}
          no_slots = true;
          continue;
        }
        candidates.push_back(o);
      }
      if (!candidates.empty()) {
        idle_ = false;
        if (!pick_client_->action_server_is_ready()) {waiting("Waiting for PickPlace server"); return;}
        const auto object = selectTarget(candidates);
        dispatch(object, static_cast<size_t>(freeSlot(object, objects)));
      } else if (deferred || (no_slots && slots_full_policy_ == "wait")) {
        waiting(no_slots ?
          (shortage_ == Shortage::REJECTED ?
          "Placement candidates rejected by precheck; waiting for change" :
          "No free placement slot; waiting for space") :
          "Waiting for association/retry eligibility");
      } else if (finish_mode_ == "batch" && idle_timeout_ > 0) {
        if (!idle_) {idle_since_ = now(); idle_ = true;}
        if ((now() - idle_since_).seconds() >= idle_timeout_) {
          finish(attempts_.empty() && !no_slots && skipped_.empty() && observations_->issues.empty() && !classes_.empty(),
            no_slots ? shortageText() : "No remaining actionable targets");
        }
      }
    } catch (const std::exception & error) {waiting(error.what());}
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<Objects>::SharedPtr subscription_;
  rclcpp::Subscription<Candidate>::SharedPtr candidates_subscription_;
  rclcpp_action::Client<Pick>::SharedPtr pick_client_;
  rclcpp_action::Client<Named>::SharedPtr named_client_;
  rclcpp::Client<Trigger>::SharedPtr recovery_client_;
  rclcpp_action::Server<Sort>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  Objects::ConstSharedPtr observations_;
  std::map<std::string, Verification> pending_verifications_;
  std::map<std::string, CachedCandidate> candidates_;
  std::shared_ptr<SortHandle> task_;
  PickHandle::SharedPtr active_pick_;
  NamedHandle::SharedPtr active_named_;
  std::map<Class, std::vector<tf2::Vector3>> slots_;
  std::map<Class, tf2::Vector3> position_values_;
  std::map<Class, my_robot_perception::ObjectClass> classes_;
  std::map<std::string, std::string> skipped_;
  std::vector<std::string> position_modes_;
  std::map<std::string, std::shared_ptr<RuleTarget>> rule_targets_;
  tf2::Vector3 active_destination_;
  bool explicit_rules_ = false;  // 显式规则集存在时不再回退 legacy classes.*
  std::map<Class, RuleKind> rule_kind_;
  // 被预检拒绝的候选：绝对规则按槽位、相对规则按对象冻结目标记录。
  std::map<Slot, CandidateFailure> slot_failures_;
  std::map<std::string, CandidateFailure> rule_failures_;
  std::map<std::string, int> precondition_failures_;
  std::map<uint8_t, int> rejection_counts_;
  std::string scene_signature_;
  Shortage shortage_ = Shortage::NONE;
  std::map<Slot, std::string> occupied_;  // 跨任务保留，最新观测可更新实际占用。
  std::map<Slot, tf2::Vector3> occupied_extent_;  // 预留尺寸：漏检时仍能阻止重叠目标。
  std::map<std::string, Attempt> attempts_;
  Object active_object_;
  Slot active_slot_{};
  tf2::Vector3 tool_to_tcp_;
  std::vector<Class> default_order_, order_;
  std::vector<double> workspace_min_, workspace_max_;
  std::string frame_, tool_frame_, default_strategy_, strategy_, observation_target_;
  std::string finish_mode_, retry_policy_, exhausted_policy_, slots_full_policy_, verification_, last_failed_;
  rclcpp::Time last_received_{0, 0, RCL_ROS_TIME}, observations_after_{0, 0, RCL_ROS_TIME};
  rclcpp::Time idle_since_{0, 0, RCL_ROS_TIME}, verification_since_{0, 0, RCL_ROS_TIME};
  Clock::time_point action_since_, next_try_, recovery_since_;
  double max_age_, timeout_, action_timeout_, recovery_timeout_, settle_time_, idle_timeout_;
  double cooldown_, verify_timeout_, slot_tolerance_, height_tolerance_, slot_clearance_;
  double candidate_max_age_;
  double candidate_max_drift_;
  int max_attempts_, max_candidate_attempts_, max_candidate_reevaluations_, max_objects_;
  int64_t recovery_request_id_ = 0;
  uint64_t sequence_ = 0, processed_sequence_ = 0;
  uint32_t completed_ = 0;
  bool busy_ = false, needs_observation_ = false, cancel_sent_ = false, verifying_ = false, idle_ = false;
  bool recovery_pending_ = false, recovery_holding_ = false;
  Phase phase_ = Phase::WAITING;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<TaskPlanner>());}
  catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("task_planner"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown();
  return 0;
}
