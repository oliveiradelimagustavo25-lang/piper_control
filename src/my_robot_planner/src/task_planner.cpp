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
  using Trigger = std_srvs::srv::Trigger;
  using SortHandle = rclcpp_action::ServerGoalHandle<Sort>;
  using PickHandle = rclcpp_action::ClientGoalHandle<Pick>;
  using NamedHandle = rclcpp_action::ClientGoalHandle<Named>;
  using Clock = std::chrono::steady_clock;
  using Class = uint16_t;
  using Slot = std::pair<Class, size_t>;
  enum class Phase {WAITING, OBSERVING, SENDING, EXECUTING, RECOVERING};
  enum class RuleState {READY, RESERVED, PLACED, UNREACHABLE};
  struct RuleTarget {Class class_id; tf2::Vector3 goal, extent; RuleState state = RuleState::READY;};
  struct Attempt {int count = 0; rclcpp::Time retry_after{0, 0, RCL_ROS_TIME};};
  struct Verification {Object source; tf2::Vector3 target; Slot slot;};
  struct PlacementCheck {const Object * destination = nullptr; bool at_source = false, reliable = false;};

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
    max_objects_ = declare_parameter<int>("max_objects", 0);
    idle_timeout_ = declare_parameter<double>("idle_timeout", 3.0);
    cooldown_ = declare_parameter<double>("retry_cooldown", 0.0);
    verify_timeout_ = declare_parameter<double>("verification_timeout", 3.0);
    slot_tolerance_ = declare_parameter<double>("slot_tolerance", 0.012);
    height_tolerance_ = declare_parameter<double>("height_tolerance", 0.006);
    slot_clearance_ = declare_parameter<double>("slot_clearance", 0.025);
    max_age_ = declare_parameter<double>("max_observation_age", 2.0);
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
    use_position_rules_ = declare_parameter<bool>("use_position_rules", false);
    // Discover explicit rules even when every referenced ID is unknown. Never
    // fall back to region coordinates after an explicit rule set was supplied.
    for (const auto & entry : get_node_parameters_interface()->get_parameter_overrides()) {
      const auto & name = entry.first;
      if (name.rfind("place_positions.", 0) == 0) {use_position_rules_ = true;}
      if (name.rfind("place_positions.", 0) == 0 || name.rfind("classes.", 0) == 0) {
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
    for (const auto & entry : classes_) {
      const auto id = entry.first;
      const auto key = std::to_string(id);
      try {
        const auto rule = declare_parameter<std::vector<double>>("place_positions." + key, std::vector<double>{});
        if (use_position_rules_) {
          if (!rule.empty()) {position_values_[id] = xyz(rule);}
          continue;
        }
        const auto prefix = "classes." + key;
        const auto origin_values = declare_parameter<std::vector<double>>(prefix + ".place_position", std::vector<double>{});
        auto values = declare_parameter<std::vector<double>>(prefix + ".place_slots", std::vector<double>{});
        const auto region = declare_parameter<std::vector<double>>(prefix + ".place_region", std::vector<double>{});
        if (values.empty() && !region.empty()) {
          if (region.size() != 5 || !std::all_of(region.begin(), region.end(), [](double v) {return std::isfinite(v);}) ||
            region[0] > region[2] || region[1] > region[3]) {throw std::invalid_argument("invalid region");}
          const auto origin = origin_values.empty() ? tf2::Vector3(region[0], region[1], region[4]) : xyz(origin_values);
          std::vector<tf2::Vector3> points;
          const int xmin = static_cast<int>(std::ceil((region[0] - origin.x()) / step[0] - 1e-9));
          const int xmax = static_cast<int>(std::floor((region[2] - origin.x()) / step[0] + 1e-9));
          const int ymin = static_cast<int>(std::ceil((region[1] - origin.y()) / step[1] - 1e-9));
          const int ymax = static_cast<int>(std::floor((region[3] - origin.y()) / step[1] + 1e-9));
          for (int y = ymin; y <= ymax; ++y) {
            for (int x = xmin; x <= xmax; ++x) {points.emplace_back(origin.x() + x * step[0], origin.y() + y * step[1], region[4]);}
          }
          std::stable_sort(points.begin(), points.end(), [&](const auto & a, const auto & b) {return (a - origin).length2() < (b - origin).length2();});
          for (const auto & point : points) {values.insert(values.end(), {point.x(), point.y(), point.z()});}
        }
        if (values.empty()) {values = origin_values;}
        if (values.size() % 3 != 0) {throw std::invalid_argument("slots require xyz triples");}
        for (size_t i = 0; i < values.size(); i += 3) {slots_[id].push_back(xyz({values[i], values[i + 1], values[i + 2]}));}
        if (!values.empty()) {set_parameter(rclcpp::Parameter(prefix + ".place_slots", values));}
        if (slots_[id].empty()) {slots_.erase(id);}
      } catch (const std::exception & error) {
        slots_.erase(id); position_values_.erase(id);
        RCLCPP_WARN(get_logger(), "Class %u placement disabled: %s", id, error.what());
      }
    }
    if (!validPolicy(default_strategy_, default_order_) || frame_.empty() || tool_frame_.empty() ||
      (finish_mode_ != "batch" && finish_mode_ != "watch") ||
      (retry_policy_ != "priority" && retry_policy_ != "immediate" && retry_policy_ != "disabled") ||
      (exhausted_policy_ != "continue" && exhausted_policy_ != "stop") ||
      (slots_full_policy_ != "wait" && slots_full_policy_ != "finish") ||
      (verification_ != "visual" && verification_ != "motion") || max_attempts_ < 1 || max_objects_ < 0)
    {throw std::invalid_argument("Invalid task policy");}
    for (const double value : {max_age_, timeout_, action_timeout_, recovery_timeout_,
      verify_timeout_, slot_tolerance_, height_tolerance_})
    {
      if (!std::isfinite(value) || value <= 0) {throw std::invalid_argument("Invalid timeout/tolerance");}
    }
    for (const double value : {idle_timeout_, cooldown_, settle_time_, slot_clearance_}) {
      if (!std::isfinite(value) || value < 0) {throw std::invalid_argument("Invalid wait/clearance");}
    }
    subscription_ = create_subscription<Objects>(
      declare_parameter<std::string>("objects_topic", "/perception/objects"), rclcpp::SensorDataQoS(),
      [this](Objects::ConstSharedPtr objects) {observations_ = objects; last_received_ = now(); ++sequence_;});
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
        unreachable_slots_.clear();
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
  {return use_position_rules_ ? position_values_.count(id) : slots_.count(id);}
  void skip(const Object & object, const std::string & reason)
  {
    if (skipped_.emplace(object.object_id, reason).second) {
      RCLCPP_WARN(get_logger(), "Skipping %s class=%u: %s", object.object_id.c_str(), object.class_id, reason.c_str());
    }
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
  void markPlaced(const std::string & source_id, Slot slot, const std::string & observed_id)
  {
    if (use_position_rules_) {
      auto target = rule_targets_.at(source_id);
      target->state = RuleState::PLACED;
      if (!observed_id.empty()) {rule_targets_[observed_id] = target;}
    } else {occupied_[slot] = observed_id;}
  }
  void confirmPlacement(const Object & source, Slot slot, const std::string & observed_id)
  {
    markPlaced(source.object_id, slot, observed_id);
    attempts_.erase(source.object_id);
    ++completed_;
    feedback(source.object_id);
    RCLCPP_INFO(get_logger(), "Placement verified: %s", source.object_id.c_str());
  }
  void releaseReservation(const std::string & id, Slot slot)
  {
    if (use_position_rules_) {rule_targets_.at(id)->state = RuleState::READY;}
    else {occupied_.erase(slot);}
  }
  bool sorted(const Object & o) const
  {
    if (!hasPlacement(o.class_id)) {return false;}
    if (use_position_rules_) {
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
          if (use_position_rules_) {rule_targets_.at(active_object_.object_id)->state = RuleState::RESERVED;}
          else {occupied_[active_slot_] = "";}
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
  int freeSlot(const Object & object, const std::vector<Object> & objects) const
  {
    if (use_position_rules_) {
      const auto own = rule_targets_.find(object.object_id);
      if (own != rule_targets_.end() && own->second->state == RuleState::UNREACHABLE) {return -1;}
      const auto goal = ruleGoal(object);
      if (blocked(object, goal, objects)) {return -1;}
      for (const auto & entry : rule_targets_) {
        const auto & target = *entry.second;
        if (own != rule_targets_.end() && own->second == entry.second) {continue;}
        if ((target.state == RuleState::RESERVED || target.state == RuleState::PLACED) &&
          std::abs(goal.x() - target.goal.x()) < extent(object).x() + target.extent.x() + slot_clearance_ &&
          std::abs(goal.y() - target.goal.y()) < extent(object).y() + target.extent.y() + slot_clearance_ &&
          std::abs(goal.z() - target.goal.z()) < extent(object).z() + target.extent.z() + slot_clearance_) {return -1;}
      }
      return 0;
    }
    for (size_t i = 0; i < slots_.at(object.class_id).size(); ++i) {
      if (occupied_.count({object.class_id, i}) || unreachable_slots_.count({object.class_id, i})) {continue;}
      if (!blocked(object, slots_.at(object.class_id)[i], objects)) {return static_cast<int>(i);}
    }
    return -1;
  }
  void updateOccupancy(const std::vector<Object> & objects)
  {
    if (use_position_rules_) {
      for (const auto & o : objects) {
        if (o.tracking_uncertain || !hasPlacement(o.class_id)) {continue;}
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
      return;
    }
    // 只有已关联物体明确移开才释放缓存；漏检/离开视野不等于槽位已空。
    for (auto it = occupied_.begin(); it != occupied_.end();) {
      const auto moved = std::find_if(objects.begin(), objects.end(), [&](const auto & o) {
        return !o.tracking_uncertain && o.object_id == it->second &&
               !near(position(o), slots_.at(it->first.first)[it->first.second]);
      });
      if (moved != objects.end()) {it = occupied_.erase(it);} else {++it;}
    }
    for (const auto & o : objects) {
      if (o.tracking_uncertain || !hasPlacement(o.class_id)) {continue;}
      for (size_t i = 0; i < slots_.at(o.class_id).size(); ++i) {
        if (near(position(o), slots_.at(o.class_id)[i])) {
          occupied_[{o.class_id, i}] = o.object_id;
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
    if (use_position_rules_) {
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
    feedback(object.object_id);
    RCLCPP_INFO(get_logger(), "Pick %s class=%u (%.4f, %.4f, %.4f) -> slot %zu (%.4f, %.4f, %.4f)",
      object.object_id.c_str(), object.class_id, object.pose.position.x, object.pose.position.y, object.pose.position.z,
      slot, p.x(), p.y(), p.z());
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
      if (!r.ready_for_next_pick || r.holding) {startRecovery(r.holding); return;}
      if (r.unreachable_placement) {
        if (use_position_rules_) {rule_targets_.at(active_object_.object_id)->state = RuleState::UNREACHABLE;}
        else {unreachable_slots_.insert(active_slot_);}
        auto & count = attempts_[active_object_.object_id].count;
        if (--count <= 0) {attempts_.erase(active_object_.object_id);}
        RCLCPP_WARN(get_logger(), "Placement candidate unreachable; selecting another slot/target");
      } else if (r.released) {
        if (use_position_rules_) {rule_targets_.at(active_object_.object_id)->state = RuleState::RESERVED;}
        else {occupied_[active_slot_] = "";}
        if (verification_ == "motion") {
          markPlaced(active_object_.object_id, active_slot_, "");
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
        if (freeSlot(o, objects) < 0) {no_slots = true; continue;}
        candidates.push_back(o);
      }
      if (!candidates.empty()) {
        idle_ = false;
        if (!pick_client_->action_server_is_ready()) {waiting("Waiting for PickPlace server"); return;}
        const auto object = selectTarget(candidates);
        dispatch(object, static_cast<size_t>(freeSlot(object, objects)));
      } else if (deferred || (no_slots && slots_full_policy_ == "wait")) {
        waiting(no_slots ? "No free placement slot; waiting for space" : "Waiting for association/retry eligibility");
      } else if (finish_mode_ == "batch" && idle_timeout_ > 0) {
        if (!idle_) {idle_since_ = now(); idle_ = true;}
        if ((now() - idle_since_).seconds() >= idle_timeout_) {
          finish(attempts_.empty() && !no_slots && skipped_.empty() && observations_->issues.empty() && !classes_.empty(), no_slots ? "Placement area full" : "No remaining actionable targets");
        }
      }
    } catch (const std::exception & error) {waiting(error.what());}
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<Objects>::SharedPtr subscription_;
  rclcpp_action::Client<Pick>::SharedPtr pick_client_;
  rclcpp_action::Client<Named>::SharedPtr named_client_;
  rclcpp::Client<Trigger>::SharedPtr recovery_client_;
  rclcpp_action::Server<Sort>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  Objects::ConstSharedPtr observations_;
  std::map<std::string, Verification> pending_verifications_;
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
  bool use_position_rules_ = false;
  std::set<Slot> unreachable_slots_;  // 仅本任务，抓前 IK 不可达的候选位置。
  std::map<Slot, std::string> occupied_;  // 跨任务保留，最新观测可更新实际占用。
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
  int max_attempts_, max_objects_;
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
