#ifndef LASER_UAV_MANAGERS__REFERENCE_HORIZON_SELECTOR_HPP_
#define LASER_UAV_MANAGERS__REFERENCE_HORIZON_SELECTOR_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <laser_msgs/msg/reference_state.hpp>
#include <laser_uav_multi_drones/msg/reference_horizon.hpp>

namespace laser_uav_managers
{

class ReferenceHorizonSelector
{
public:
  enum class Source {Planner, External, Hold};
  struct FlightState
  {
    bool takeoff_done;
    bool is_flying;
    bool taking_off;
    bool landing;
    bool emergency_hover;
  };
  struct Selection
  {
    Source source;
    std::vector<laser_msgs::msg::ReferenceState> references;
  };

  explicit ReferenceHorizonSelector(std::size_t steps) : steps_(steps) {}

  Selection select(bool external_enabled, const FlightState & flight,
    const laser_uav_multi_drones::msg::ReferenceHorizon::ConstSharedPtr & fresh_horizon,
    const laser_msgs::msg::ReferenceState & hover_candidate, const std::string & frame)
  {
    if (!external_enabled || !flight.takeoff_done || !flight.is_flying ||
      flight.taking_off || flight.landing || flight.emergency_hover)
    {
      reset();
      return {Source::Planner, {}};
    }
    if (fresh_horizon) {
      reset();
      return {Source::External, fresh_horizon->references};
    }
    // Capture the measured pose once, so missing references do not cause drift
    // or resume a previous planner mission. A frame change requires a new pose.
    if (!holding_ || frame != hold_frame_) {
      hold_ = hover_candidate;
      hold_frame_ = frame;
      holding_ = true;
    }
    return {Source::Hold, std::vector<laser_msgs::msg::ReferenceState>(steps_ + 1, hold_)};
  }

  void reset()
  {
    holding_ = false;
    hold_frame_.clear();
  }

private:
  const std::size_t steps_;
  bool holding_{false};
  std::string hold_frame_;
  laser_msgs::msg::ReferenceState hold_;
};

}  // namespace laser_uav_managers

#endif  // LASER_UAV_MANAGERS__REFERENCE_HORIZON_SELECTOR_HPP_
