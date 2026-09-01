// Copyright 2026 University of Delaware
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CARKIT_BEHAVIOR__BEHAVIOR_ENGINE_HPP_
#define CARKIT_BEHAVIOR__BEHAVIOR_ENGINE_HPP_

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// CARKit learning annotation: declares interfaces implemented by the corresponding source.
namespace carkit_behavior
{

inline constexpr char kNormalNav2[] = "NORMAL_NAV2";
inline constexpr char kStopSign[] = "STOP_SIGN";
inline constexpr char kTrafficLight[] = "TRAFFIC_LIGHT";
inline constexpr char kSpeedSign[] = "SPEED_SIGN";
inline constexpr char kCone[] = "CONE";

struct BehaviorDecision
{
  std::string state{kNormalNav2};
  std::string rule{"normal_navigation"};
  std::optional<double> target_speed_mps;
  std::string reason;

  bool override_active() const {return state != kNormalNav2;}
  bool stops_vehicle() const {return override_active() && !target_speed_mps.has_value();}

  static BehaviorDecision normal();
  static BehaviorDecision stop(
    const std::string & state, const std::string & rule, const std::string & reason);
  static BehaviorDecision speed(
    const std::string & state, const std::string & rule, double speed_mps,
    const std::string & reason);
};

// A read-only snapshot assembled by the ROS adapter once per planning cycle.
// New rules may use existing fields or add new observations without changing
// BehaviorEngine itself.
struct BehaviorContext
{
  std::function<bool()> stop_sign_triggered{[]() {return false;}};
  double stop_sign_stop_duration_sec{5.0};
  std::function<bool()> traffic_light_stop_active{[]() {return false;}};
  std::function<bool()> cone_speed_override_active{[]() {return false;}};
  double cone_override_speed_mps{0.8};
  std::function<bool()> speed_sign_pass_triggered{[]() {return false;}};
  std::function<std::optional<double>()> speed_sign_override_speed{
    []() {return std::nullopt;}};
  double speed_sign_override_duration_sec{3.0};
};

class BehaviorRule
{
public:
  virtual ~BehaviorRule() = default;
  virtual std::string name() const = 0;
  virtual int priority() const = 0;
  virtual std::optional<BehaviorDecision> evaluate(
    const BehaviorContext & context, double now_sec) = 0;
};

class BehaviorEngine
{
public:
  void register_rule(std::unique_ptr<BehaviorRule> rule);
  BehaviorDecision evaluate(const BehaviorContext & context, double now_sec);
  std::vector<std::string> rule_names() const;

private:
  std::vector<std::unique_ptr<BehaviorRule>> rules_;
};

std::vector<std::unique_ptr<BehaviorRule>> build_behavior_rules(
  const std::vector<std::string> & names);

}  // namespace carkit_behavior

#endif  // CARKIT_BEHAVIOR__BEHAVIOR_ENGINE_HPP_
