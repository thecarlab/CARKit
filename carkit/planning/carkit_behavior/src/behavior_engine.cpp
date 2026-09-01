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

#include "carkit_behavior/behavior_engine.hpp"

#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

// CARKit learning annotation: implements the behavior described by this file's package and module.
namespace carkit_behavior
{

BehaviorDecision BehaviorDecision::normal()
{
  return {};
}

BehaviorDecision BehaviorDecision::stop(
  const std::string & state, const std::string & rule, const std::string & reason)
{
  return {state, rule, std::nullopt, reason};
}

BehaviorDecision BehaviorDecision::speed(
  const std::string & state, const std::string & rule, double speed_mps,
  const std::string & reason)
{
  return {state, rule, speed_mps, reason};
}

namespace
{

class StopSignRule : public BehaviorRule
{
public:
  std::string name() const override {return "stop_sign";}
  int priority() const override {return 400;}

  std::optional<BehaviorDecision> evaluate(
    const BehaviorContext & context, double now_sec) override
  {
    if (now_sec < stop_until_) {
      return BehaviorDecision::stop(
        kStopSign, name(), "completing mandatory stop duration");
    }
    if (!context.stop_sign_triggered()) {
      return std::nullopt;
    }
    stop_until_ = now_sec + context.stop_sign_stop_duration_sec;
    return BehaviorDecision::stop(kStopSign, name(), "stop line reached");
  }

private:
  double stop_until_{0.0};
};

class TrafficLightRule : public BehaviorRule
{
public:
  std::string name() const override {return "traffic_light";}
  int priority() const override {return 300;}
  std::optional<BehaviorDecision> evaluate(
    const BehaviorContext & context, double) override
  {
    if (!context.traffic_light_stop_active()) {
      return std::nullopt;
    }
    return BehaviorDecision::stop(
      kTrafficLight, name(), "confirmed stop signal ahead");
  }
};

class ConeRule : public BehaviorRule
{
public:
  std::string name() const override {return "cone";}
  int priority() const override {return 200;}
  std::optional<BehaviorDecision> evaluate(
    const BehaviorContext & context, double) override
  {
    if (!context.cone_speed_override_active()) {
      return std::nullopt;
    }
    return BehaviorDecision::speed(
      kCone, name(), context.cone_override_speed_mps,
      "stable cone track requires reduced speed");
  }
};

class SpeedSignRule : public BehaviorRule
{
public:
  std::string name() const override {return "speed_sign";}
  int priority() const override {return 100;}
  std::optional<BehaviorDecision> evaluate(
    const BehaviorContext & context, double now_sec) override
  {
    if (now_sec >= override_until_) {
      if (!context.speed_sign_pass_triggered()) {
        return std::nullopt;
      }
      override_until_ = now_sec + context.speed_sign_override_duration_sec;
    }
    active_speed_ = context.speed_sign_override_speed();
    if (!active_speed_) {
      return std::nullopt;
    }
    return BehaviorDecision::speed(
      kSpeedSign, name(), *active_speed_, "speed sign passed");
  }

private:
  double override_until_{0.0};
  std::optional<double> active_speed_;
};

using RuleFactory = std::unique_ptr<BehaviorRule>(*)();

template<typename RuleT>
std::unique_ptr<BehaviorRule> make_rule()
{
  return std::make_unique<RuleT>();
}

const std::unordered_map<std::string, RuleFactory> kFactories{
  {"stop_sign", &make_rule<StopSignRule>},
  {"traffic_light", &make_rule<TrafficLightRule>},
  {"cone", &make_rule<ConeRule>},
  {"speed_sign", &make_rule<SpeedSignRule>},
};


}  // namespace

void BehaviorEngine::register_rule(std::unique_ptr<BehaviorRule> rule)
{
  if (!rule || rule->name().empty()) {
    throw std::invalid_argument("behavior rule name cannot be empty");
  }
  if (std::any_of(
      rules_.begin(), rules_.end(), [&rule](const auto & existing) {
        return existing->name() == rule->name();
      }))
  {
    throw std::invalid_argument("behavior rule already registered: " + rule->name());
  }
  rules_.push_back(std::move(rule));
  std::sort(
    rules_.begin(), rules_.end(), [](const auto & left, const auto & right) {
      if (left->priority() != right->priority()) {
        return left->priority() > right->priority();
      }
      return left->name() < right->name();
    });
}

/// Return the first decision claimed by the priority-sorted rule set.
/// A rule that returns nullopt explicitly delegates to the next-lower priority.
BehaviorDecision BehaviorEngine::evaluate(const BehaviorContext & context, double now_sec)
{
  for (auto & rule : rules_) {
    auto decision = rule->evaluate(context, now_sec);
    if (decision) {
      return *decision;
    }
  }
  return BehaviorDecision::normal();
}

std::vector<std::string> BehaviorEngine::rule_names() const
{
  std::vector<std::string> names;
  names.reserve(rules_.size());
  for (const auto & rule : rules_) {
    names.push_back(rule->name());
  }
  return names;
}

std::vector<std::unique_ptr<BehaviorRule>> build_behavior_rules(
  const std::vector<std::string> & names)
{
  std::vector<std::unique_ptr<BehaviorRule>> output;
  std::unordered_set<std::string> seen;
  output.reserve(names.size());
  for (const auto & name : names) {
    const auto factory = kFactories.find(name);
    if (factory == kFactories.end()) {
      throw std::invalid_argument("unknown behavior rule: " + name);
    }
    if (!seen.insert(name).second) {
      throw std::invalid_argument("behavior rule names must be unique");
    }
    output.push_back(factory->second());
  }
  return output;
}

}  // namespace carkit_behavior
