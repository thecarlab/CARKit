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

#include <memory>
#include <optional>
#include <string>

#include "carkit_behavior/behavior_engine.hpp"
#include "gtest/gtest.h"

namespace cb = carkit_behavior;

namespace
{
class SchoolZoneRule : public cb::BehaviorRule
{
public:
  std::string name() const override {return "school_zone";}
  int priority() const override {return 250;}
  std::optional<cb::BehaviorDecision> evaluate(const cb::BehaviorContext &, double) override
  {
    return cb::BehaviorDecision::speed("SCHOOL_ZONE", name(), 0.5, "school zone active");
  }
};
}  // namespace

TEST(BehaviorEngine, UsesPriorityRatherThanRegistrationOrder)
{
  cb::BehaviorEngine engine;
  auto rules = cb::build_behavior_rules({"cone", "stop_sign"});
  for (auto & rule : rules) {
    engine.register_rule(std::move(rule));
  }
  cb::BehaviorContext context;
  context.stop_sign_triggered = []() {return true;};
  context.cone_speed_override_active = []() {return true;};

  const auto decision = engine.evaluate(context, 2.0);
  EXPECT_EQ(engine.rule_names(), (std::vector<std::string>{"stop_sign", "cone"}));
  EXPECT_EQ(decision.state, cb::kStopSign);
  EXPECT_TRUE(decision.stops_vehicle());
  EXPECT_EQ(engine.evaluate(context, 3.0).state, cb::kStopSign);
}

TEST(BehaviorEngine, ReturnsNormalWhenNoRuleClaimsContext)
{
  cb::BehaviorEngine engine;
  auto rules = cb::build_behavior_rules({"cone"});
  engine.register_rule(std::move(rules.front()));
  const auto decision = engine.evaluate({}, 2.0);
  EXPECT_EQ(decision.state, cb::kNormalNav2);
  EXPECT_FALSE(decision.override_active());
}

TEST(BehaviorEngine, AcceptsNewRuleWithoutEngineChanges)
{
  cb::BehaviorEngine engine;
  engine.register_rule(std::make_unique<SchoolZoneRule>());
  const auto decision = engine.evaluate({}, 2.0);
  EXPECT_EQ(decision.rule, "school_zone");
  ASSERT_TRUE(decision.target_speed_mps);
  EXPECT_DOUBLE_EQ(*decision.target_speed_mps, 0.5);
}

TEST(BehaviorEngine, RejectsUnknownAndDuplicateRules)
{
  EXPECT_THROW(cb::build_behavior_rules({"not_a_rule"}), std::invalid_argument);
  EXPECT_THROW(cb::build_behavior_rules({"cone", "cone"}), std::invalid_argument);
}
