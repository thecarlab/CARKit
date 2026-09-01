# CARKit Behavior Planner

`carkit_behavior` is the rule-level planning layer between perception/Nav2 and
the control-center safety arbiter. It does not directly own the chassis.

## Architecture

```text
ROS callbacks -> tracked world context -> BehaviorEngine -> BehaviorDecision
                                                       -> /behavior/*
```

The C++ node owns ROS I/O and the tracked world context. `BehaviorEngine` owns
deterministic priority arbitration. Each road behavior implements one small
rule derived from `BehaviorRule`:

```cpp
class SchoolZoneRule : public BehaviorRule {
public:
  std::string name() const override {return "school_zone";}
  int priority() const override {return 250;}
  std::optional<BehaviorDecision> evaluate(
    const BehaviorContext & context, double now_sec) override;
};
```

Register it with `BehaviorEngine::register_rule()`. Built-in rules are created
by `build_behavior_rules()`; deployments select them with the `behavior_rules`
parameter in `config/behavior_center.yaml`. Unknown or duplicate names fail at
startup instead of silently disabling safety logic.

Current priority is:

1. Stop sign
2. Traffic light
3. Cone speed limit
4. Speed-sign override
5. Normal Nav2 command

Behavior input subscriptions are activated only in `AUTO_DRIVE`. The control
center remains the final safety authority and consumes:

- `/behavior/state`
- `/behavior/override_active`
- `/behavior/override_cmd`

Launch with:

```bash
ros2 launch carkit_behavior behavior_center.launch.py
```
