# CARKit Planning

Planning converts localized perception and route information into driving
intent. It is deliberately separated from control, which only arbitrates and
executes commands.

- `carkit_behavior`: C++ rule-level planning for stop signs, traffic lights,
  cones, speed signs, and future road behaviors.
- `carkit_navigation` currently remains under `carkit/navigation`; it owns
  global/local path generation and supplies `/plan` and `/drive`.

The intended data flow is:

```text
perception + localization + route
  -> behavior rules
  -> prioritized behavior decision
  -> control-center safety arbiter
  -> chassis
```

The ROS package name and `/behavior/*` interface remain stable even though the
package now lives in the planning source tree.
