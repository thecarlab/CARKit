# CARKit WebUI design specification

Status: implementation baseline for `ada26`

## Product intent

The interface is a classroom vehicle operations console. It should feel calm,
precise, and trustworthy while a physical car is powered—not like a generic
admin dashboard. A student should be able to answer three questions in under
three seconds:

1. Is CARKit connected and safe?
2. What is running, and which course/chassis configuration is active?
3. What is the vehicle currently seeing and commanding?

Configuration is important before a session, but live spatial information is
more important during a session. The UI therefore separates configuration
from operation instead of placing every form field above the visualization.

## Information architecture

```text
┌──────────────┬──────────────────────────────────────────────────────────┐
│ Brand rail   │ Top bar: session identity · connection · Configure/Stop │
│              ├──────────────────────────────────────────────────────────┤
│ Overview     │ Health strip: mode · speed · steering · lidar · camera  │
│ Code editor  │ ADA Python and Intro2AV Python/C++ workspaces            │
│ Compile      │ Shared build targets and compiler output                │
│ Live view    ├───────────────────────────────────┬──────────────────────┤
│ System log   │                                   │ Camera               │
│              │ Map / lidar / path                ├──────────────────────┤
│ Active setup │ (primary operational surface)     │ Runtime health       │
│              │                                   │ and current setup    │
│ CAR Lab      ├───────────────────────────────────┴──────────────────────┤
│ ADA 2026     │ Collapsible system console                               │
└──────────────┴──────────────────────────────────────────────────────────┘
                                                  ┌───────────────────────┐
                                                  │ Configuration drawer  │
                                                  │ course + chassis      │
                                                  │ algorithms + modules  │
                                                  │ map + install/start   │
                                                  └───────────────────────┘
```

## Interaction model

- **Configure** opens a right-side drawer. Closing it never discards the form.
- **Install chassis** is visually secondary and reports progress in the drawer.
- **Launch session** is the primary configuration action. When running, the
  top bar presents a clear **Stop stack** action.
- Initial-pose and goal tools are one-shot. The tool must be armed before a map
  click can publish anything, and disarms immediately after publishing.
- The system console is collapsed by default and expandable without leaving
  the live view.
- The code editor exposes only course-owned perception, planning, and control
  package roots. Its explorer includes source, headers, tests, configuration,
  manifests, build files, and documentation; reference implementations are
  never returned by its API. The module selector chooses the initial file but
  does not restrict navigation to it.
- Course files are live shared documents. Concurrent edits are rebased onto
  the authoritative version, written atomically, and shown with presence and
  colored remote cursors; every explorer file maintains a separate session.
- The dependency-free editor colors Python and C++ locally. Python AST errors
  and C++ delimiter/string errors mark the affected line before compilation.
- Status is communicated with text and shape as well as color.
- There is no decorative animation that competes with live vehicle motion.

## Visual system

The palette is derived from the CARKit mark:

| Token | Value | Use |
| --- | --- | --- |
| Ink | `#10233f` | Primary text, navigation, control surfaces |
| CAR blue | `#0b5ca8` | Brand, active navigation, primary actions |
| Signal yellow | `#f4b41a` | Vehicle marker, attention, selected tools |
| Canvas | `#f3f6f9` | Application background |
| Surface | `#ffffff` | Cards and drawers |
| Border | `#dce4ec` | Quiet structure |
| Success | `#16845b` | Connected/running state |
| Danger | `#c63f3f` | Stop action and faults |

Typography uses the local system sans-serif stack and tabular numerals for
telemetry. Headings are compact rather than oversized. Shadows are restrained;
hierarchy comes mainly from spacing, borders, and surface contrast.

## Component rules

- Sidebar: 224 px desktop width, icon-only compact state below 1100 px, hidden
  in favor of the top bar below 760 px.
- Content: maximum useful width is unconstrained so large classroom displays
  give the map more space.
- Map: minimum 620 px desktop height where viewport permits; camera retains a
  4:3 aspect ratio.
- Telemetry: five equal metric cells, values use tabular numerals.
- Taskbar monitor: chassis freshness and battery voltage from ROS, plus shared
  Jetson CPU, memory, load, temperature, and uptime sampled by the WebUI server.
- Status chips: `Connected`, `Starting`, `Running`, `Stopped`, or `Fault` with
  explicit labels.
- Inputs: 44 px minimum height, visible focus rings, labels above controls.
- Buttons: sentence case; primary blue, stop red outline, tool selection yellow.

## Responsive behavior

- Above 1180 px: sidebar + map/right-column dashboard.
- 760–1179 px: compact sidebar; right column flows below map.
- Below 760 px: no sidebar, stacked cards, horizontally scrollable telemetry,
  full-width configuration drawer, and map tools wrap onto two rows.

## Acceptance criteria

- Live map is visible above the fold on a 1440×900 display.
- Configuration does not consume dashboard space while closed.
- Running/stopped state is visible without opening the drawer.
- All existing element IDs and backend API contracts remain supported.
- Browser rendering remains dependency-free and works without internet access.
- Five simultaneous users sustain the 10 Hz perception view; live ROSBridge
  subscriptions and image decoding retain only the newest pending frame.
- Five simultaneous editor clients can edit the same file without locking;
  server-side operation history preserves concurrent non-overlapping changes.
- Map, camera, telemetry, setup, logs, initial pose, and goal publication remain
  functional after the visual rewrite.
