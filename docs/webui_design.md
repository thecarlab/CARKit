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
│ Code editor  │ Reference viewer + ADA/Intro2AV editable workspaces      │
│ Compile      │ Shared build targets and compiler output                │
│ Terminal     │ Shared interactive shells inside the CARKit container   │
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
- The code editor exposes reference, ADA, and Intro2AV perception, planning,
  and control implementations. Its explorer is scoped by both implementation
  and selected component. Reference files are browseable but read-only in the
  browser, and both legacy save and collaborative sync endpoints enforce the
  same boundary. ADA and Intro2AV files remain editable.
- Course files are live shared documents. Concurrent edits are rebased onto
  the authoritative version, written atomically, and shown with presence and
  colored remote cursors; every explorer file maintains a separate session.
- The dependency-free editor colors Python and C++ locally. Python AST errors
  and C++ delimiter/string errors mark the affected line before compilation.
- The terminal page lists server-owned PTY sessions. Opening a listed terminal
  joins the same shell and scrollback, so students at different browsers on
  the same machine can observe and collaborate in one session.
- Terminal shells inherit the container's ROS environment and begin in
  `/workspaces/CARKit`. Closing a terminal ends it for every attached browser;
  stopping the vehicle stack does not close terminal sessions.
- Status is communicated with text and shape as well as color.
- There is no decorative animation that competes with live vehicle motion.

## Visual system

The palette is derived from the CARKit mark:

The top-left brand lockup uses the official CAR Lab icon from
`https://www.thecarlab.org/`, embedded locally so the classroom dashboard does
not depend on internet access.

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
- Taskbar monitor: chassis freshness and battery voltage from ROS, plus Jetson
  CPU and RAM, CARKit container CPU and RAM, temperature, and uptime sampled by
  the WebUI server. CARKit usage is measured from the Docker cgroup.
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
- Five simultaneous editor clients can edit the same ADA or Intro2AV file
  without locking; server-side operation history preserves concurrent
  non-overlapping changes. Reference files remain selectable and copyable but
  cannot be modified through either editor API.
- Terminal sessions are visible to every dashboard client and remain attached
  to the WebUI server rather than to a browser tab. The server caps concurrent
  shells and buffered output to prevent unbounded resource use. Users type and
  paste directly into the focused terminal viewport; no separate command field
  is used. A cursor tracks the emulated PTY position, and Tab is forwarded to
  Bash for native command and path completion.
- Map, camera, telemetry, setup, logs, initial pose, and goal publication remain
  functional after the visual rewrite.

## Terminal security boundary

The Terminal tab intentionally provides command execution inside the CARKit
container. The dashboard has no user authentication, so port 8080 must remain
on the trusted classroom LAN. Do not expose it to the public internet. The
implementation allocates PTYs directly in the already-running WebUI container;
it does not mount or expose the Docker daemon socket.
