# CARKit Web Bridge

`carkit_web_bridge` is the native C++ browser bridge used by the CARKit WebUI.
It deliberately implements only the CARKit browser protocol instead of exposing
the complete rosbridge API.

The bridge:

- sends ROS messages as binary CBOR, including JPEG data as a CBOR byte string;
- shares each serialized message across all connected clients;
- applies each client's requested `throttle_rate`;
- retains only the newest queued message per topic for a slow client; and
- accepts at most five simultaneous WebSocket clients by default.

Supported browser publications are `/enable_autonomous_control`, `/initialpose`,
and `/goal_pose`. The allowed subscription topics are defined in
`src/web_bridge_node.cpp`. This fixed allowlist is also a safety boundary: the
browser cannot publish arbitrary commands to the ROS graph.
