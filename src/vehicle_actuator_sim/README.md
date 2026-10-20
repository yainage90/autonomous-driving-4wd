# Watchdog wheel actuator

This Gazebo Sim 8 System replaces DiffDrive's unbounded Twist input with the
receiver-owned actuator protocol. See [the protocol and validation guide](../../docs/actuator_watchdog.md).

`src/WatchdogDiffDrive.cc` and its header are derived from Gazebo Sim **8.15.0**,
tag `gz-sim8_8.15.0`, commit `446a44335a45b704b4d36dabcc5508ee34eeb3d8`,
under Apache-2.0. Original copyright notices are retained.

Upstream files:

- https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.15.0/src/systems/diff_drive/DiffDrive.cc
- https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.15.0/src/systems/diff_drive/DiffDrive.hh

SHA-256 of the original downloaded source:

```text
DiffDrive.cc dae83e0a666a3ac111dd9a98c0aac4827d985dde1ba1c1b08d6d21390783e43a
DiffDrive.hh ff60a2789eabc53d918406489a8a396321adfaa4de952631b0786997a79f42d0
```

Changes: replace command/enable subscriptions with custom ROS interfaces, own the
ROS context/executor, check expiry and pause before wheel output in PreUpdate,
clear limiter history on disarm, and retain wheel odometry/TF publication.
Review these changes against the upstream implementation when upgrading Gazebo.
