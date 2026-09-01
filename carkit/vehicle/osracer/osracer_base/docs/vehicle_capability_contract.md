# Vehicle Capability Contract

OSRacer Base reads vehicle geometry and operating limits from a compatible
controller when each serial connection is established. This document defines
the public host-visible contract used by the ROS 2 driver.

## Startup sequence

The driver completes this sequence before accepting motion commands:

1. `stream off`
2. `fw version`
3. `profile get`
4. `vehicle get`
5. `stream sync`
6. `s`
7. `link up ros`

The firmware response must report Proto 1.1. The profile response must report
`State=READY` and `Motion=Yes`.

## Vehicle response

`vehicle get` returns one newline-terminated response with this exact field
order:

```text
VEHICLE: Contract=1, Profile=<profile>, Schema=<schema>,
WheelbaseMm=<uint>, ForwardMaxMmps=<uint>, ReverseMaxMmps=<uint>,
SteeringMaxMdeg=<uint>, BatteryMinMv=<uint>, BatteryMaxMv=<uint>
```

| Field | Unit | Accepted range | Driver use |
| --- | --- | --- | --- |
| `WheelbaseMm` | millimetres | 1 to 9999 | Twist-to-steering conversion |
| `ForwardMaxMmps` | mm/s | 1 to 20000 | Forward command limit |
| `ReverseMaxMmps` | mm/s | 1 to 20000 | Reverse command limit |
| `SteeringMaxMdeg` | millidegrees | 1 to 90000 | Steering command limit |
| `BatteryMinMv` | millivolts | 0 to 60000 | Battery display lower bound |
| `BatteryMaxMv` | millivolts | 1 to 60000 | Battery display upper bound |

## Validation and lifetime

The driver requires Contract 1, exact field spelling and order, unsigned integer
values, and a Profile/Schema pair identical to the preceding `profile get`
response. Geometry, speed, and steering values must be positive and within the
accepted ranges above. The battery minimum must be lower than the maximum.

Missing, malformed, unknown, out-of-range, or timed-out responses close the
connection without enabling motion. Firmware that does not implement
`vehicle get` is therefore not motion-compatible with this driver version.

Accepted values exist only in memory and are bound to the serial connection
that supplied them. Closing, replacing, or reconnecting the device clears the
entire capability set. A new connection must complete the firmware version,
profile, and vehicle checks again.

## ROS boundary

Vehicle capability values are not published as normal ROS topics or parameters
and are not included in routine acceptance logs. This limits normal interface
exposure; it is not encryption or a confidentiality boundary.

TF remains a ROS responsibility. The capability response does not contain frame
names, sensor mounting transforms, or static transforms, and it does not change
the driver's `odom` to `base_footprint` transform semantics.
