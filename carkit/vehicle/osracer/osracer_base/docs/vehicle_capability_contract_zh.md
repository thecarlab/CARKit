# 车辆能力合同

OSRacer Base 每次建立串口连接时，都会从兼容控制器读取车辆几何参数和运行
限值。本文定义 ROS 2 驱动使用的公开主机侧合同。

## 启动顺序

驱动在接受运动命令前依次完成：

1. `stream off`
2. `fw version`
3. `profile get`
4. `vehicle get`
5. `stream sync`
6. `s`
7. `link up ros`

固件必须报告 Proto 1.1。Profile 响应必须包含 `State=READY` 和
`Motion=Yes`。

## 车辆响应

`vehicle get` 返回一条以换行符结束的响应，字段顺序必须与下列定义一致：

```text
VEHICLE: Contract=1, Profile=<profile>, Schema=<schema>,
WheelbaseMm=<uint>, ForwardMaxMmps=<uint>, ReverseMaxMmps=<uint>,
SteeringMaxMdeg=<uint>, BatteryMinMv=<uint>, BatteryMaxMv=<uint>
```

| 字段 | 单位 | 接受范围 | 驱动用途 |
| --- | --- | --- | --- |
| `WheelbaseMm` | mm | 1 至 9999 | Twist 到转角的换算 |
| `ForwardMaxMmps` | mm/s | 1 至 20000 | 前进命令限值 |
| `ReverseMaxMmps` | mm/s | 1 至 20000 | 倒车命令限值 |
| `SteeringMaxMdeg` | 毫度 | 1 至 90000 | 转向命令限值 |
| `BatteryMinMv` | mV | 0 至 60000 | 电量显示下限 |
| `BatteryMaxMv` | mV | 1 至 60000 | 电量显示上限 |

## 校验与生命周期

驱动要求 Contract 1、严格一致的字段名称和顺序，以及无符号整数值。
Profile/Schema 必须与此前的 `profile get` 响应一致。轴距、速度和转角值必须
为正数并处于上述范围内，电池下限必须小于上限。

缺失、格式错误、未知、越界或超时的响应都会使驱动关闭连接，并且不允许运动。
因此，未实现 `vehicle get` 的固件不具备与此驱动版本的运动兼容性。

通过校验的值只保存在内存中，并绑定到提供这些值的串口连接。关闭、更换或重连
设备都会清除整组能力数据；新连接必须重新完成固件版本、Profile 和车辆能力
校验。

## ROS 边界

车辆能力值不会作为普通 ROS 话题或参数发布，也不会写入常规验收日志。该设计
用于减少普通接口暴露，但不构成加密或保密边界。

TF 仍由 ROS 负责。能力响应不包含 frame 名称、传感器安装变换或静态变换，
也不会改变驱动发布 `odom` 到 `base_footprint` 变换的语义。
