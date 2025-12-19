# 日志格式（CSV，MVP）

每帧一行，包含用于回放与离线动画的字段。

## Header

```
time,s,pitch,pitch_rate,lift,tilt,ceiling_z,floor_z,
rb_x,rb_z,rt_x,rt_z,fb_x,fb_z,ft_x,ft_z,
clearance_top,clearance_bottom,
lift_cmd,tilt_cmd,speed_limit,
safety_level,terrain_state,worst_point_id
```

单位：
- `time` 秒
- `s` 米
- `pitch/tilt` 弧度
- `lift` 米（fork pivot 的世界 z）
- `ceiling_z/floor_z` 米
- 角点坐标 `*_x/*_z` 米

枚举：
- `safety_level`: 0=OK, 1=WARN, 2=STOP, 3=DEGRADED
- `terrain_state`: 0=Ground, 1=FrontOnRamp, 2=OnRamp, 3=FrontInContainerRearOnRamp, 4=InContainer
- `worst_point_id`: 0=RearBottom,1=RearTop,2=FrontBottom,3=FrontTop
