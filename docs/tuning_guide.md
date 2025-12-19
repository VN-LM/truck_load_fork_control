# 参数调试指南（MVP）

## 1. 关键阈值

- `margin_top / margin_bottom`：软边界余量（m）。建议从 0.05–0.10m 起。
- `warn_threshold`：进入 WARN 的净空阈值（m）。建议 0.10–0.15m。
- `hard_threshold`：进入 STOP 的净空阈值（m）。建议 0.00m 或允许少量负值（例如 -0.01m）取决于测量误差。

## 2. 搜索范围与分辨率

- `search_lift_half_range_m`：每帧 lift 搜索半径（m）。建议 0.05–0.15m。
- `search_tilt_half_range_rad`：tilt 搜索半径（rad）。建议 2–6°（0.035–0.105rad）。
- `grid_lift_steps / grid_tilt_steps`：网格步数。MVP 默认 9×9，可权衡速度与平滑。

## 3. 权重建议

- `w_center`：居中度权重，增大可让料笼“尽量居中”在上下净空之间。
- `w_dl/w_dt`：动作幅度惩罚，增大可抑制抖动，但可能更贴近边界。
- `w_smooth`：速率变化惩罚，增大可显著提升可控性与可诊断性。

## 4. 退化模式（DEGRADED）

当 `dt` 异常、输入无效或 pitch_rate 抖动过大：
- margin 放大（例如 ×2）
- rate_limit 与 speed_limit 降低（例如 ×0.5）

## 5. 调试流程

1) 用 `viz_realtime` 内置轨迹跑通；观察最危险切换段 `FrontInContainerRearOnRamp`。
2) 调整 margin 直到 WARN/STOP 只在预期边界出现。
3) 增大 `w_smooth` 消除抖动，再用 `w_dl/w_dt` 折中响应速度。
4) 用 CSV 回放和 `tools/animate.py` 导出动画用于评审。\
