# 架构设计（MVP）

## 目标

在“地面→上桥→坡上→前轮进车厢/后轮仍在坡上→完全进车厢”的过程中，车体 pitch 与地形高度变化会使料笼相对于车厢顶/地板的净空快速变化。

本仓库实现一个**控制内核（C++17 library）**，每帧根据上层输入估算料笼关键点高度，在当前 `(lift, tilt)` 附近做轻量可行域搜索，选择满足上下净空约束的目标指令；同时提供**安全状态机**与**可视化/回放**能力。

约束：不做完整 MPC；不依赖外部 QP；可落地、可测试、可回放动画。

---

## 坐标与变量定义

### 2D 侧视（x-z 平面）

- $x$：沿车辆行驶方向，**进入车厢方向为正**。
- $z$：竖直向上。
- 门口位置：默认 $x = 0$。

### 进度 s

`s` 定义为：**货叉俯仰铰点（fork pivot）**在世界坐标系中的 $x$ 位置（米）。

- `s < 0`：铰点在门外。
- `s = 0`：铰点位于门口。
- `s > 0`：铰点在车厢内。

该定义便于将“门框/地板/坡面”可视化为固定几何，并让上层定位系统可直接提供 `s`。

### 姿态

- `pitch`：车体绕 $y$ 轴的俯仰角（弧度），**车头上扬为正**。
- `tilt_angle`：货叉相对车体的俯仰（弧度），**叉尖上扬为正**。
- 料笼在世界的总俯仰：

$$\theta = pitch + tilt\_angle$$

### 料笼矩形包络

MVP 中将料笼建模为 2D 矩形：
- `rack.length`：沿 $x$ 的长度（m）
- `rack.height`：沿 $z$ 的高度（m）

使用 4 个角点：
- `RearBottom`, `RearTop`, `FrontBottom`, `FrontTop`

料笼通过 `rack.mount_offset`（从 fork pivot 到料笼“后下角”的 2D 偏置，单位 m）与 `lift_pos`（fork pivot 的世界 $z$ 高度，m）放置到世界坐标。

---

## 输入/输出边界（控制内核不耦合传感器细节）

输入（上层提供）：
- `dt`
- `pitch, pitch_rate`
- `s`
- `TerrainState`（用于诊断/可视化；控制上可用于保守策略）
- 当前执行器状态 `lift_pos, tilt_angle`
- 环境几何：
  - 简化：`ceiling_z, floor_z`
  - 扩展：`ceiling_plane, floor_plane`（$ax+by+cz+d=0$，MVP 假定 $y=0$）
- 设备参数：`RackParams`, `ForkliftParams`

输出：
- `ControlCommand { lift_target, lift_rate_limit, tilt_target, tilt_rate_limit, speed_limit }`
- `SafetyStatus { level, code, message, clearance_top, clearance_bottom, worst_point_id }`
- `DebugFrame`：每帧的几何、约束、候选解与状态机信息（用于日志与可视化）。

---

## 控制算法：轻量可行域搜索 + 平滑代价 + 安全状态机

### 1) 每帧几何与净空计算

计算 4 个角点世界坐标（x,z）。

净空定义（对顶/底角点分别取最危险值）：

- 顶部净空：

$$clearance\_{top} = \min_{p \in top\_points} (ceiling\_z(x_p) - z_p) - margin\_{top}$$

- 底部净空：

$$clearance\_{bottom} = \min_{p \in bottom\_points} (z_p - floor\_z(x_p)) - margin\_{bottom}$$

若使用标量 `ceiling_z/floor_z`，则 $ceiling\_z(x)$ 与 $floor\_z(x)$ 为常数；若使用 plane，则按 $z=-(a x + d)/c$（假定 $y=0$，且 $c \neq 0$）。

### 2) 可行域搜索

在当前 `(lift, tilt)` 附近做小范围网格搜索（MVP 默认 2D 网格）：
- `lift ∈ [lift - ΔL, lift + ΔL]`
- `tilt ∈ [tilt - ΔT, tilt + ΔT]`

可行性：
- `clearance_top >= 0` 且 `clearance_bottom >= 0`

### 3) 在可行域内选最小代价解

定义“居中度”变量：

$$clearance\_{mid} = clearance\_{top} - clearance\_{bottom}$$

直觉：若 `clearance_mid > 0`，说明“更接近底部约束”；若 `<0`，更接近顶部约束。居中目标为 `clearance_mid_target = 0`。

代价函数：

$$J = w_{center}(0 - clearance\_{mid})^2 + w_{dl}(lift^* - lift)^2 + w_{dt}(tilt^* - tilt)^2 + w_{smooth}(\Delta \dot{lift}^2 + \Delta \dot{tilt}^2)$$

其中 $\Delta \dot{lift}$ 与 $\Delta \dot{tilt}$ 是“相对上一帧指令”的速率变化（抑制抖动）。

输出目标：
- `lift_target_position = lift*`，并给出 `lift_rate_limit`
- `tilt_target_angle = tilt*`，并给出 `tilt_rate_limit`
- `speed_limit`（推荐：净空逼近或 pitch_rate 大时降低）

若无可行解：MVP 采用“最小违反”策略：最大化 `min(clearance_top, clearance_bottom)`，并进入 `WARN/STOP`。

### 4) 安全状态机

安全等级：`OK / WARN / STOP / DEGRADED`

- `STOP`：`clearance_top` 或 `clearance_bottom` < `hard_threshold`
- `WARN`：净空接近边界（例如 `< warn_threshold`）或约束轻微违反（但未到 hard）
- `DEGRADED`：输入无效/抖动过大（例如 pitch_rate 超阈值、dt 异常）
  - 策略：更大 margin、更低速、更小 rate

输出诊断：
- 最危险角点 `worst_point_id`
- 最小净空
- 状态机等级与原因码

---

## 可视化与回放

### 实时（C++ / ImGui）

- 2D 侧视（x-z）：坡面、车厢地板/顶线、门框、料笼矩形包络、净空数值与 SafetyStatus。
- 支持：播放/暂停、时间轴拖动（回放）、参数实时调。
- 支持输入模式：内置仿真轨迹 / CSV 日志回放。

### 离线（Python / matplotlib）

- 读取 CSV/JSONL（MVP 用 CSV）
- 生成 gif 或 mp4，并叠加净空曲线。

---

## MVP 的简化与后续迭代点

MVP 为可运行闭环：几何 + 搜索 + 状态机 + 日志 + 可视化。
你说“继续”后再迭代：更真实叉车/门框几何、plane 全面支持（按角点取值）、性能优化、更丰富轨迹与输入有效性判定。\
