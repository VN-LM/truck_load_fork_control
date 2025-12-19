# truck_load_fork_control

C++17 控制内核 + 实时可视化调试工具，用于“平衡重叉车将料笼装入货柜车厢”过程中，由登车桥坡度与车体 pitch 变化导致的顶部/底部净空约束问题。

## 快速开始

### 1) 构建

```bash
cd /Users/ybc/code/truck_load_fork_control
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

> 说明：`viz_realtime` 需要联网 FetchContent 拉取 `glfw` 与 `imgui` 源码；若环境离线，可先只构建库/示例/测试：`-DTLF_BUILD_VIZ=OFF`。

### 2) 运行实时可视化（内置轨迹）

```bash
./build/viz_realtime
```

- 默认“内置仿真轨迹模式”播放。
- 可在右侧参数面板实时调整 margin、hard_threshold、搜索范围、权重与 rate_limit。

### 3) 生成日志并离线动画

生成 CSV：

```bash
./build/example_sim_trajectory --out /tmp/tlf_log.csv
```

### 3b) 更简单：用 Web 看侧视回放（推荐）

1) 先生成 CSV：

```bash
./build/example_sim_trajectory --out /tmp/tlf_log.csv
```

2) 打开工具页面：

- 直接用浏览器打开：`tools/web_viewer/index.html`
- 点击“选择文件”，选中 `/tmp/tlf_log.csv`

可选：加载建模配置

- 在页面里选择 `tools/web_viewer/model_config_default.json`（或你自己保存的 JSON）

说明：默认配置按“车厢在门口左侧、登车桥+地面在右侧”的示意图布局；为兼容早期示例日志，Web viewer 默认开启 `flip_log_x`。

即可在网页里播放/暂停/拖动时间轴，看 2D 侧视动画与净空数值。

生成动画（gif 或 mp4）：

```bash
python3 -m pip install -r tools/requirements.txt
python3 tools/animate.py --log /tmp/tlf_log.csv --out /tmp/tlf.gif
```

#### 常见错误

- `zsh: no such file or directory: /build/example_sim_trajectory`
	- 原因：`/build/...` 以 `/` 开头表示“从系统根目录开始”，当然不存在。
	- 正确：在仓库根目录执行 `./build/example_sim_trajectory ...`（注意前面的 `./`）。

- `zsh: unknown file attribute: h`
	- 原因：把 VS Code 里可点击的链接文本（例如 `[animate.py](http://...)`）原样粘贴进终端，`zsh` 会把 `[...]` 解析成 glob/属性匹配。
	- 正确：直接运行脚本路径：`python3 tools/animate.py --log ... --out ...`。

### 4) 跑单测

```bash
ctest --test-dir build --output-on-failure
```

## 文档

- 方案与架构：docs/architecture.md
- 参数调试指南：docs/tuning_guide.md
- 日志格式：docs/log_format.md
