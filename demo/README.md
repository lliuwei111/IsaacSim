# Isaac Sim 仿真 Demo 使用手册

## 目录结构

```
demo/
├── app/                              # Script Editor 版本（在 Isaac Sim GUI 中运行）
│   ├── pick_biscuit_with_vis.py      # 机械臂抓取饼干盒（带视觉传感器）
│   └── load_glb_asset_app.py         # 加载 GLB 资产
├── stand_alone/                      # Standalone 版本（命令行运行，自动录制视频）
│   └── pick_biscuit_standalone.py    # 机械臂抓取饼干盒（无头模式 + 视频输出）
├── pick_place_demo.py                # 基础抓取放置演示
├── load_glb_asset.py                 # 加载 GLB 资产
├── pick_using_vsion_detection.py     # 视觉检测抓取
├── load_3dgs_asset.py                # 加载 3DGS 资产
└── asset_sensor_test.py              # 传感器测试
```

## 前置条件

- 云安全组已放行 **TCP 10002** 端口（noVNC 网页访问）
- 容器内已安装：Xvfb、fluxbox、x11vnc、noVNC（websockify）
- GLB 模型文件已放置在 `/workspace/IsaacSim/biscuit.glb`

---

## 方式一：VNC 远程操作（GUI 模式）

适用于需要实时观察仿真画面的场景，通过浏览器远程操作 Isaac Sim 界面。

### 1. 启动虚拟显示和窗口管理器

```bash
Xvfb :0 -screen 0 1920x1080x24 &
sleep 1
DISPLAY=:0 fluxbox -display :0 &
sleep 1
```

### 2. 启动 VNC 服务

```bash
DISPLAY=:0 x11vnc -display :0 -forever -nopw -shared -rfbport 10010 -bg -o /tmp/x11vnc.log &
sleep 1
/usr/bin/python3 /usr/bin/websockify --web=/usr/share/novnc 10002 localhost:10010 &
sleep 1
```

### 3. 启动 Isaac Sim GUI 模式

```bash
export DISPLAY=:0
cd /isaac-sim
/isaac-sim/isaac-sim.sh \
  --/persistent/isaac/asset_root/default="$OMNI_SERVER" \
  --merge-config="/isaac-sim/config/open_endpoint.toml" \
  --allow-root > /tmp/isaac_gui.log 2>&1 &
```

等待约 20-30 秒，检查是否启动完成：

```bash
grep "Isaac Sim Full App is loaded" /tmp/isaac_gui.log
```

### 4. 浏览器访问

```
http://<服务器公网IP>:10002/vnc.html
```

### 5. 在 Isaac Sim 中运行脚本

1. 菜单栏 → **Window** → **Script Editor**
2. 点击 **Open**（或 `Ctrl+O`）
3. 打开 `/workspace/IsaacSim/pick_biscuit_with_vis.py`
4. 点击 **Run**（或 `Ctrl+Shift+R`）

---

## 方式二：Standalone 命令行运行（无头模式）

适用于 A100 等 GPU 无需 GUI 的场景，自动录制视频到当前目录。

```bash
cd /isaac-sim
./python.sh /workspace/IsaacSim/pick_biscuit_standalone.py
```

视频输出路径：`/workspace/IsaacSim/pick_biscuit.mp4`

---

## 一键启动（VNC 模式）

```bash
Xvfb :0 -screen 0 1920x1080x24 &
sleep 1
DISPLAY=:0 fluxbox -display :0 &
sleep 1
DISPLAY=:0 x11vnc -display :0 -forever -nopw -shared -rfbport 10010 -bg -o /tmp/x11vnc.log &
sleep 1
/usr/bin/python3 /usr/bin/websockify --web=/usr/share/novnc 10002 localhost:10010 &
sleep 1
export DISPLAY=:0
cd /isaac-sim
/isaac-sim/isaac-sim.sh \
  --/persistent/isaac/asset_root/default="$OMNI_SERVER" \
  --merge-config="/isaac-sim/config/open_endpoint.toml" \
  --allow-root > /tmp/isaac_gui.log 2>&1 &
```

## 一键停止

```bash
kill $(pgrep -f "isaac-sim.sh") 2>/dev/null
kill $(pgrep -f "kit/kit") 2>/dev/null
kill $(pgrep -f "websockify") 2>/dev/null
kill $(pgrep -f "x11vnc") 2>/dev/null
kill $(pgrep -f "fluxbox") 2>/dev/null
kill $(pgrep -f "Xvfb") 2>/dev/null
```

## 常用检查命令

```bash
# 检查所有服务是否运行
ps aux | grep -E "Xvfb|fluxbox|x11vnc|websockify|kit/kit" | grep -v grep

# 检查端口监听
netstat -tlnp | grep -E "10002|10010"

# 查看 Isaac Sim 启动日志
tail -20 /tmp/isaac_gui.log
```

---

## 端口说明

| 端口 | 协议 | 用途 |
|------|------|------|
| 10002 | TCP | noVNC 网页访问入口（浏览器直接访问） |
| 10010 | TCP | x11vnc VNC 服务端（内部转发，无需外部访问） |

## 云安全组要求

需放行 **TCP 10002** 入方向。

---

## GPU 兼容性说明

| GPU 系列 | VNC GUI 模式 | Standalone 无头模式 | WebRTC 推流 |
|----------|-------------|-------------------|-------------|
| RTX 4090 | 支持 | 支持 | 支持（有 NVENC） |
| A100/A800 | 支持 | 支持 | 不支持（无 NVENC） |
| 华为 910B | 不支持 | 不支持（无 CUDA） | 不支持 |

A100/A800 缺少 NVENC 硬件编码器，无法使用 WebRTC 推流，请使用 VNC 或 Standalone 录制视频方式。
