#!/bin/bash

VNC_PORT=${VNC_PORT:-10010}
NOVNC_PORT=${NOVNC_PORT:-10020}
DISPLAY_WIDTH=${DISPLAY_WIDTH:-1920}
DISPLAY_HEIGHT=${DISPLAY_HEIGHT:-1080}
DISPLAY_DEPTH=${DISPLAY_DEPTH:-24}

echo "============================================"
echo "  Isaac Sim VNC 服务启动脚本"
echo "============================================"

echo "[1/7] 检查依赖..."
MISSING=""
for cmd in Xvfb fluxbox x11vnc websockify xdpyinfo; do
    if ! command -v $cmd &>/dev/null; then
        MISSING="$MISSING $cmd"
    fi
done
if [ ! -f /usr/share/novnc/vnc.html ]; then
    MISSING="$MISSING novnc"
fi
if [ -n "$MISSING" ]; then
    echo "  缺少依赖:$MISSING，正在安装..."
    DEBIAN_FRONTEND=noninteractive apt-get update -qq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y -qq xvfb x11vnc fluxbox novnc websockify x11-utils
    if [ $? -ne 0 ]; then
        echo "  错误: 依赖安装失败，请检查网络和 apt 源"
        exit 1
    fi
    echo "  依赖安装完成"
else
    echo "  所有依赖已就绪"
fi

echo "[2/7] 清理旧进程..."
pkill -9 -f "isaac-sim/kit/kit" 2>/dev/null
pkill -9 -f "isaac-sim.sh" 2>/dev/null
pkill -9 -f "Xvfb" 2>/dev/null
pkill -9 -f "fluxbox" 2>/dev/null
pkill -9 -f "x11vnc" 2>/dev/null
pkill -9 -f "websockify" 2>/dev/null
sleep 3

REMAIN=$(ps aux | grep -E "Xvfb|fluxbox|x11vnc|websockify|isaac-sim/kit" | grep -v grep | wc -l)
if [ "$REMAIN" -gt 0 ]; then
    echo "  仍有 $REMAIN 个残留进程，强制清理..."
    ps aux | grep -E "Xvfb|fluxbox|x11vnc|websockify|isaac-sim/kit" | grep -v grep | awk '{print $2}' | xargs kill -9 2>/dev/null
    sleep 2
fi
echo "  旧进程已清理"

echo "[3/7] 清理锁文件和旧日志..."
rm -f /tmp/.X0-lock /tmp/.X11-unix/X0
rm -f /tmp/x11vnc.log /tmp/websockify.log /tmp/Xvfb.log /tmp/isaac_sim_gui.log
echo "  锁文件已清理"

echo "[4/7] 启动 Xvfb 虚拟显示..."
Xvfb :0 -screen 0 ${DISPLAY_WIDTH}x${DISPLAY_HEIGHT}x${DISPLAY_DEPTH} -ac +extension GLX +render -noreset >> /tmp/Xvfb.log 2>&1 &
sleep 3
export DISPLAY=:0

if xdpyinfo -display :0 > /dev/null 2>&1; then
    echo "  Xvfb 已启动 (${DISPLAY_WIDTH}x${DISPLAY_HEIGHT}x${DISPLAY_DEPTH})"
else
    echo "  错误: Xvfb 启动失败!"
    exit 1
fi

echo "[5/7] 启动窗口管理器和 VNC 服务..."
fluxbox -display :0 >> /tmp/fluxbox.log 2>&1 &
sleep 1

x11vnc -display :0 -forever -nopw -shared -rfbport ${VNC_PORT} -bg -o /tmp/x11vnc.log
sleep 2

if netstat -tlnp 2>/dev/null | grep -q ":${VNC_PORT}"; then
    echo "  x11vnc 已启动 (端口 ${VNC_PORT})"
else
    echo "  错误: x11vnc 端口 ${VNC_PORT} 未监听!"
    tail -10 /tmp/x11vnc.log
    exit 1
fi

websockify --web=/usr/share/novnc ${NOVNC_PORT} localhost:${VNC_PORT} >> /tmp/websockify.log 2>&1 &
sleep 2

if netstat -tlnp 2>/dev/null | grep -q ":${NOVNC_PORT}"; then
    echo "  websockify 已启动 (端口 ${NOVNC_PORT})"
else
    echo "  错误: websockify 端口 ${NOVNC_PORT} 未监听!"
    tail -10 /tmp/websockify.log
    exit 1
fi

echo "[6/7] 启动 Isaac Sim..."
cd /isaac-sim
./isaac-sim.sh \
    --/persistent/isaac/asset_root/default="$OMNI_SERVER" \
    --merge-config="/isaac-sim/config/open_endpoint.toml" \
    --allow-root \
    > /tmp/isaac_sim_gui.log 2>&1 &
ISAAC_PID=$!
echo "  Isaac Sim 正在启动 (PID: $ISAAC_PID, 日志: /tmp/isaac_sim_gui.log)"

echo "[7/7] 等待 Isaac Sim 启动完成..."
READY=0
for i in $(seq 1 60); do
    if grep -q "app ready" /tmp/isaac_sim_gui.log 2>/dev/null; then
        READY=1
        break
    fi
    if ! kill -0 $ISAAC_PID 2>/dev/null; then
        echo "  错误: Isaac Sim 进程已退出!"
        tail -20 /tmp/isaac_sim_gui.log
        exit 1
    fi
    sleep 5
    echo "  等待中... (${i}/60)"
done

if [ "$READY" -eq 1 ]; then
    echo "  Isaac Sim 启动完成!"
else
    echo "  警告: Isaac Sim 启动超时，请检查日志: /tmp/isaac_sim_gui.log"
fi

echo ""
echo "============================================"
echo "  所有服务已启动!"
echo "============================================"
echo "  VNC 客户端访问:   <IP>:${VNC_PORT}"
echo "  noVNC 浏览器访问: http://<IP>:${NOVNC_PORT}/vnc.html"
echo "  Isaac Sim 日志:   tail -f /tmp/isaac_sim_gui.log"
echo "============================================"
echo ""
echo "运行进程:"
ps aux | grep -E "Xvfb|fluxbox|x11vnc|websockify|isaac-sim/kit" | grep -v grep

echo ""
echo "注意事项:"
echo "  - A100/A800 GPU 不支持 Vulkan Ray Tracing，Isaac Sim 界面可能无法正常渲染"
echo "  - 如遇黑屏，请使用 standalone 模式: cd /isaac-sim && ./python.sh <脚本路径>"
echo "  - 停止所有服务: pkill -f 'isaac-sim/kit/kit'; pkill -f Xvfb; pkill -f x11vnc; pkill -f websockify; pkill -f fluxbox"
