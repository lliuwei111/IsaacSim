#!/bin/bash

CONTAINER_NAME=${CONTAINER_NAME:-isaac-sim}
VNC_PORT=${VNC_PORT:-10010}
NOVNC_PORT=${NOVNC_PORT:-10020}

echo "============================================"
echo "  Isaac Sim VNC 远程启动脚本 (宿主机)"
echo "============================================"

echo "[1/2] 检查容器状态..."
if ! docker ps --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "  错误: 容器 '${CONTAINER_NAME}' 未运行!"
    echo "  请先启动容器: docker run --name isaac-sim ..."
    exit 1
fi
CONTAINER_ID=$(docker ps --format "{{.ID}} {{.Names}}" | grep " ${CONTAINER_NAME}$" | awk '{print $1}')
echo "  容器已找到: ${CONTAINER_NAME} (${CONTAINER_ID})"

echo "[2/2] 在容器内启动 VNC + Isaac Sim..."
docker exec ${CONTAINER_ID} bash -c "VNC_PORT=${VNC_PORT} NOVNC_PORT=${NOVNC_PORT} bash /isaac-sim/tools/start_vnc_isaacsim.sh"

echo ""
echo "============================================"
echo "  启动完成!"
echo "============================================"
echo "  VNC 客户端访问:  <宿主机IP>:${VNC_PORT}"
echo "  noVNC 浏览器访问: http://<宿主机IP>:${NOVNC_PORT}/vnc.html"
echo "============================================"
