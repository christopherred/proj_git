#!/bin/bash

# 检查log文件夹是否存在，如果存在则删除
if [ -d "log" ]; then
    echo "删除已存在的log文件夹..."
    rm -rf log
fi

# 创建新的log文件夹
echo "创建新的log文件夹..."
mkdir -p log

# 运行ros_master.py并将输出重定向到log/manager.log
echo "启动ros_manager.py..."
python3 ros_manager.py > log/manager.log 2>&1 &

# 等待一秒钟确保ROS节点已启动
sleep 1

# 运行streamlit应用
echo "启动Streamlit WebUI..."
streamlit run webui.py
