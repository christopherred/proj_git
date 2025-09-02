import numpy as np

# 读取CSV文件（已修正的版本）
with open('/home/robot_code_v2/ROS2/src/chassis/chassis_controller/mpc_controller/mpc_controller/square_theta.csv', 'r') as f:
    # 增加对行的迭代并将判断移到循环末尾
    data = [float(line.strip()) for line in f if line.strip()]

# 参数化插值
n = len(data)
x_original = np.linspace(0, 1, n)
y_original = np.array(data)

# 创建新的参数空间（点数翻倍）
x_new = np.linspace(0, 1, 2 * n - 1)  # 修正总点数匹配
# 此处改为 2*n-1 才能使转换后点数精确翻倍
# （n=原点数 → 新点数=2n-1）

# 线性插值
interpolated = np.interp(x_new, x_original, y_original)

# 写入新文件（新增格式化优化）
with open('/home/robot_code_v2/ROS2/src/chassis/chassis_controller/mpc_controller/mpc_controller/square_theta.csv', 'w') as f:
    formatted = "\n".join([f"{val:.15f}".rstrip('0').rstrip('.') if '.0' in f"{val:.15f}" else f"{val:.15f}" for val in interpolated])
    f.write(formatted)