import numpy as np

# 小车的坐标和方向
x_c, y_c, theta_c = 2, 2, np.pi/4  # theta_c = 180度，转换为弧度

# 地标的坐标
x_m, y_m = 4, 4

# 构造测量矩阵 H
H = np.array([
    [-np.cos(theta_c), -np.sin(theta_c), 0, np.cos(theta_c), np.sin(theta_c)],
    [np.sin(theta_c), -np.cos(theta_c), 0, -np.sin(theta_c), np.cos(theta_c)]
])

# 状态向量
x = np.array([x_c, y_c, theta_c, x_m, y_m])

# 计算 Hx
Hx = H @ x

# 对 Hx 的每个元素保留三位小数
Hx_rounded = np.round(Hx, 3)

print(Hx_rounded)
