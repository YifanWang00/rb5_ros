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


# 小车的全局坐标和方向
x_c, y_c, theta_c = 2, 2, np.pi / 4  # 小车位置(2,2)和方向角45度

# 观测到的地标相对位置
x_obs, y_obs = 2 * np.sqrt(2), 0  # 假设观测到的地标相对于小车的局部坐标

# 将局部坐标转换为全局坐标
x_m = x_c + x_obs * np.cos(theta_c) - y_obs * np.sin(theta_c)
y_m = y_c + x_obs * np.sin(theta_c) + y_obs * np.cos(theta_c)

print((x_m, y_m))  # 地标的全局坐标

n=4

def expand_diag_matrix(original_matrix, n):
    # 提取原始矩阵的对角元素
    diag_elements = np.diag(original_matrix)
    
    # 创建一个n*n的零矩阵
    expanded_matrix = np.zeros((n, n))

    # 将对角元素放置在新矩阵的对角线上
    for i in range(min(len(diag_elements), n)):
        expanded_matrix[i, i] = diag_elements[i]
    
    return expanded_matrix

# 示例
original_matrix = np.diag([0.05 ** 2, 0.05 ** 2, 0.15 ** 2])  # x, y, z是原始3x3对角矩阵的对角线元素
expanded_matrix = expand_diag_matrix(original_matrix, n)
print(expanded_matrix)

import numpy as np

def expand_and_fill_diag_matrix(original_matrix, n, x):
    # 确定原始矩阵的大小
    original_size = original_matrix.shape[0]
    
    # 创建一个n*n的零矩阵
    expanded_matrix = np.zeros((n, n))

    # 将原始矩阵的对角线元素复制到新矩阵
    for i in range(original_size):
        expanded_matrix[i, i] = original_matrix[i, i]

    # 用x填充剩余的对角线元素
    for i in range(original_size, n):
        expanded_matrix[i, i] = x
    
    return expanded_matrix

# 示例
original_matrix = np.diag([1, 1, 1])  # 假设原始矩阵是3x3的，对角线元素是a, b, c
n = 5  # 新矩阵的大小
x = 5  # 填充值
expanded_matrix = expand_and_fill_diag_matrix(original_matrix, n, x)
print(expanded_matrix)

