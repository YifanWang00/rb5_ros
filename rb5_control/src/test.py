import numpy as np
from utils import get_H, calculate_Kalman_gain_coeff_K, utilize_Kalman_gain_coeff_K, \
    calculate_observation_residuals, update_state_covariance

# def expand_diag_matrix(original_matrix, n):
#     diag_elements = np.diag(original_matrix)
    
#     expanded_matrix = np.zeros((n, n))

#     for i in range(min(len(diag_elements), n)):
#         expanded_matrix[i, i] = diag_elements[i]
    
#     return expanded_matrix

# X_k = np.array([1,1,1,1,1,1,1])
# State_cov = np.diag([2,2,2,2,2,2,2]) 
# Q_m = np.diag([1,1,1]) 
# State_cov = State_cov + expand_diag_matrix(Q_m, len(X_k))
# print(State_cov)

# def expand_and_fill_diag_matrix(original_matrix, n, x):
#     original_size = original_matrix.shape[0]
    
#     expanded_matrix = np.zeros((n, n))

#     for i in range(original_size):
#         expanded_matrix[i, i] = original_matrix[i, i]

#     for i in range(original_size, n):
#         expanded_matrix[i, i] = x
    
#     return expanded_matrix

# Cov_init = 1
# X_k = np.array([1,1,1,1,1,1,1])
# State_cov = np.diag([2,2,2,2,2]) 
# State_cov = expand_and_fill_diag_matrix(State_cov, len(X_k), Cov_init)
# print(State_cov)

# def expand_X_update(original_array, new_length):

#     if len(original_array) != 3:
#         raise ValueError("Input must be a 1D array of length 3.")

#     if new_length < 3:
#         raise ValueError("New length must be at least 3.")

#     # Create an array of zeros with the new length
#     expanded_array = np.zeros(new_length)

#     # Copy the original array into the expanded array
#     expanded_array[:3] = original_array

#     return expanded_array

# X_k = np.array([1.0,1.0,1.0,1.0,1.0,1.0,1.0])
# X_update = expand_X_update(np.array([1,1,1]), len(X_k))
# print(type(X_k[0]))
# print(type(X_update[0]))
# X_k += X_update
# print(X_k)

H_m = get_H(0)
print(H_m, H_m.shape)

test_cal_ob_residual = calculate_observation_residuals(H_m, X_k, marker_info[0], marker_name, marker_dic)
