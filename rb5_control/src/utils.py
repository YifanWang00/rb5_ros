import numpy as np
from math import cos, sin, pi, pow


id_dict = {0:1, 3:0, 4:2}
H = np.array(
    [[-cos(pi/4), -sin(pi/4), 0, cos(pi/4), sin(pi/4)],
    [sin(pi/4), -cos(pi/4), 0, -sin(pi/4), cos(pi/4)]]
)
R = np.diag([pow(0.05, 2), pow(0.05, 2)])

def get_H(theta):
    return np.array([
    [-np.cos(theta), -np.sin(theta), 0, np.cos(theta), np.sin(theta)],
    [np.sin(theta), -np.cos(theta), 0, -np.sin(theta), np.cos(theta)]
])

def pad_zero_in_vector_to_n(vector, n):
    return np.expand_dims(np.pad(vector, (0, n-vector.size), "constant"), axis=1).T

def pad_zero_in_vector_with_position(id, V, id_dict):
    if V.ndim == 2:
        V = V.T[0]
    if id in id_dict:
        idx = id_dict[id]
    else:
        print('{} NOT in id dictionary!'.format(id))
    n = len(id_dict) * 2 + 3
    ret_vec = np.zeros(n)
    for i in range(3):
        ret_vec[i] = V[i]
    ret_vec[3+2*idx] = V[3]
    ret_vec[4+2*idx] = V[4]

    return np.expand_dims(ret_vec, axis=1)

def pad_zero_in_matrix_with_position(id, M, id_dict):
    V = np.diag(M)
    pad_diag_vec = pad_zero_in_vector_with_position(id, V, id_dict)
    return np.diag(pad_diag_vec.T[0])
    
def cut_element_in_vector_with_position(id, V, id_dict):
    if V.ndim == 2:
        V = V.T[0]
    if id in id_dict:
        idx = id_dict[id]
    else:
        print('{} NOT in id dictionary!'.format(id))
    ret_vec = np.zeros(5)
    for i in range(3):
        ret_vec[i] = V[i]
    ret_vec[3] = V[3+2*idx]
    ret_vec[4] = V[4+2*idx]

    return np.expand_dims(ret_vec, axis=1)

def cut_element_in_matrix_with_position(id, M, id_dict):
    V = np.diag(M)
    del_diag_vec = cut_element_in_vector_with_position(id, V, id_dict)
    return np.diag(del_diag_vec.T[0])

# def calculate_Kalman_gain_coeff_K(cov_mat, H, R, id, id_dict):
#     cutted_cov_mat = cut_element_in_matrix_with_position(id, cov_mat, id_dict)
#     return cutted_cov_mat @ H.T @ np.linalg.inv(H @ cutted_cov_mat @ H.T + R)

def calculate_Kalman_gain_coeff_K(cov_mat, H, R, id, id_dict):
    cutted_cov_mat = cut_element_in_matrix_with_position(id, cov_mat, id_dict)
    intermediate = np.dot(cutted_cov_mat, H.T)
    gain = np.dot(intermediate, np.linalg.inv(np.dot(H, np.dot(cutted_cov_mat, H.T)) + R))
    return gain

def utilize_Kalman_gain_coeff_K(cov_mat, H, R, xk, zk, id, id_dict):
    observation_residuals = calculate_observation_residuals(H, xk, zk ,id, id_dict)
    Kalman_gain_coeff_K = calculate_Kalman_gain_coeff_K(cov_mat, H, R, id, id_dict)
    xk = np.expand_dims(xk, axis=1)
    return (xk - pad_zero_in_vector_with_position(id, np.dot(Kalman_gain_coeff_K, observation_residuals), id_dict)).T[0]

def calculate_observation_residuals(H, xk, zk, id, id_dict):
    cut_xk = cut_element_in_vector_with_position(id, xk, id_dict)
    return np.dot(H, cut_xk) - np.expand_dims(np.array(zk), axis=1)

def update_state_covariance(cov_mat, H, K):
    KH = np.dot(K, H)
    identity_mat = np.ones(KH.shape)
    return np.dot(identity_mat - KH, cov_mat)


if __name__ == "__main__":
    id_dict = {0:1, 3:0, 4:2}
    dummy_vec = np.array([1, 2, 3, 4, 5])
    dummy_mat = np.diag(dummy_vec)

    print('-'*15)
    print("Start testing")
    print('-'*15)

    print(" - Testing function pad_zero_in_vector_to_n()...")
    test_pad_zero_to_n = pad_zero_in_vector_to_n(dummy_vec, 8)
    print(test_pad_zero_to_n.T)
    print('-'*15)

    print(" - Testing function pad_zero_in_vector_with_position()...")
    test_pad_zero_in_position = pad_zero_in_vector_with_position(0, dummy_vec, id_dict)
    print(test_pad_zero_in_position)
    print('-'*15)

    print(" - Testing function pad_zero_in_matrix_with_position()...")
    test_pad_zero_in_matrix_with_position = pad_zero_in_matrix_with_position(0, dummy_mat, id_dict)
    print(test_pad_zero_in_matrix_with_position)
    print('-'*15)

    print(" - Testing function cut_element_in_vector_with_position()...")
    test_cut_element_in_vector_with_position = cut_element_in_vector_with_position(0, test_pad_zero_in_position, id_dict)
    print(test_cut_element_in_vector_with_position)
    print('-'*15)

    print(" - Testing function cut_element_in_matrix_with_position()...")
    test_cut_element_in_matrix_with_position = cut_element_in_matrix_with_position(0, test_pad_zero_in_matrix_with_position, id_dict)
    print(test_cut_element_in_matrix_with_position)
    print('-'*15)

    print(" - Testing function calculate_Kalman_gain_coeff_K()...")
    test_K = calculate_Kalman_gain_coeff_K(test_pad_zero_in_matrix_with_position, H, R, 0, id_dict)
    print(test_K)
    print('-'*15)

    print(np.dot(H , np.array([2, 2, np.pi/4, 4, 4])))
