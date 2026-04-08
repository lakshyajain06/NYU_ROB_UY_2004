import math
import numpy as np
import scipy

def forward_kinematics(theta1, theta2, theta3):
    def rotation_x(angle):
        return np.array([
            [1, 0, 0, 0],
            [0, np.cos(angle), -np.sin(angle), 0],
            [0, np.sin(angle), np.cos(angle), 0],
            [0, 0, 0, 1]
        ])

    def rotation_y(angle):
        return np.array([
            [np.cos(angle), 0, np.sin(angle), 0],
            [0, 1, 0, 0],
            [-np.sin(angle), 0, np.cos(angle), 0],
            [0, 0, 0, 1]
        ])

    def rotation_z(angle):
        return np.array([
            [np.cos(angle), -np.sin(angle), 0, 0],
            [np.sin(angle), np.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def translation(x, y, z):
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])

    T_0_1 = translation(0.07500, -0.08350, 0) @ rotation_x(1.57080) @ rotation_z(theta1)
    T_1_2 = rotation_y(-1.57080) @ rotation_z(theta2)
    T_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta3)
    T_3_ee = translation(0.06231, -0.06216, 0.01800)
    T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
    return T_0_ee[:3, 3]


def inverse_kinematics_with_optimizer(target):

    def get_error(theta, desired_position):
        scalar_difference = np.linalg.norm(forward_kinematics(*theta) - desired_position)
        return scalar_difference
    
    initial_guess = [0, 0, 0]
    res = scipy.optimize.minimize(get_error, initial_guess, args=(target,) )
    return res.x

def inverse_kinematics_with_gradient(target):
    def get_cost(theta, target):
        curr_pos = forward_kinematics(*theta)
        sum_sqr_err = np.square(target - curr_pos).sum()
        mean_abs_err = np.absolute(target - curr_pos).mean()

        return sum_sqr_err, mean_abs_err
    
    def get_gradient(theta, target):
        def compute_partial(index, theta, target):
            dx = 0.0001
            diff = np.array([0.0 , 0.0 , 0.0])
            diff[index] = 0.0001
            
            upper = get_cost(theta + diff, target)[0]
            lower = get_cost(theta - diff, target)[0]
            return (upper - lower) / (2 * dx)
        
        grad = []

        for i in range(len(theta)):
            grad.append(compute_partial(i, theta, target))

        return np.array(grad)
    
    max_steps = 5000
    tolerance = 0.0002
    lbd = 1

    curr_guess = np.array([0.0, 0.0, 0.0])
    for i in range(max_steps):
        curr_cost = get_cost(curr_guess, target)
        if (curr_cost[1]) < tolerance:
            return curr_guess
        
        grad = get_gradient(curr_guess, target)
        curr_guess -= lbd * grad

    return curr_guess

