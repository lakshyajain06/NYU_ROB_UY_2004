import numpy as np

def rotation_x(angle):
    return np.array(
        [
            [1, 0, 0, 0],
            [0, np.cos(angle), -np.sin(angle), 0],
            [0, np.sin(angle), np.cos(angle), 0],
            [0, 0, 0, 1],
        ]
    )

def rotation_y(angle):
    return np.array(
        [
            [np.cos(angle), 0, np.sin(angle), 0],
            [0, 1, 0, 0],
            [-np.sin(angle), 0, np.cos(angle), 0],
            [0, 0, 0, 1],
        ]
    )

def rotation_z(angle):
    return np.array(
        [
            [np.cos(angle), -np.sin(angle), 0, 0],
            [np.sin(angle), np.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )

def translation(x, y, z):
    return np.array(
        [
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1],
        ]
    )

def get_T01(theta_1):
    return rotation_z(theta_1)

def get_T12(theta_2):
    # motor_rotation = rotation_y(theta_2)
    # base_trans = translation(0, 0.3, 0) @ rotation_x(np.pi/2) .

    # return motor_rotation @ base_trans
    return translation(0, 0.3, 0) @ rotation_x(np.pi/2) @ rotation_z(theta_2)

def get_T23(theta_3):
    # base_translation = translation(0.4, 0, 0)
    # return base_translation @ rotation_z(theta_3)
    return translation(0.4, 0, 0) @ rotation_z(theta_3) 

def get_T34():
    # return translation(0, 0.3, 0)
    return translation(0, 0.3, 0) @ rotation_x(-np.pi/2)

def get_FK(theta_1, theta_2, theta_3):
    return get_T01(theta_1) @ get_T12(theta_2) @ get_T23(theta_3) @ get_T34()

def ee_in_collision(thetas, p_point, tolerance):
    end_pose = get_FK(*thetas)[0:3, 3]
    return np.linalg.norm(end_pose-p_point) < tolerance
    
def path_in_collision(path, object_list):
    print(len(path))
    print(len(object_list))
    for frame in path:
        for object in object_list:
            if ee_in_collision(frame, object[0], object[1]):
                return True
    
    return False