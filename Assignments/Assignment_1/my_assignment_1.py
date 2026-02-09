import numpy as np

def rotate2D(theta, p_point):
    trans = np.array(
        [
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ]
    )

    return np.matmul(trans, p_point)


def rotate3D(theta, axis_of_rotation, p_point):

    if axis_of_rotation == 'z':
        # rotation around z
        trans = np.array(
            [
                [np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta),  np.cos(theta), 0],
                [0,              0,             1]
            ]
        )
    elif axis_of_rotation == 'y':
        # rotation around y
        trans = np.array(
            [
                [ np.cos(theta), 0, np.sin(theta)],
                [ 0,             1, 0            ],
                [-np.sin(theta), 0, np.cos(theta)]
            ]
        )
    elif axis_of_rotation == 'x':
        # rotation around x
        trans = np.array(
            [
                [1,              0,             0],
                [0, np.cos(theta), -np.sin(theta)],
                [0, np.sin(theta),  np.cos(theta)]
            ]
        )
    else:
        return "Invalid axis input"

    return np.matmul(trans, p_point)

def rotate3D_many_times(rotation_list, p_point):
    new_point = p_point
    for rotation in rotation_list:
        new_point = rotate3D(*rotation, new_point)

    return new_point