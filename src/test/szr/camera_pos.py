import numpy as np

np.set_printoptions(suppress=True, precision=4)


def deg2rad(degrees):
    return np.deg2rad(degrees)

def dh_transform(theta, d, a, alpha):
    theta = np.deg2rad(theta)
    alpha = np.deg2rad(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    # return np.array([
    #     [ct, -st * ca,  st * sa, a * ct],
    #     [st,  ct * ca, -ct * sa, a * st],
    #     [0,       sa,       ca,      d],
    #     [0,        0,        0,      1]
    # ])


    # return np.array([
    #     [ct, st * ca,  st * sa, -a * ct],
    #     [-st,  ct * ca, ct * sa, a * st],
    #     [0,       -sa,       ca,     -d],
    #     [0,        0,        0,      1]
    # ])


    return np.array([
        [ct     , -st     ,   0,  a     ],
        [st * ca,  ct * ca, -sa, -sa * d],
        [st * sa,  ct * sa,  ca,  ca * d],
        [0      ,  0      ,  0 ,  1     ]
    ])

# Example DH parameters for Kinova Gen3 Lite (need to replace with actual values!)
# Format: [theta, d, a, alpha]
dh_params = [
    [0, 243.3,   0,   0],     # Joint 1
    [0,    30,   0,  90],     # Joint 2
    [0,    20, 280, 180],     # Joint 3
    [0,   245,   0,  90],     # Joint 4
    [0,    109,   0,  90],     # Joint 5
    [0,   149,   -34, -90],     # Joint 6
]

def forward_kinematics(joint_angles):
    T = np.eye(4)
    for i in range(6):
        theta = joint_angles[i]
        d, a, alpha = dh_params[i][1:]
        T_i = dh_transform(theta, d, a, alpha)
        T =  T @ T_i
    return T

def matrix_to_euler_xyz(T):
    r11, r12, r13 = T[0, :3]
    r21, r22, r23 = T[1, :3]
    r31, r32, r33 = T[2, :3]

    if np.abs(r31) != 1:
        beta = -np.arcsin(r31)
        alpha = np.arctan2(r32 / np.cos(beta), r33 / np.cos(beta))
        gamma = np.arctan2(r21 / np.cos(beta), r11 / np.cos(beta))
    else:
        gamma = 0
        if r31 == -1:
            beta = np.pi / 2
            alpha = gamma + np.arctan2(r12, r13)
        else:
            beta = -np.pi / 2
            alpha = -gamma + np.arctan2(-r12, -r13)

    return np.rad2deg([alpha, beta, gamma])

def print_output(name, T):
    px, py, pz = T[:3, 3]
    alpha, beta, gamma = matrix_to_euler_xyz(T)
    print(f"{name}:")
    # print(f"Position: px={px:.1f} mm, py={py:.1f} mm, pz={pz:.1f} mm")
    # print(f"Euler angles: α={alpha:.1f}°, β={beta:.1f}°, γ={gamma:.1f}°\n")
    print("Homogeneous transformation matrix:")
    print(np.round(T, 4))
    print("\n")

if __name__ == "__main__":
    # Sample input: REPLACE with desired angles (in degrees)
    test_inputs = {
        "ZERO": [0, 0, 0, 0, 0, 0],
        "HOME": [0, 345, 75, 0, 300, 0],
        "RETRACT": [357, 21, 150, 272, 320, 273],
        "PACKAGING": [270, 148, 148, 270, 140, 0],
        "PICK": [20.5, 313.5, 100, 265.5, 327, 57],
    }

    for name, angles in test_inputs.items():
        angles[1] += 90
        angles[2] += 90
        angles[3] += 90
        angles[5] -= 90
        T = forward_kinematics(angles)
        print_output(name, T)
