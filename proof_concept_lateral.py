import numpy as np

# Camera Intrinsic Matrix based on 640x640 resolution
K = np.array([
    [192, 0, 320],
    [0, 192, 320],
    [0,   0,   1]
])

print("--- Step: Intrinsic Matrix K ---")
print(K)

K_inv = np.linalg.inv(K)
print("\n--- Step: Inverse Intrinsic Matrix K_inv ---")
print(K_inv)

def pixel_to_camera_ray(u, v, label):
    pixel_h = np.array([u, v, 1.0])
    ray = K_inv @ pixel_h
    normalized_ray = ray / np.linalg.norm(ray)
    
    print(f"\n--- Step: Back-Projection ({label}) ---")
    print(f"Pixel vector p_h: {pixel_h}")
    print(f"Unnormalized ray: {ray}")
    print(f"Normalized camera ray r_cam: {normalized_ray}")
    return normalized_ray

def intersect_ground(ray_cam, camera_height, label):
    dz = ray_cam[2]

    if abs(dz) < 1e-6:
        return None 

    # lambda = h / (r . e3)
    lam = -camera_height / dz
    point_g = lam * ray_cam
    
    print(f"\n--- Step: Ground Intersection ({label}) ---")
    print(f"Vertical component (dz): {dz}")
    print(f"Scaling factor (lambda): {lam}")
    print(f"Ground point P_g: {point_g}")
    return point_g

def estimate_distance(u1, v1, u2, v2, baseline, camera_height):
    r1 = pixel_to_camera_ray(u1, v1, "t")
    r2 = pixel_to_camera_ray(u2, v2, "t+dt")

    P1 = intersect_ground(r1, camera_height, "t")
    P2 = intersect_ground(r2, camera_height, "t+dt")

    if P1 is None or P2 is None:
        return None

    delta_P = np.linalg.norm(P2 - P1)
    
    print(f"\n--- Step: Parallax Calculation ---")
    print(f"Ground points difference: {P2 - P1}")
    print(f"Apparent displacement (delta_P): {delta_P}")
    
    if delta_P < 1e-6:
        return None

    # D = delta_T / delta_Pg
    D = baseline / delta_P
    return D

# Inputs
u1, v1 = 316, 325
u2, v2 = 184, 325
baseline = 1
camera_height = 0.7

D = estimate_distance(u1, v1, u2, v2, baseline, camera_height)

print(f"\n--- Final Estimate ---")
print("Estimated distance (D_obj):", D, "meters")