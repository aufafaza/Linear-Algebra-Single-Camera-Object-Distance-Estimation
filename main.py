from ultralytics import YOLO 
import cv2 
import numpy as np 
import time 
from pymavlink import mavutil

def get_model():
    model = YOLO("yolo11n.pt")
    return model

def connect_mavlink(connection_string):
    mav = mavutil.mavlink_connection(connection_string)
    mav.wait_heartbeat()
    print("MAVLink connected")
    return mav

def get_drone_state(mav):
    msg = mav.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE'], blocking=True)

    position = None
    attitude = None

    if msg.get_type() == 'LOCAL_POSITION_NED':
        position = np.array([msg.x, msg.y, -msg.z])  # convert NED → ENU

    elif msg.get_type() == 'ATTITUDE':
        attitude = np.array([msg.roll, msg.pitch, msg.yaw])

    return position, attitude

def open_camera(index=0):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError("Camera not accessible")
    return cap

def capture_two_frames(cap, delta_t=1.0):
    ret1, frame1 = cap.read()
    t1 = time.time()

    time.sleep(delta_t)

    ret2, frame2 = cap.read()
    t2 = time.time()

    if not ret1 or not ret2:
        raise RuntimeError("Frame capture failed")

    return frame1, frame2, t1, t2


def pixel_to_camera_ray(u, v, K_inv):
    pixel_h = np.array([u, v, 1.0])
    ray_cam = K_inv @ pixel_h
    ray_cam = ray_cam / np.linalg.norm(ray_cam)
    return ray_cam


def rotation_matrix_from_euler(roll, pitch, yaw):
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])

    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx


def rotate_ray_to_world(ray_cam, roll, pitch, yaw):
    R = rotation_matrix_from_euler(roll, pitch, yaw)
    ray_world = R @ ray_cam
    ray_world /= np.linalg.norm(ray_world)
    return ray_world


def intersect_ground(ray_world, drone_height):
    ez = np.array([0, 0, 1])
    denom = ray_world.dot(ez)

    if abs(denom) < 1e-6:
        return None  # parallel to ground

    lam = drone_height / denom
    return lam * ray_world


def process_frame(u, v, attitude, height):
    ray_cam = pixel_to_camera_ray(u, v, K_inv)
    ray_world = rotate_ray_to_world(ray_cam, *attitude)
    P_ground = intersect_ground(ray_world, height)
    return P_ground


def estimate_distance(P1, P2, drone_translation):
    delta_ground = np.linalg.norm(P2 - P1)
    baseline = np.linalg.norm(drone_translation)

    if delta_ground < 1e-6:
        return None

    D = baseline / delta_ground
    return D


def monocular_motion_stereo(u1, v1, u2, v2,
                            attitude1, attitude2,
                            pos1, pos2,
                            height):

    P1 = process_frame(u1, v1, attitude1, height)
    P2 = process_frame(u2, v2, attitude2, height)

    drone_translation = pos2 - pos1

    distance = estimate_distance(P1, P2, drone_translation)
    return distance


