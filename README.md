# Monocular Distance Estimation POC

A proof-of-concept implementation for estimating object distances using monocular vision and ground plane intersection. This repository contains two Python scripts demonstrating distance estimation through parallax analysis from temporal frame displacement.

## Overview

These scripts implement a computer vision technique that estimates the distance to objects on the ground plane by analyzing their apparent motion across consecutive camera frames. The method uses camera intrinsics, ray back-projection, and ground plane intersection to calculate real-world distances without requiring stereo cameras or depth sensors.

## Method

The distance estimation works through the following steps:

1. **Back-projection**: Convert 2D pixel coordinates to 3D camera rays using the inverse intrinsic matrix
2. **Ground Intersection**: Find where each ray intersects the ground plane based on camera height
3. **Parallax Calculation**: Measure the apparent displacement of the object between frames
4. **Distance Estimation**: Calculate actual distance using the relationship `D = baseline / delta_P`

### Key Assumptions

- Objects are on a flat ground plane
- Camera intrinsics are known and fixed
- Camera height above ground is known
- Small time interval between frames (object moves rigidly)

## Files

### POC_forward.py

Estimates distance for objects moving **toward/away** from the camera (forward motion).

**Parameters:**

- Camera height: 1.0 meter
- Baseline (temporal displacement): 0.5 meters
- Resolution: 640x640 pixels
- Focal length: 192 pixels

**Example input:**

```
370 362
376 405
```

### POC_lateral.py

Estimates distance for objects moving **laterally** across the camera's field of view.

**Parameters:**

- Camera height: 0.7 meters
- Baseline (temporal displacement): 1.0 meter
- Resolution: 640x640 pixels
- Focal length: 192 pixels

**Example input:**

```
316 325
184 325
```

## Requirements

```bash
pip install numpy
```

## Usage

Run either script and provide two sets of pixel coordinates when prompted:

```bash
python POC_forward.py
# Enter first observation: 370 362
# Enter second observation: 376 405
```

The script will output:

- Step-by-step calculations with matrices
- Back-projected camera rays
- Ground plane intersection points
- Parallax displacement
- Final distance estimate in meters

## Camera Intrinsic Matrix

Both scripts use the same intrinsic matrix K for a 640x640 resolution camera:

```
K = [[192,   0, 320],
     [  0, 192, 320],
     [  0,   0,   1]]
```

Where:

- `fx = fy = 192`: Focal length in pixels
- `cx = cy = 320`: Principal point at image center

## Understanding the Output

The scripts provide detailed output for each computational step:

- **Intrinsic Matrix K**: Camera parameters
- **Back-Projection**: Conversion from pixels to 3D rays
- **Ground Intersection**: Where rays hit the ground plane
- **Parallax Calculation**: Apparent motion on ground plane
- **Final Estimate**: Calculated distance in meters

## Limitations

- Assumes flat ground plane (fails on slopes/stairs)
- Requires accurate camera calibration
- Sensitive to camera height measurement errors
- Objects must be on the ground plane
- Does not handle occlusions or multiple objects
- Requires tracked object between frames

## Applications

This technique can be useful for:

- Robotics navigation and obstacle avoidance
- Autonomous vehicle distance estimation
- Sports analytics and player tracking
- Surveillance systems
- Mobile robot odometry validation

## License

This is proof-of-concept code for educational and research purposes.
