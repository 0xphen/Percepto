import numpy as np
import math
import random
import csv

# Constants
AZIMUTH_STEPS = 3600
ELEVATION_ANGLES = [
    +10.67, +9.34, +8.01, +6.68, +5.35, +4.02, +2.69, +1.36, +0.03, -1.30, -2.63,
    -3.96, -5.29, -6.62, -7.95, -9.28, -10.61, -11.94, -13.27, -14.60, -15.93, -17.26,
    -18.59, -19.92, -21.25, -22.58, -23.91, -25.24, -26.57, -27.90, -29.23, -30.56
]
NUM_TRIANGLES = 100000
RADIUS = 1.0

def spherical_to_cartesian(azimuth, elevation, radius=RADIUS):
    """Convert spherical coordinates to Cartesian coordinates"""
    x = radius * math.cos(elevation) * math.cos(azimuth)
    y = radius * math.cos(elevation) * math.sin(azimuth)
    z = radius * math.sin(elevation)
    return np.array([x, y, z])

# Generate azimuth angles
azimuth_angles = np.linspace(0, 2 * np.pi, AZIMUTH_STEPS, endpoint=False)

# Generate triangles
triangles = []
for _ in range(NUM_TRIANGLES):
    # Randomly select ray direction
    az_idx = random.randint(0, AZIMUTH_STEPS - 1)
    el_idx = random.randint(0, len(ELEVATION_ANGLES) - 1)
    
    # Center point (v0)
    az = azimuth_angles[az_idx]
    el_rad = math.radians(ELEVATION_ANGLES[el_idx])
    v0 = spherical_to_cartesian(az, el_rad)
    
    # Next azimuth point (v1)
    next_az_idx = (az_idx + 1) % AZIMUTH_STEPS
    next_az = azimuth_angles[next_az_idx]
    v1 = spherical_to_cartesian(next_az, el_rad)
    
    # Next elevation point (v2)
    if el_idx < len(ELEVATION_ANGLES) - 1:
        next_el_rad = math.radians(ELEVATION_ANGLES[el_idx + 1])
    else:
        next_el_rad = el_rad + math.radians(1.0)  # Small offset for last elevation
    
    v2 = spherical_to_cartesian(az, next_el_rad)
    
    # Ensure front-facing orientation
    edge1 = v1 - v0
    edge2 = v2 - v0
    normal = np.cross(edge1, edge2)
    if np.dot(normal, v0) < 0:
        v1, v2 = v2, v1  # Swap vertices to flip normal
    
    triangles.append(np.concatenate([v0, v1, v2]))

# Save to CSV
with open("triangles.csv", "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["x0", "y0", "z0", "x1", "y1", "z1", "x2", "y2", "z2"])
    writer.writerows(triangles)
