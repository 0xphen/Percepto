import math
import csv
import sys
import tomllib # Correct module name for Python 3.11+ for TOML parsing

def load_config(filepath='config.toml'):
    """Loads configuration from a TOML file."""
    try:
        # For tomllib, the file must be opened in binary read mode ("rb")
        with open(filepath, "rb") as f:
            config = tomllib.load(f)
        return config
    except FileNotFoundError:
        print(f"Error: Config file not found at '{filepath}'")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading config file: {e}")
        sys.exit(1)

# --- Load configuration at the start of your script ---
# Assuming config.toml is in the parent directory (project root) relative to 'scripts/'
config = load_config('../config.toml')

# --- Configuration parameters from TOML ---
# LiDAR Sensor
N_AZIMUTH_RAYS = config['LIDAR_SENSOR']['azimuth_steps']
ELEVATION_ANGLES = config['LIDAR_SENSOR']['elevation_angles']

# Scene Generation
TARGET_VERTICES = config['SCENE_GENERATION']['target_vertices']
DENSE_RADIUS = config['SCENE_GENERATION']['dense_radius']
SPARSE_RADIUS = config['SCENE_GENERATION']['sparse_radius']
MARGIN = config['SCENE_GENERATION']['angular_margin']

# Ray Tracer (used by C++, but good to keep consistent for Python's ray generation example)
RAY_T_MIN = config['RAY_TRACER']['ray_t_min']
RAY_T_MAX = config['RAY_TRACER']['ray_t_max']

# --- Rest of your script logic (updated to use config variables) ---

# --- Command-line argument parsing ---
if len(sys.argv) != 2:
    print("Usage: python generate_scene.py [dense|sparse]")
    sys.exit(1)

scene_type = sys.argv[1].lower()

# Default values, will be updated based on scene_type
scene_radius = DENSE_RADIUS
output_filename = 'scenes/dense/scene_triangles_dense.csv' # Default output path

if scene_type == 'dense':
    scene_radius = DENSE_RADIUS
    output_filename = 'scenes/dense/scene_triangles_dense.csv'
    print("--- Generating DENSE Scene ---")
elif scene_type == 'sparse':
    scene_radius = SPARSE_RADIUS
    output_filename = 'scenes/sparse/scene_triangles_sparse.csv'
    print("--- Generating SPARSE Scene ---")
else:
    print(f"Error: Invalid scene type '{scene_type}'. Please use 'dense' or 'sparse'.")
    sys.exit(1)

# N_AZIMUTH_MESH and N_ELEV_POINTS calculation remains the same, but uses N_AZIMUTH_RAYS from config
N_AZIMUTH_MESH = N_AZIMUTH_RAYS
N_ELEV_POINTS = math.floor(TARGET_VERTICES / N_AZIMUTH_MESH)
if N_ELEV_POINTS < 2:
    N_ELEV_POINTS = 2
    # This recalculation is less likely to be hit with current TARGET_VERTICES and N_AZIMUTH_RAYS
    if N_AZIMUTH_MESH * N_ELEV_POINTS < TARGET_VERTICES:
        N_AZIMUTH_MESH = math.floor(TARGET_VERTICES / N_ELEV_POINTS)

ACTUAL_TOTAL_VERTICES = N_AZIMUTH_MESH * N_ELEV_POINTS

print(f"Scene Type Selected: {scene_type.upper()}")
print(f"Target vertices for scene: {TARGET_VERTICES}")
print(f"Calculated N_AZIMUTH_MESH (points): {N_AZIMUTH_MESH}")
print(f"Calculated N_ELEV_POINTS (points): {N_ELEV_POINTS}")
print(f"Actual total vertices generated: {ACTUAL_TOTAL_VERTICES}")
print(f"Number of elevation bands (segments): {N_ELEV_POINTS - 1}")
print(f"Scene Geometry Radius: {scene_radius} units")

# Compute elevation range for the mesh (uses ELEVATION_ANGLES and MARGIN from config)
min_elev = min(ELEVATION_ANGLES) - MARGIN
max_elev = max(ELEVATION_ANGLES) + MARGIN

# Build vertices on a spherical band
vertices = []
for i in range(N_AZIMUTH_MESH):
    az = 2 * math.pi * i / N_AZIMUTH_MESH
    for j in range(N_ELEV_POINTS):
        elev_scale_divisor = (N_ELEV_POINTS - 1) if N_ELEV_POINTS > 1 else 1
        elev = min_elev + (max_elev - min_elev) * j / elev_scale_divisor
        
        x = scene_radius * math.cos(elev) * math.cos(az)
        y = scene_radius * math.cos(elev) * math.sin(az)
        z = scene_radius * math.sin(elev)
        vertices.append((x, y, z))

# Build triangles with inward-facing winding (CCW when viewed from inside)
triangles = []
if N_ELEV_POINTS > 1:
    for i in range(N_AZIMUTH_MESH):
        i_next = (i + 1) % N_AZIMUTH_MESH
        for j in range(N_ELEV_POINTS - 1):
            idx0 = i * N_ELEV_POINTS + j
            idx1 = i * N_ELEV_POINTS + j + 1
            idx2 = i_next * N_ELEV_POINTS + j
            idx3 = i_next * N_ELEV_POINTS + j + 1

            v0, v1, v2, v3 = vertices[idx0], vertices[idx1], vertices[idx2], vertices[idx3]

            triangles.append((v0, v1, v2))
            triangles.append((v2, v1, v3))

print(f"Total triangles generated: {len(triangles)}")

# Ensure the 'scenes/dense' and 'scenes/sparse' directories exist
import os
output_dir = os.path.dirname(output_filename)
os.makedirs(output_dir, exist_ok=True)

with open(output_filename, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['x0','y0','z0','x1','y1','z1','x2','y2','z2'])
    for (x0,y0,z0), (x1,y1,z1), (x2,y2,z2) in triangles:
        writer.writerow([x0,y0,z0, x1,y1,z1, x2,y2,z2])

print(f"Scene saved to: {output_filename}")

# --- Ray generation for reference (uses config values now) ---
RAY_ORIGIN = (0.0, 0.0, 0.0) # Rays originate just inside the shell to avoid coincident-origin issues

ray_directions = []
for i in range(N_AZIMUTH_RAYS):
    az_ray = 2 * math.pi * i / N_AZIMUTH_RAYS
    for elev_angle in ELEVATION_ANGLES:
        x_dir = math.cos(elev_angle) * math.cos(az_ray)
        y_dir = math.cos(elev_angle) * math.sin(az_ray)
        z_dir = math.sin(elev_angle)
        ray_directions.append((x_dir, y_dir, z_dir))