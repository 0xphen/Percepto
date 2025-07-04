import math
import csv
import sys # Import the sys module to access command-line arguments

# Given elevation angles (radians)
ELEVATION_ANGLES = [
    0.1863, 0.1629, 0.1398, 0.1166, 0.0934, 0.0702, 0.0470, 0.0237,
    0.0005, -0.0227, -0.0459, -0.0692, -0.0924, -0.1156, -0.1388, -0.1620,
    -0.1852, -0.2084, -0.2316, -0.2548, -0.2780, -0.3012, -0.3244, -0.3476,
    -0.3708, -0.3940, -0.4172, -0.4404, -0.4636, -0.4868, -0.5100, -0.5332
]

# Configuration
N_AZIMUTH_RAYS = 3600   # Azimuth steps for rays
# RADIUS will be set based on dense/sparse argument
MARGIN = 0.1            # angular margin (radians)

TARGET_VERTICES = 100000

# Define radii for dense and sparse scenes
DENSE_RADIUS = 1000.0  # Rays (t_max=2000) will hit this
SPARSE_RADIUS = 5000.0 # Rays (t_max=2000) will NOT hit this

# --- Command-line argument parsing ---
if len(sys.argv) != 2:
    print("Usage: python generate_scene.py [dense|sparse]")
    sys.exit(1)

scene_type = sys.argv[1].lower() # Get the argument and convert to lowercase

scene_radius = DENSE_RADIUS # Default to dense
output_filename = 'scene_triangles_dense.csv'

if scene_type == 'dense':
    scene_radius = DENSE_RADIUS
    output_filename = 'scene_triangles_dense.csv'
    print("--- Generating DENSE Scene ---")
elif scene_type == 'sparse':
    scene_radius = SPARSE_RADIUS
    output_filename = 'scene_triangles_sparse.csv'
    print("--- Generating SPARSE Scene ---")
else:
    print(f"Error: Invalid scene type '{scene_type}'. Please use 'dense' or 'sparse'.")
    sys.exit(1)

# To create a "dense" scene and maximize ray intersection probability,
# we want N_AZIMUTH_MESH to be high, ideally matching N_AZIMUTH_RAYS.
N_AZIMUTH_MESH = N_AZIMUTH_RAYS # Set mesh azimuth resolution to match ray azimuth resolution

# Calculate N_ELEV_POINTS to get close to TARGET_VERTICES
# N_ELEV_POINTS refers to the number of discrete elevation points (vertices)
N_ELEV_POINTS = math.floor(TARGET_VERTICES / N_AZIMUTH_MESH)

# Ensure N_ELEV_POINTS is at least 2 to form triangles
if N_ELEV_POINTS < 2:
    N_ELEV_POINTS = 2
    # Recalculate N_AZIMUTH_MESH if N_ELEV_POINTS was too small to hit target_vertices
    # (This path is unlikely to be taken with current TARGET_VERTICES and N_AZIMUTH_RAYS)
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


# Compute elevation range for the mesh
min_elev = min(ELEVATION_ANGLES) - MARGIN
max_elev = max(ELEVATION_ANGLES) + MARGIN

# Build vertices on a spherical band
vertices = []
for i in range(N_AZIMUTH_MESH):
    az = 2 * math.pi * i / N_AZIMUTH_MESH
    for j in range(N_ELEV_POINTS):
        # Scale elevation within the calculated range
        # Use (N_ELEV_POINTS - 1) for division if N_ELEV_POINTS > 1, else 1 to avoid division by zero
        elev_scale_divisor = (N_ELEV_POINTS - 1) if N_ELEV_POINTS > 1 else 1
        elev = min_elev + (max_elev - min_elev) * j / elev_scale_divisor
        
        # Use the dynamically selected scene_radius
        x = scene_radius * math.cos(elev) * math.cos(az)
        y = scene_radius * math.cos(elev) * math.sin(az)
        z = scene_radius * math.sin(elev)
        vertices.append((x, y, z))

# Build triangles with inward-facing winding (CCW when viewed from inside)
triangles = []
# Only generate triangles if there's at least one elevation segment
if N_ELEV_POINTS > 1:
    for i in range(N_AZIMUTH_MESH):
        i_next = (i + 1) % N_AZIMUTH_MESH
        for j in range(N_ELEV_POINTS - 1): # Loop through elevation segments
            idx0 = i * N_ELEV_POINTS + j
            idx1 = i * N_ELEV_POINTS + j + 1
            idx2 = i_next * N_ELEV_POINTS + j
            idx3 = i_next * N_ELEV_POINTS + j + 1

            v0, v1, v2, v3 = vertices[idx0], vertices[idx1], vertices[idx2], vertices[idx3]

            # Triangle 1: (i,j), (i,j+1), (i_next,j) - CCW from inside
            triangles.append((v0, v1, v2))

            # Triangle 2: (i_next,j), (i,j+1), (i_next,j+1) - CCW from inside
            triangles.append((v2, v1, v3))

print(f"Total triangles generated: {len(triangles)}")

# Write the scene triangles out to a CSV file in the specified format
with open(output_filename, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['x0','y0','z0','x1','y1','z1','x2','y2','z2'])
    for (x0,y0,z0), (x1,y1,z1), (x2,y2,z2) in triangles:
        writer.writerow([x0,y0,z0, x1,y1,z1, x2,y2,z2])

print(f"Scene saved to: {output_filename}")

# (Optional) Example ray-casting setup (for your reference, not part of output)
RAY_ORIGIN = (0.0, 0.0, 0.0) # Rays originate just inside the shell to avoid coincident-origin issues

# Generate ray directions for the benchmark (these rays don't change based on scene type)
ray_directions = []
for i in range(N_AZIMUTH_RAYS):
    az_ray = 2 * math.pi * i / N_AZIMUTH_RAYS
    for elev_angle in ELEVATION_ANGLES:
        x_dir = math.cos(elev_angle) * math.cos(az_ray)
        y_dir = math.cos(elev_angle) * math.sin(az_ray)
        z_dir = math.sin(elev_angle)
        ray_directions.append((x_dir, y_dir, z_dir))