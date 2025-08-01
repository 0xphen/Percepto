# Percepto: High-Performance LiDAR Ray Tracer Simulator

**Percepto** is a C++-based LiDAR ray tracing simulator designed to simulate realistic LiDAR scans using triangle meshes. It is built for robotics applications requiring high-performance, customizable scanning simulations, and will integrate cleanly with real-world SLAM, perception, and visualization pipelines.

---

## ✅ Current Features (Foundation Layer)

- Triangle loading from CSV
- Azimuth and elevation-based ray scanning
- Basic ray-triangle intersection using Möller–Trumbore
- Simple scene parsing and console output

This forms a solid MVP for custom scan simulation.

---

## 🔁 Next Steps – Prioritized Roadmap

A structured roadmap to scale Percepto into a production-grade tool:

✅### 1. 🔹 Add Benchmarking (First)

**Why:** Establish baseline performance metrics before optimizing.

**Tasks:**
- Measure total scan time per frame
- Measure per-ray intersection cost
- Track rays per second (RPS)
- Log memory usage

🛠 Use `std::chrono`, Google Benchmark, or custom timers.

---

### 2. ⚙️ Parallelize the Scan Loop

**Why:** Intersection tests are highly parallelizable.

**Tasks:**
- Use `std::thread`, `std::async`, or `std::execution::par_unseq`
- Divide work by azimuth steps or elevation rows

💡 Start simple and scale up.

---
### 3a. ☁️ Log structured scan summary
**Tasks:**
- Print scene name, hits, runtime, scan stats
- Prepare for future automated regression testing
Example:
[✔] Simulation Complete
  ├─ Scene:            warehouse.csv
  ├─ Rays Fired:       128000
  ├─ Hits Detected:    38290
  ├─ Total Runtime:    82.5 ms
  ├─ Rays/sec:         1.55 million
  ├─ Avg Ray Time:     0.64 µs
  ├─ Elevation Angles: [-15, -10, ..., 15]
  └─ Azimuth Steps:    512


### 3b. ☁️ Output Point Clouds (e.g. CSV, PCD)

**Why:** Enables output for perception pipelines (PCL, ROS, etc).

**Tasks:**
- Output XYZ points (with optional intensity)
- Support formats: CSV (first), PCD, and optionally .PLY
- Include scan metadata or timestamps

---

### 4. 🧠 SIMD-Accelerate Ray Intersections

**Why:** Ray math (dot/cross products) can benefit from vectorization.

**Tasks:**
- Use SIMD intrinsics (`<immintrin.h>`) or libraries like xsimd/Eigen
- Benchmark before and after

📈 Watch for correctness and actual speed gains.

---

### 5. 🌲 Add a BVH Acceleration Structure

**Why:** Reduces intersection cost from O(n) to O(log n).

**Tasks:**
- Compute AABBs per triangle
- Build BVH using midpoint/split heuristics
- Traverse BVH per ray

📌 Combine with SIMD and multithreading for maximum gain.

---

### 6. 📦 Load .OBJ Files

**Why:** OBJ is standard for 3D mesh data in robotics and graphics.

**Tasks:**
- Load vertices and faces from `.obj` files
- Replace CSV loader or support both

🛠 Use [tinyobjloader](https://github.com/syoyo/tinyobjloader).

---

### 7. 🚀 Custom Allocator for Optimization

**Why:** For better control over memory allocation in hot paths.

**Tasks:**
- Implement or integrate a bump/pool allocator
- Use for BVH nodes, ray data, etc.

📉 Delay if memory performance is not yet a bottleneck.

---

## 🧪 Bonus / Experimental Features

Once the above is complete, consider:

- Simulating sensor noise (dropout, Gaussian jitter)
- Adding occlusion and self-shadowing
- ROS2 integration to publish point clouds
- Visualizing results in RViz or Meshlab

---

## ✅ Immediate Next 3 Tasks

1. **Add benchmarking tools**  
   Measure and log all scan performance metrics.

2. **Parallelize the core scan loop**  
   Use `std::thread` or `std::async` to split work by azimuth.

3. **Output basic point cloud in CSV format**  
   Allows testing in external tools like PCL or Meshlab.

---

## 📌 Goal

> Build a **production-grade**, **high-performance**, and **ROS-compatible** LiDAR simulation tool for robotics and perception research.

---
