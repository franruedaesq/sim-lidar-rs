# sim-lidar-rs – Architecture & Product Documentation

## General Description

**sim-lidar-rs** is a high-performance simulated LiDAR sensor library built for the web. Physical LiDAR sensors fire hundreds of thousands of lasers per second to build a 3D "Point Cloud" of their environment. Doing this inside a JavaScript 3D engine (like Three.js) via standard raycasting completely locks the main thread, making browser-based robotics simulation impossible.

This library solves this by shifting the computation to Rust. It ingests 3D environment data (triangle meshes), builds a highly optimised spatial index (Bounding Volume Hierarchy – BVH), and calculates thousands of ray intersections simultaneously. It compiles to WebAssembly (Wasm) and exposes a professional, strictly typed TypeScript API. The output is a zero-copy `Float32Array` representing the exact coordinates of the laser hits, ready to be digested by ROS (Robot Operating System) nodes or visualised instantly in the browser.

---

## 1. Core Objectives

| Objective | Description |
|-----------|-------------|
| **Performance** | Capable of calculating 100,000+ ray intersections per frame at 60 FPS without blocking the main browser UI thread. |
| **DX (Developer Experience)** | An intuitive, framework-agnostic TypeScript API that plays perfectly with Three.js, Babylon.js, or raw WebGL/WebGPU. |
| **Reliability** | Strict memory management across the JS/Wasm boundary to prevent memory leaks. Extensive test coverage guaranteeing mathematical accuracy. |
| **Portability** | Distributed as a standard npm package with Wasm bundles included. |

---

## 2. Technical Architecture

### Overview

```
┌──────────────────────────────────────────────────────────────┐
│  Browser Main Thread                                          │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  TypeScript Client API  (LidarClient)                  │  │
│  │  • postMessage(init | scan | setConfig)                │  │
│  │  • Receives transferable Float32Array of hit points    │  │
│  └──────────────────────────┬─────────────────────────────┘  │
│                             │  Web Worker boundary            │
│  ┌──────────────────────────▼─────────────────────────────┐  │
│  │  Web Worker  (worker.ts)                               │  │
│  │  • Instantiates Wasm module                            │  │
│  │  • Owns LidarSimulator (Rust/Wasm object)              │  │
│  └──────────────────────────┬─────────────────────────────┘  │
│                             │  Wasm boundary                  │
│  ┌──────────────────────────▼─────────────────────────────┐  │
│  │  Core Engine  (Rust / WebAssembly)                     │  │
│  │  ┌──────────┐  ┌─────────────┐  ┌──────────────────┐  │  │
│  │  │   BVH    │  │   Sensor    │  │   Raycaster      │  │  │
│  │  │  builder │  │   config    │  │   (Möller-       │  │  │
│  │  │  (SAH-   │  │  + ray-dir  │  │   Trumbore +     │  │  │
│  │  │  median) │  │  generator  │  │   Gaussian noise)│  │  │
│  │  └──────────┘  └─────────────┘  └──────────────────┘  │  │
│  └────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────┘
```

### Components

#### Core Engine (Rust)
Handles all 3D mathematics:
- **BVH (`src/bvh.rs`)** – Constructs a Bounding Volume Hierarchy from raw vertex/index data using median-axis splitting. Provides fast ray-AABB and ray-triangle (Möller–Trumbore) intersection tests.
- **Sensor (`src/sensor.rs`)** – Encapsulates sensor parameters and generates spherical ray direction vectors for a full scan given a sensor pose.
- **Raycaster (`src/raycaster.rs`)** – Executes the scan loop: for each ray direction, traverses the BVH, filters by range limits, optionally applies Gaussian noise, and collects world-space hit coordinates.

#### Bridge (wasm-bindgen)
Exposes `LidarSimulator` and `SensorConfig` Rust structs to JavaScript. Returns `Vec<f32>` (mapped to `Float32Array`) directly from Wasm memory to avoid expensive data copies.

#### Concurrency Wrapper (TypeScript / Web Workers)
`LidarClient` (`ts/src/index.ts`) automatically spawns the Wasm module inside a Web Worker, keeping the main thread free. The heavy `scan()` call is non-blocking and returns a `Promise<ScanResult>`. Hit-point buffers are transferred (zero-copy) from the worker to the caller via `Transferable`.

### Data Flow

1. **Initialisation** – JS passes static environment geometry (flat `Float32Array` of vertices + `Uint32Array` of indices) and a `SensorConfig` to the worker. The worker builds the BVH inside Wasm.
2. **Simulation Loop** – JS calls `lidar.scan(pose)` with the sensor's current `Position` and `Rotation`. Inside Wasm, ray trajectories are computed for every channel × azimuth step, BVH traversal finds intersections, and results are written to a buffer.
3. **Extraction** – The hit buffer is transferred back to the main thread as a `Float32Array` view—zero copies.

---

## 3. Sensor Configuration (The API Surface)

The sensor is configurable to mimic real-world LiDARs.

### `SensorConfig` fields

| Field | Type | Description |
|-------|------|-------------|
| `horizontalResolution` | `number` | Number of rays per full horizontal sweep (360°). |
| `verticalChannels` | `number` | Number of vertical laser rings (e.g. 16, 32, 64). |
| `verticalFovUpper` | `number` | Upper vertical FOV limit in degrees (e.g. +15 for VLP-16). |
| `verticalFovLower` | `number` | Lower vertical FOV limit in degrees (e.g. -15 for VLP-16). |
| `minRange` | `number` | Minimum valid range in metres. Hits closer than this are discarded. |
| `maxRange` | `number` | Maximum valid range in metres. Rays beyond this are considered misses. |
| `noiseStddev` | `number` | Standard deviation (metres) of Gaussian noise applied to each hit distance. Set to `0` to disable. |

### Built-in Presets

```ts
import { VLP16_CONFIG, OUSTER_OS1_32_CONFIG, OUSTER_OS1_64_CONFIG } from 'sim-lidar-rs';
```

| Preset | Channels | H-Res | V-FOV | Max Range |
|--------|----------|-------|-------|-----------|
| `VLP16_CONFIG` | 16 | 1800 | ±15° | 100 m |
| `OUSTER_OS1_32_CONFIG` | 32 | 1024 | ±22.5° | 120 m |
| `OUSTER_OS1_64_CONFIG` | 64 | 2048 | ±22.5° | 120 m |

### Quick-Start Example

```ts
import { LidarClient, VLP16_CONFIG } from 'sim-lidar-rs';

// 1. Build the environment geometry (e.g. from Three.js BufferGeometry)
const vertices = new Float32Array([/* x,y,z ... */]);
const indices  = new Uint32Array([/* i0,i1,i2 ... */]);

// 2. Create the client – BVH is built inside the Web Worker
const lidar = new LidarClient(vertices, indices, VLP16_CONFIG);
await lidar.ready();

// 3. Fire a scan every animation frame
function animate() {
  requestAnimationFrame(animate);
  lidar.scan({
    position: { x: 0, y: 1.5, z: 0 },
    rotation: { x: 0,   y: 0,   z: 0, w: 1 },
  }).then(({ hits, hitCount }) => {
    // hits is a transferable Float32Array [x,y,z, x,y,z, ...]
    console.log(`Got ${hitCount} hits`);
  });
}
animate();

// 4. Clean up when done
lidar.dispose();
```
