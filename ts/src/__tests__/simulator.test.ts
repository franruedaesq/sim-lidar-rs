/**
 * Node-based integration test for the Wasm `Simulator` class.
 *
 * Loads the compiled `.wasm` binary synchronously, exercises `load_geometry`
 * (zero-copy ingestion) and `perform_scan` (zero-copy output), and verifies
 * the returned `Float32Array` contains geometrically correct hit points.
 */
import { describe, it, expect, beforeAll } from "vitest";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { join, dirname } from "node:path";
import { initSync, Simulator, SensorConfig } from "../../wasm/sim_lidar_rs.js";

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

// Initialise the Wasm module once before any test runs.
beforeAll(() => {
  const wasmPath = join(__dirname, "../../wasm/sim_lidar_rs_bg.wasm");
  const wasmBytes = readFileSync(wasmPath);
  initSync({ module: wasmBytes });
});

describe("Simulator – WebAssembly bindings", () => {
  /**
   * A 20 × 20 ground plane at y = 0 (two triangles).
   * The sensor is placed 1 m above at (0, 1, 0) with identity rotation.
   * A downward-only FOV (-10° … -20°) guarantees every ray intersects the plane.
   */
  it("load_geometry + perform_scan returns a Float32Array with all hits on y=0 plane", () => {
    // 20×20 ground plane at y = 0
    const vertices = new Float32Array([
      -10.0, 0.0, -10.0,
       10.0, 0.0, -10.0,
       10.0, 0.0,  10.0,
      -10.0, 0.0,  10.0,
    ]);
    const indices = new Uint32Array([0, 1, 2, 0, 2, 3]);

    // Downward-only config: 36 horizontal × 4 vertical → 144 rays total.
    // All rays are aimed between -10° and -20° elevation so every ray hits
    // the plane within the 20 m max range.
    const config = new SensorConfig(
      36,    // horizontal_resolution
      4,     // vertical_channels
      -10.0, // vertical_fov_upper (negative = below horizon)
      -20.0, // vertical_fov_lower
      0.1,   // min_range (metres)
      20.0,  // max_range (metres)
      0.0    // noise_stddev (disabled)
    );

    const sim = new Simulator(config);
    sim.load_geometry(vertices, indices);

    // Identity quaternion (0, 0, 0, 1): no rotation applied
    const hits = sim.perform_scan(0, 1, 0, 0, 0, 0, 1);

    // Copy immediately so subsequent Wasm calls don't invalidate the view
    const pts = hits.slice();

    expect(pts).toBeInstanceOf(Float32Array);
    // Every ray should hit the plane: 36 × 4 = 144 hits → 432 floats
    expect(pts.length).toBe(144 * 3);
    expect(pts.length % 3).toBe(0);

    // Every hit point must lie on the ground plane (y ≈ 0)
    for (let i = 1; i < pts.length; i += 3) {
      expect(Math.abs(pts[i])).toBeLessThan(0.01);
    }

    // Hit points should be spread around the sensor in X and Z
    const xs = Array.from({ length: pts.length / 3 }, (_, k) => pts[k * 3]);
    const minX = Math.min(...xs);
    const maxX = Math.max(...xs);
    expect(maxX - minX).toBeGreaterThan(1.0);

    sim.free();
  });

  it("last_hit_count matches hits.length / 3", () => {
    const vertices = new Float32Array([
      -10.0, 0.0, -10.0,
       10.0, 0.0, -10.0,
       10.0, 0.0,  10.0,
      -10.0, 0.0,  10.0,
    ]);
    const indices = new Uint32Array([0, 1, 2, 0, 2, 3]);
    const config = new SensorConfig(36, 4, -10.0, -20.0, 0.1, 20.0, 0.0);

    const sim = new Simulator(config);
    sim.load_geometry(vertices, indices);

    const hits = sim.perform_scan(0, 1, 0, 0, 0, 0, 1);
    const count = sim.last_hit_count();

    expect(count).toBe(hits.length / 3);

    sim.free();
  });

  it("load_geometry can be called multiple times to swap geometry", () => {
    // First geometry: ground plane at y=0
    const v1 = new Float32Array([
      -10.0, 0.0, -10.0,
       10.0, 0.0, -10.0,
       10.0, 0.0,  10.0,
      -10.0, 0.0,  10.0,
    ]);
    const i1 = new Uint32Array([0, 1, 2, 0, 2, 3]);

    // Second geometry: elevated plane at y=0.5 (closer to sensor at y=1)
    const v2 = new Float32Array([
      -10.0, 0.5, -10.0,
       10.0, 0.5, -10.0,
       10.0, 0.5,  10.0,
      -10.0, 0.5,  10.0,
    ]);
    const i2 = new Uint32Array([0, 1, 2, 0, 2, 3]);

    const config = new SensorConfig(36, 4, -10.0, -20.0, 0.1, 20.0, 0.0);
    const sim = new Simulator(config);

    // First scan: hits on y=0 plane
    sim.load_geometry(v1, i1);
    const hits1 = sim.perform_scan(0, 1, 0, 0, 0, 0, 1).slice();

    // Second scan: hits on y=0.5 plane (shorter range)
    sim.load_geometry(v2, i2);
    const hits2 = sim.perform_scan(0, 1, 0, 0, 0, 0, 1).slice();

    // Both scans must have the same number of hits
    expect(hits2.length).toBe(hits1.length);

    // y-coordinates of all hits in scan 2 must be ~0.5
    for (let i = 1; i < hits2.length; i += 3) {
      expect(Math.abs(hits2[i] - 0.5)).toBeLessThan(0.01);
    }

    // y-coordinates of scan 1 must be ~0
    for (let i = 1; i < hits1.length; i += 3) {
      expect(Math.abs(hits1[i])).toBeLessThan(0.01);
    }

    sim.free();
  });
});
