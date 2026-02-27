/**
 * Type declarations for the wasm-pack generated Wasm module.
 *
 * This stub is used for type-checking during development and CI lint passes
 * before the actual module has been compiled by `wasm-pack build`.
 * The generated `sim_lidar_rs.js` at build time will match these declarations.
 */

/** Sensor configuration mirroring real-world LiDARs. */
export class SensorConfig {
  free(): void;
  constructor(
    horizontal_resolution: number,
    vertical_channels: number,
    vertical_fov_upper: number,
    vertical_fov_lower: number,
    min_range: number,
    max_range: number,
    noise_stddev: number,
  );
  horizontal_resolution: number;
  vertical_channels: number;
  vertical_fov_upper: number;
  vertical_fov_lower: number;
  min_range: number;
  max_range: number;
  noise_stddev: number;
}

/**
 * Streamlined LiDAR simulator for direct JS use.
 * Geometry is loaded separately via `load_geometry`.
 */
export class Simulator {
  free(): void;
  constructor(config: SensorConfig);
  /** Ingest environment geometry and (re)build the internal BVH. */
  load_geometry(vertices: Float32Array, indices: Uint32Array): void;
  /**
   * Run a full scan from the given pose.
   * Returns a zero-copy `Float32Array` view into Wasm linear memory.
   * Copy it immediately before making any further Wasm calls.
   */
  perform_scan(
    x: number,
    y: number,
    z: number,
    qx: number,
    qy: number,
    qz: number,
    qw: number,
  ): Float32Array;
  /** Replace the sensor configuration without rebuilding the BVH. */
  set_config(config: SensorConfig): void;
  /** Returns the number of valid hits from the last scan. */
  last_hit_count(): number;
}

/**
 * LiDAR simulator that accepts geometry at construction time.
 * @deprecated Prefer `Simulator` for more flexible geometry management.
 */
export class LidarSimulator {
  free(): void;
  constructor(vertices: Float32Array, indices: Uint32Array, config: SensorConfig);
  /** Replace the sensor configuration at runtime. */
  set_config(config: SensorConfig): void;
  /** Run a full scan from the given pose. */
  scan(
    px: number,
    py: number,
    pz: number,
    qx: number,
    qy: number,
    qz: number,
    qw: number,
  ): Float32Array;
  /** Returns the hit count from the last scan. */
  last_hit_count(): number;
}

/**
 * Synchronously initialise the Wasm module from pre-fetched bytes.
 * Suitable for Node.js test environments.
 */
export function initSync(input: { module: BufferSource } | BufferSource): void;

/**
 * Asynchronously initialise the Wasm module.
 * Call this once before using any exported classes.
 */
export default function init(
  input?: RequestInfo | URL | Response | BufferSource | WebAssembly.Module,
): Promise<WebAssembly.Exports>;
