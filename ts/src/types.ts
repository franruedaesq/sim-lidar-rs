/**
 * Sensor configuration mirroring real-world LiDARs (e.g. Velodyne VLP-16, Ouster).
 */
export interface SensorConfig {
  /** Number of rays per full horizontal sweep (360°). */
  horizontalResolution: number;
  /** Number of vertical laser rings (e.g. 16, 32, 64). */
  verticalChannels: number;
  /** Upper vertical FOV limit in degrees (e.g. +15 for VLP-16). */
  verticalFovUpper: number;
  /** Lower vertical FOV limit in degrees (e.g. -15 for VLP-16). */
  verticalFovLower: number;
  /** Minimum valid range in metres. */
  minRange: number;
  /** Maximum valid range in metres. */
  maxRange: number;
  /** Standard deviation of Gaussian noise added to range measurements. 0 = disabled. */
  noiseStddev: number;
}

/**
 * Sensor pose: position + orientation.
 */
export interface Pose {
  /** World-space sensor origin. */
  position: { x: number; y: number; z: number };
  /**
   * Sensor orientation as a unit quaternion.
   * Defaults to identity `{ x:0, y:0, z:0, w:1 }` if omitted.
   */
  rotation?: { x: number; y: number; z: number; w: number };
}

/**
 * Result of a single LiDAR scan.
 */
export interface ScanResult {
  /**
   * Flat `Float32Array` of hit world-space coordinates: `[x,y,z, x,y,z, …]`.
   * Length is `hitCount * 3`.
   */
  hits: Float32Array;
  /** Number of valid hits returned. */
  hitCount: number;
}

/** VLP-16 preset. */
export const VLP16_CONFIG: Readonly<SensorConfig> = {
  horizontalResolution: 1800,
  verticalChannels: 16,
  verticalFovUpper: 15,
  verticalFovLower: -15,
  minRange: 0.1,
  maxRange: 100,
  noiseStddev: 0,
};

/** Ouster OS1-32 preset. */
export const OUSTER_OS1_32_CONFIG: Readonly<SensorConfig> = {
  horizontalResolution: 1024,
  verticalChannels: 32,
  verticalFovUpper: 22.5,
  verticalFovLower: -22.5,
  minRange: 0.1,
  maxRange: 120,
  noiseStddev: 0,
};

/** Ouster OS1-64 preset. */
export const OUSTER_OS1_64_CONFIG: Readonly<SensorConfig> = {
  horizontalResolution: 2048,
  verticalChannels: 64,
  verticalFovUpper: 22.5,
  verticalFovLower: -22.5,
  minRange: 0.1,
  maxRange: 120,
  noiseStddev: 0,
};

/**
 * Returns the total number of rays fired per scan for a given config.
 */
export function totalRays(config: SensorConfig): number {
  return config.horizontalResolution * config.verticalChannels;
}

/**
 * Environment geometry consumed by {@link SimLidar.updateEnvironment}.
 */
export interface Geometry {
  /** Flat array of vertex positions `[x,y,z, …]`. */
  vertices: Float32Array;
  /** Flat array of triangle vertex indices. */
  indices: Uint32Array;
}

// ─── Custom Error classes ─────────────────────────────────────────────────────

/**
 * Base error class for all sim-lidar-rs errors.
 * Exported so consumers can use `err instanceof SimLidarError` to distinguish
 * library errors from other runtime errors.
 */
export class SimLidarError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "SimLidarError";
    // Restore prototype chain for subclass `instanceof` checks in transpiled environments.
    Object.setPrototypeOf(this, new.target.prototype);
  }
}

/**
 * Thrown when a scan or environment update is requested before the simulator
 * has been initialised (i.e. before `init()` or `ready()` has resolved).
 */
export class SimLidarNotInitializedError extends SimLidarError {
  constructor() {
    super("Simulator not initialised. Await init() / ready() before calling scan().");
    this.name = "SimLidarNotInitializedError";
    Object.setPrototypeOf(this, new.target.prototype);
  }
}

/**
 * Thrown when any method is called on a simulator instance that has already
 * been disposed via `destroy()` or `dispose()`.
 */
export class SimLidarDisposedError extends SimLidarError {
  constructor() {
    super("This SimLidar instance has been destroyed and can no longer be used.");
    this.name = "SimLidarDisposedError";
    Object.setPrototypeOf(this, new.target.prototype);
  }
}

// ─── Observability ───────────────────────────────────────────────────────────

/**
 * Optional event callbacks that can be passed to {@link SimLidar} and
 * {@link LidarClient} constructors for observability and debugging.
 */
export interface SimLidarEventHandlers {
  /**
   * Called once the Wasm module has been loaded and the simulator is ready.
   * Equivalent to awaiting `init()` / `ready()`.
   */
  onReady?: () => void;
  /**
   * Called whenever a worker-level error occurs (e.g. Wasm panic, unhandled
   * exception inside the worker). Provides the error before it propagates to
   * pending Promise rejections.
   */
  onError?: (err: SimLidarError) => void;
}
