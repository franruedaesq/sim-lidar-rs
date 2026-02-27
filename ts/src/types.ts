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
