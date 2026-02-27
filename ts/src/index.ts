/**
 * sim-lidar-rs – Public TypeScript API
 *
 * Usage:
 * ```ts
 * import { LidarClient, VLP16_CONFIG } from 'sim-lidar-rs';
 *
 * const lidar = new LidarClient(vertices, indices, VLP16_CONFIG);
 * await lidar.ready();
 * const { hits, hitCount } = await lidar.scan({ position: { x: 0, y: 1, z: 0 } });
 * ```
 */

export type { SensorConfig, Pose, ScanResult, Geometry, SimLidarEventHandlers } from "./types.js";
export {
  VLP16_CONFIG,
  OUSTER_OS1_32_CONFIG,
  OUSTER_OS1_64_CONFIG,
  totalRays,
  SimLidarError,
  SimLidarNotInitializedError,
  SimLidarDisposedError,
} from "./types.js";

import type { SensorConfig, Pose, ScanResult, Geometry, SimLidarEventHandlers } from "./types.js";
import { SimLidarError, SimLidarDisposedError } from "./types.js";

/**
 * `LidarClient` wraps the Wasm LiDAR simulator running inside a Web Worker.
 * All heavy computation runs off the main thread.
 */
export class LidarClient {
  private worker: Worker;
  private pending = new Map<string, { resolve: (v: unknown) => void; reject: (e: Error) => void }>();
  private readyPromise: Promise<void>;
  private _scanCounter = 0;
  private _disposed = false;
  private readonly _handlers: SimLidarEventHandlers;

  constructor(
    vertices: Float32Array,
    indices: Uint32Array,
    config: SensorConfig,
    /** Optional URL/path to the worker script. Defaults to the bundled worker. */
    workerUrl?: string | URL,
    /** Optional observability callbacks. */
    handlers?: SimLidarEventHandlers
  ) {
    this._handlers = handlers ?? {};
    const url = workerUrl ?? new URL("./worker.js", import.meta.url);
    this.worker = new Worker(url, { type: "module" });
    this.worker.addEventListener("message", this._onMessage.bind(this));

    this.readyPromise = new Promise<void>((resolve, reject) => {
      this.pending.set("__ready__", {
        resolve: resolve as (v: unknown) => void,
        reject,
      });
    });

    // Transfer vertex and index buffers to avoid copying
    const verticesCopy = new Float32Array(vertices);
    const indicesCopy = new Uint32Array(indices);
    this.worker.postMessage(
      { type: "init", vertices: verticesCopy, indices: indicesCopy, config },
      [verticesCopy.buffer, indicesCopy.buffer]
    );
  }

  /** Resolves once the Wasm module has been initialised and the BVH built. */
  ready(): Promise<void> {
    return this.readyPromise;
  }

  /** Update the sensor configuration without rebuilding the BVH. */
  setConfig(config: SensorConfig): void {
    if (this._disposed) throw new SimLidarDisposedError();
    this.worker.postMessage({ type: "setConfig", config });
  }

  /** Run a full scan and return the resulting point cloud. */
  scan(pose: Pose): Promise<ScanResult> {
    if (this._disposed) return Promise.reject(new SimLidarDisposedError());
    return new Promise<ScanResult>((resolve, reject) => {
      const id = `scan_${++this._scanCounter}`;
      this.pending.set(id, {
        resolve: resolve as (v: unknown) => void,
        reject,
      });
      this.worker.postMessage({ type: "scan", pose, __id: id });
    });
  }

  /** Terminate the underlying Web Worker, rejecting any in-flight scan Promises. */
  dispose(): void {
    if (this._disposed) return;
    this._disposed = true;
    const err = new SimLidarDisposedError();
    for (const entry of this.pending.values()) {
      entry.reject(err);
    }
    this.pending.clear();
    this.worker.terminate();
  }

  private _onMessage(evt: MessageEvent): void {
    const msg = evt.data as { type: string; [k: string]: unknown };

    if (msg.type === "ready") {
      const entry = this.pending.get("__ready__");
      if (entry) {
        this.pending.delete("__ready__");
        entry.resolve(undefined);
        this._handlers.onReady?.();
      }
      return;
    }

    if (msg.type === "scan") {
      const id = (msg as { __id?: string }).__id;
      const entry = id ? this.pending.get(id) : undefined;
      if (entry) {
        this.pending.delete(id!);
        entry.resolve({ hits: msg.hits, hitCount: msg.hitCount });
      }
      return;
    }

    if (msg.type === "error") {
      const err = new SimLidarError(msg.message as string);
      this._handlers.onError?.(err);
      for (const entry of this.pending.values()) {
        entry.reject(err);
      }
      this.pending.clear();
    }
  }
}

/**
 * `SimLidar` is the primary TypeScript API for the sim-lidar-rs library.
 *
 * It abstracts the Web Worker communication using Promises and provides
 * explicit lifecycle management to ensure Wasm instances are freed when
 * the simulator is no longer needed.
 *
 * ```ts
 * const lidar = new SimLidar(VLP16_CONFIG);
 * await lidar.init();
 * await lidar.updateEnvironment({ vertices, indices });
 * const hits = await lidar.scan({ position: { x: 0, y: 1, z: 0 } });
 * // hits is a Float32Array [x,y,z, x,y,z, …]
 * lidar.destroy();
 * ```
 */
export class SimLidar {
  private worker: Worker;
  private pending = new Map<
    string,
    { resolve: (v: unknown) => void; reject: (e: Error) => void }
  >();
  private _counter = 0;
  private readonly _config: SensorConfig;
  private _disposed = false;
  private readonly _handlers: SimLidarEventHandlers;

  constructor(
    config: SensorConfig,
    /** Optional URL/path to the worker script. Defaults to the bundled worker. */
    workerUrl?: string | URL,
    /** Optional observability callbacks. */
    handlers?: SimLidarEventHandlers
  ) {
    this._config = config;
    this._handlers = handlers ?? {};
    const url = workerUrl ?? new URL("./worker.js", import.meta.url);
    this.worker = new Worker(url, { type: "module" });
    this.worker.addEventListener("message", this._onMessage.bind(this));
  }

  /**
   * Initialise the Wasm module inside the Web Worker.
   * Must be called before {@link updateEnvironment} or {@link scan}.
   */
  init(): Promise<void> {
    if (this._disposed) return Promise.reject(new SimLidarDisposedError());
    return new Promise<void>((resolve, reject) => {
      this.pending.set("__ready__", {
        resolve: resolve as (v: unknown) => void,
        reject,
      });
      this.worker.postMessage({ type: "init", config: this._config });
    });
  }

  /**
   * Load (or replace) the environment geometry and rebuild the BVH.
   * Buffers are transferred to the worker to avoid copying.
   */
  updateEnvironment(geometry: Geometry): Promise<void> {
    if (this._disposed) return Promise.reject(new SimLidarDisposedError());
    return new Promise<void>((resolve, reject) => {
      const id = `env_${++this._counter}`;
      this.pending.set(id, {
        resolve: resolve as (v: unknown) => void,
        reject,
      });
      const verticesCopy = new Float32Array(geometry.vertices);
      const indicesCopy = new Uint32Array(geometry.indices);
      this.worker.postMessage(
        { type: "updateEnvironment", vertices: verticesCopy, indices: indicesCopy, __id: id },
        [verticesCopy.buffer, indicesCopy.buffer]
      );
    });
  }

  /**
   * Run a full LiDAR scan from the given pose.
   *
   * @returns A `Promise` that resolves with a `Float32Array` of world-space
   *   hit coordinates `[x,y,z, x,y,z, …]`.
   */
  scan(pose: Pose): Promise<Float32Array> {
    if (this._disposed) return Promise.reject(new SimLidarDisposedError());
    return new Promise<Float32Array>((resolve, reject) => {
      const id = `scan_${++this._counter}`;
      this.pending.set(id, {
        resolve: resolve as (v: unknown) => void,
        reject,
      });
      this.worker.postMessage({ type: "scan", pose, __id: id });
    });
  }

  /**
   * Destroy the simulator: asks the worker to explicitly free the Wasm
   * instance (releasing linear memory) and then terminates the worker thread.
   *
   * Any in-flight Promises are rejected with a {@link SimLidarDisposedError}.
   */
  destroy(): void {
    if (this._disposed) return;
    this._disposed = true;
    const err = new SimLidarDisposedError();
    for (const entry of this.pending.values()) {
      entry.reject(err);
    }
    this.pending.clear();
    // Ask the worker to call simulator.free() before we terminate it.
    this.worker.postMessage({ type: "destroy" });
  }

  private _onMessage(evt: MessageEvent): void {
    const msg = evt.data as { type: string; [k: string]: unknown };

    if (msg.type === "ready") {
      const entry = this.pending.get("__ready__");
      if (entry) {
        this.pending.delete("__ready__");
        entry.resolve(undefined);
        this._handlers.onReady?.();
      }
      return;
    }

    if (msg.type === "environmentUpdated") {
      const id = msg.__id as string | undefined;
      if (id) {
        const entry = this.pending.get(id);
        if (entry) {
          this.pending.delete(id);
          entry.resolve(undefined);
        }
      }
      return;
    }

    if (msg.type === "scan") {
      const id = msg.__id as string | undefined;
      if (id) {
        const entry = this.pending.get(id);
        if (entry) {
          this.pending.delete(id);
          entry.resolve(msg.hits as Float32Array);
        }
      }
      return;
    }

    if (msg.type === "destroyed") {
      // Worker has freed Wasm memory; now it is safe to terminate.
      this.worker.terminate();
      return;
    }

    if (msg.type === "error") {
      const err = new SimLidarError(msg.message as string);
      this._handlers.onError?.(err);
      for (const entry of this.pending.values()) {
        entry.reject(err);
      }
      this.pending.clear();
    }
  }
}

