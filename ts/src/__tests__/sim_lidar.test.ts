/**
 * Unit tests for the SimLidar TypeScript wrapper.
 *
 * Because the real Web Worker API is not available in the Node.js test
 * environment, a minimal `StubWorker` is injected via `vi.stubGlobal` to
 * simulate synchronous worker responses without loading any Wasm binary.
 */
import { describe, it, expect, vi, beforeEach, afterEach } from "vitest";
import type { SensorConfig, Pose, Geometry } from "../types.js";
import { SimLidarDisposedError } from "../types.js";

// ── Fixture data ──────────────────────────────────────────────────────────────

const TINY_CONFIG: SensorConfig = {
  horizontalResolution: 36,
  verticalChannels: 4,
  verticalFovUpper: -10,
  verticalFovLower: -20,
  minRange: 0.1,
  maxRange: 20,
  noiseStddev: 0,
};

const FLAT_GEOMETRY: Geometry = {
  vertices: new Float32Array([-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10]),
  indices: new Uint32Array([0, 1, 2, 0, 2, 3]),
};

const ORIGIN_POSE: Pose = { position: { x: 0, y: 1, z: 0 } };

// ── StubWorker ────────────────────────────────────────────────────────────────

/**
 * Minimal Web Worker stub.  Intercepts `postMessage` calls and replies with
 * the same responses a real worker would produce, using `queueMicrotask` to
 * ensure responses arrive asynchronously (matching real Worker behaviour).
 */
class StubWorker {
  private _handlers: Array<(evt: MessageEvent) => void> = [];
  private _terminated = false;

  // eslint-disable-next-line @typescript-eslint/no-useless-constructor
  constructor(_url: string | URL, _opts?: WorkerOptions) {}

  addEventListener(type: string, handler: (evt: MessageEvent) => void): void {
    if (type === "message") this._handlers.push(handler);
  }

  postMessage(data: unknown): void {
    if (this._terminated) return;
    const msg = data as { type: string; __id?: string };
    const dispatch = (payload: unknown): void => {
      if (this._terminated) return;
      const evt = { data: payload } as MessageEvent;
      for (const h of this._handlers) h(evt);
    };

    queueMicrotask(() => {
      switch (msg.type) {
        case "init":
          dispatch({ type: "ready" });
          break;
        case "updateEnvironment":
          dispatch({ type: "environmentUpdated", __id: msg.__id });
          break;
        case "scan": {
          // Return a synthetic three-point hit (one hit = [x,y,z])
          const hits = new Float32Array([1.0, 0.0, 2.0]);
          dispatch({ type: "scan", hits, hitCount: 1, __id: msg.__id });
          break;
        }
        case "destroy":
          dispatch({ type: "destroyed" });
          break;
        default:
          break;
      }
    });
  }

  terminate(): void { this._terminated = true; }
}

// ── Test suite ────────────────────────────────────────────────────────────────

describe("SimLidar", () => {
  beforeEach(() => {
    // Replace the browser Worker API with our stub for every test.
    vi.stubGlobal("Worker", StubWorker);
  });

  afterEach(() => {
    vi.unstubAllGlobals();
  });

  it("scan() returns a Promise that resolves with a Float32Array", async () => {
    const { SimLidar } = await import("../index.js");

    const lidar = new SimLidar(TINY_CONFIG, "stub://worker");
    await lidar.init();
    await lidar.updateEnvironment(FLAT_GEOMETRY);

    const hits = await lidar.scan(ORIGIN_POSE);

    expect(hits).toBeInstanceOf(Float32Array);
    expect(hits.length % 3).toBe(0);

    lidar.destroy();
  });

  it("rapid successive scan() calls do not crash the worker", async () => {
    const { SimLidar } = await import("../index.js");

    const lidar = new SimLidar(TINY_CONFIG, "stub://worker");
    await lidar.init();
    await lidar.updateEnvironment(FLAT_GEOMETRY);

    // Fire 20 concurrent scans – none should reject or throw.
    const results = await Promise.all(
      Array.from({ length: 20 }, () => lidar.scan(ORIGIN_POSE))
    );

    expect(results).toHaveLength(20);
    for (const result of results) {
      expect(result).toBeInstanceOf(Float32Array);
    }

    lidar.destroy();
  });

  it("init() resolves before scan() is called", async () => {
    const { SimLidar } = await import("../index.js");

    const lidar = new SimLidar(TINY_CONFIG, "stub://worker");

    // init() must settle without rejection
    await expect(lidar.init()).resolves.toBeUndefined();

    lidar.destroy();
  });

  it("updateEnvironment() resolves with undefined", async () => {
    const { SimLidar } = await import("../index.js");

    const lidar = new SimLidar(TINY_CONFIG, "stub://worker");
    await lidar.init();

    await expect(lidar.updateEnvironment(FLAT_GEOMETRY)).resolves.toBeUndefined();

    lidar.destroy();
  });

  it("destroy() rejects in-flight scan() calls", async () => {
    const { SimLidar } = await import("../index.js");

    const lidar = new SimLidar(TINY_CONFIG, "stub://worker");
    await lidar.init();
    await lidar.updateEnvironment(FLAT_GEOMETRY);

    // Start a scan but destroy before the microtask resolves it.
    const scanPromise = lidar.scan(ORIGIN_POSE);
    lidar.destroy();

    await expect(scanPromise).rejects.toThrow(SimLidarDisposedError);
  });
});
