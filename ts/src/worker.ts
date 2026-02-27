/**
 * worker.ts – Web Worker script that hosts the Wasm module.
 *
 * Messages sent TO the worker:
 *   { type: 'init', vertices: Float32Array, indices: Uint32Array, config: SensorConfig }
 *   { type: 'scan', pose: Pose }
 *   { type: 'setConfig', config: SensorConfig }
 *
 * Messages posted FROM the worker:
 *   { type: 'ready' }
 *   { type: 'scan', hits: Float32Array, hitCount: number }
 *   { type: 'error', message: string }
 */

import type { SensorConfig, Pose } from "./types.js";

// The wasm-pack output is loaded dynamically so this worker is usable both in
// browser and Node.js (via vitest) without bundler magic at worker-load time.
// In production the path resolves relative to the built bundle.

type WasmModule = typeof import("../wasm/sim_lidar_rs.js");
type LidarSimulator = InstanceType<WasmModule["LidarSimulator"]>;
type WasmSensorConfig = InstanceType<WasmModule["SensorConfig"]>;

let wasm: WasmModule | null = null;
let simulator: LidarSimulator | null = null;

async function loadWasm(): Promise<WasmModule> {
  // Dynamic import allows the bundler to treeshake and inline the wasm URL
  const mod = await import("../wasm/sim_lidar_rs.js");
  await mod.default(); // initialise the wasm binary
  return mod;
}

function tsConfigToWasm(wasm: WasmModule, cfg: SensorConfig): WasmSensorConfig {
  return new wasm.SensorConfig(
    cfg.horizontalResolution,
    cfg.verticalChannels,
    cfg.verticalFovUpper,
    cfg.verticalFovLower,
    cfg.minRange,
    cfg.maxRange,
    cfg.noiseStddev
  );
}

self.addEventListener("message", async (evt: MessageEvent) => {
  const msg = evt.data as {
    type: string;
    vertices?: Float32Array;
    indices?: Uint32Array;
    config?: SensorConfig;
    pose?: Pose;
  };

  try {
    if (msg.type === "init") {
      if (!wasm) {
        wasm = await loadWasm();
      }
      const wasmCfg = tsConfigToWasm(wasm, msg.config!);
      simulator = new wasm.LidarSimulator(msg.vertices!, msg.indices!, wasmCfg);
      self.postMessage({ type: "ready" });
      return;
    }

    if (msg.type === "setConfig") {
      if (!wasm || !simulator) throw new Error("Simulator not initialised");
      const wasmCfg = tsConfigToWasm(wasm, msg.config!);
      simulator.set_config(wasmCfg);
      return;
    }

    if (msg.type === "scan") {
      if (!simulator) throw new Error("Simulator not initialised");
      const pose = msg.pose!;
      const rot = pose.rotation ?? { x: 0, y: 0, z: 0, w: 1 };
      const hits: Float32Array = simulator.scan(
        pose.position.x,
        pose.position.y,
        pose.position.z,
        rot.x,
        rot.y,
        rot.z,
        rot.w
      );
      self.postMessage(
        { type: "scan", hits, hitCount: hits.length / 3, __id: (msg as { __id?: string }).__id },
        [hits.buffer]
      );
      return;
    }
  } catch (err: unknown) {
    const message = err instanceof Error ? err.message : String(err);
    self.postMessage({ type: "error", message });
  }
});
