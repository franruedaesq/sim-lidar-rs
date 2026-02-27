/**
 * worker.ts – Web Worker script that hosts the Wasm module.
 *
 * Messages sent TO the worker:
 *   { type: 'init', config: SensorConfig, vertices?: Float32Array, indices?: Uint32Array }
 *   { type: 'updateEnvironment', vertices: Float32Array, indices: Uint32Array, __id: string }
 *   { type: 'scan', pose: Pose, __id: string }
 *   { type: 'setConfig', config: SensorConfig }
 *   { type: 'destroy' }
 *
 * Messages posted FROM the worker:
 *   { type: 'ready' }
 *   { type: 'environmentUpdated', __id: string }
 *   { type: 'scan', hits: Float32Array, hitCount: number, __id: string }
 *   { type: 'destroyed' }
 *   { type: 'error', message: string }
 */

import type { SensorConfig, Pose } from "./types.js";

// The wasm-pack output is loaded dynamically so this worker is usable both in
// browser and Node.js (via vitest) without bundler magic at worker-load time.
// In production the path resolves relative to the built bundle.

type WasmModule = typeof import("../wasm/sim_lidar_rs.js");
type SimulatorInstance = InstanceType<WasmModule["Simulator"]>;
type WasmSensorConfig = InstanceType<WasmModule["SensorConfig"]>;

let wasm: WasmModule | null = null;
let simulator: SimulatorInstance | null = null;

async function loadWasm(): Promise<WasmModule> {
  // Dynamic import allows the bundler to treeshake and inline the wasm URL
  const mod = await import("../wasm/sim_lidar_rs.js");
  await mod.default(); // initialise the wasm binary
  return mod;
}

function tsConfigToWasm(wasmMod: WasmModule, cfg: SensorConfig): WasmSensorConfig {
  return new wasmMod.SensorConfig(
    cfg.horizontalResolution,
    cfg.verticalChannels,
    cfg.verticalFovUpper,
    cfg.verticalFovLower,
    cfg.minRange,
    cfg.maxRange,
    cfg.noiseStddev
  );
}

/** Explicitly free the current Wasm simulator to release linear memory. */
function freeSimulator(): void {
  if (simulator) {
    simulator.free();
    simulator = null;
  }
}

self.addEventListener("message", async (evt: MessageEvent) => {
  const msg = evt.data as {
    type: string;
    vertices?: Float32Array;
    indices?: Uint32Array;
    config?: SensorConfig;
    pose?: Pose;
    __id?: string;
  };

  try {
    if (msg.type === "init") {
      if (!wasm) {
        wasm = await loadWasm();
      }
      // Free any previously-allocated Wasm instance before replacing it.
      freeSimulator();
      const wasmCfg = tsConfigToWasm(wasm, msg.config!);
      simulator = new wasm.Simulator(wasmCfg);
      // Accept optional geometry at init-time for backward compatibility.
      if (msg.vertices && msg.indices) {
        simulator.load_geometry(msg.vertices, msg.indices);
      }
      self.postMessage({ type: "ready" });
      return;
    }

    if (msg.type === "updateEnvironment") {
      if (!wasm || !simulator) throw new Error("Simulator not initialised");
      simulator.load_geometry(msg.vertices!, msg.indices!);
      self.postMessage({ type: "environmentUpdated", __id: msg.__id });
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
      // Slice immediately to own the data – the Wasm view is only valid until
      // the next allocating call, so we must copy before transferring.
      const hits = simulator.perform_scan(
        pose.position.x,
        pose.position.y,
        pose.position.z,
        rot.x,
        rot.y,
        rot.z,
        rot.w
      ).slice();
      self.postMessage(
        { type: "scan", hits, hitCount: hits.length / 3, __id: msg.__id },
        [hits.buffer]
      );
      return;
    }

    if (msg.type === "destroy") {
      // Explicitly release Wasm linear memory before the worker is terminated.
      freeSimulator();
      self.postMessage({ type: "destroyed" });
      return;
    }
  } catch (err: unknown) {
    const message = err instanceof Error ? err.message : String(err);
    self.postMessage({ type: "error", message });
  }
});
