# sim-lidar-rs

High-performance simulated LiDAR sensor library built for the web.  
Computes 100,000+ ray intersections per frame in Rust/WebAssembly, off the main thread.

## Features

- ⚡ **Fast** – Bounding Volume Hierarchy (BVH) accelerator built in Rust, compiled to Wasm
- 🔒 **Non-blocking** – All computation runs inside a Web Worker
- 🎯 **Realistic** – Configurable vertical channels, FOV, range limits, and Gaussian noise
- 📦 **Zero-copy** – Hit-point buffers transferred directly from Wasm to JS
- 🔷 **Strictly typed** – Full TypeScript API with sensor presets (VLP-16, Ouster OS1-32/64)

## Quick Start

```ts
import { LidarClient, VLP16_CONFIG } from 'sim-lidar-rs';

const vertices = new Float32Array([/* x,y,z ... */]);
const indices  = new Uint32Array([/* i0,i1,i2 ... */]);

const lidar = new LidarClient(vertices, indices, VLP16_CONFIG);
await lidar.ready();

const { hits, hitCount } = await lidar.scan({
  position: { x: 0, y: 1.5, z: 0 },
});
console.log(`${hitCount} hits`); // hits is Float32Array [x,y,z, ...]

lidar.dispose();
```

## Build

```bash
# Build Wasm module
wasm-pack build --target web --out-dir ts/wasm

# Build TypeScript library
npm run build

# Run Rust unit tests
cargo test

# Run TypeScript tests
npm test
```

## Documentation

See [ARCHITECTURE.md](./ARCHITECTURE.md) for the full architecture, core objectives, technical design, and sensor configuration reference.

## License

MIT
