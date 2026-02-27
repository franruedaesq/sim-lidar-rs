mod bvh;
mod raycaster;
mod sensor;

use glam::{Quat, Vec3};
use wasm_bindgen::prelude::*;

pub use bvh::{Intersection, Ray};
pub use sensor::SensorConfig;

/// The main LiDAR simulator.  Holds the pre-built BVH for the environment
/// geometry and exposes scanning methods to JavaScript via wasm-bindgen.
#[wasm_bindgen]
pub struct LidarSimulator {
    bvh: bvh::Bvh,
    config: SensorConfig,
    /// Pre-allocated hit buffer re-used across scans to avoid repeated allocation.
    hit_buffer: Vec<f32>,
}

#[wasm_bindgen]
impl LidarSimulator {
    /// Create a new simulator.
    ///
    /// * `vertices` – Flat `Float32Array` of vertex positions `[x,y,z, ...]`.
    /// * `indices`  – Flat `Uint32Array` of triangle indices.
    /// * `config`   – Sensor configuration.
    #[wasm_bindgen(constructor)]
    pub fn new(vertices: &[f32], indices: &[u32], config: SensorConfig) -> LidarSimulator {
        let bvh = bvh::Bvh::build(vertices, indices);
        let capacity = (config.total_rays() * 3) as usize;
        LidarSimulator {
            bvh,
            config,
            hit_buffer: Vec::with_capacity(capacity),
        }
    }

    /// Replace the sensor configuration at runtime.
    pub fn set_config(&mut self, config: SensorConfig) {
        self.config = config;
    }

    /// Run a full scan from a given pose.
    ///
    /// * `px`, `py`, `pz`        – Sensor world-space position.
    /// * `qx`, `qy`, `qz`, `qw` – Sensor orientation quaternion.
    ///
    /// Returns a `Float32Array` view `[x,y,z, x,y,z, …]` of the hit points.
    /// The view is valid until the next call to `scan`.
    pub fn scan(&mut self, px: f32, py: f32, pz: f32, qx: f32, qy: f32, qz: f32, qw: f32) -> Vec<f32> {
        let position = Vec3::new(px, py, pz);
        let rotation = Quat::from_xyzw(qx, qy, qz, qw).normalize();
        let result = raycaster::scan(&self.bvh, &self.config, position, rotation);
        self.hit_buffer = result.hits;
        self.hit_buffer.clone()
    }

    /// Returns the last scan's hit count.
    pub fn last_hit_count(&self) -> usize {
        self.hit_buffer.len() / 3
    }
}
