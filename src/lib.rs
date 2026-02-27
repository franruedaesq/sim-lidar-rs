mod bvh;
mod raycaster;
mod sensor;

use glam::{Quat, Vec3};
use js_sys::Float32Array;
use wasm_bindgen::prelude::*;

pub use bvh::{Intersection, Ray};
pub use sensor::{LidarConfig, SensorConfig};

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

/// A streamlined LiDAR simulator designed for direct use from JavaScript.
///
/// Unlike [`LidarSimulator`], geometry is loaded separately via [`Simulator::load_geometry`]
/// and scan results are returned as a zero-copy `Float32Array` view directly into
/// Wasm linear memory.
#[wasm_bindgen]
pub struct Simulator {
    bvh: Option<bvh::Bvh>,
    config: SensorConfig,
    /// Pre-allocated hit buffer re-used across scans to avoid repeated allocation.
    hit_buffer: Vec<f32>,
}

#[wasm_bindgen]
impl Simulator {
    /// Create a new `Simulator` with the given sensor configuration.
    ///
    /// Call [`load_geometry`] before [`perform_scan`].
    ///
    /// [`load_geometry`]: Simulator::load_geometry
    /// [`perform_scan`]: Simulator::perform_scan
    #[wasm_bindgen(constructor)]
    pub fn new(config: SensorConfig) -> Simulator {
        Simulator {
            bvh: None,
            config,
            // hit_buffer is populated on the first call to perform_scan; its
            // capacity grows to fit the scan output and is reused in subsequent
            // calls by swapping in the raycaster's output Vec.
            hit_buffer: Vec::new(),
        }
    }

    /// Ingest environment geometry and (re)build the internal BVH.
    ///
    /// * `vertices` – Flat `Float32Array` of vertex positions `[x,y,z, …]`.
    /// * `indices`  – Flat `Uint32Array` of triangle vertex indices.
    ///
    /// This method may be called multiple times to swap the environment at runtime.
    pub fn load_geometry(&mut self, vertices: &[f32], indices: &[u32]) {
        self.bvh = Some(bvh::Bvh::build(vertices, indices));
    }

    /// Run a full scan from a given pose and return the hit point cloud.
    ///
    /// * `x`, `y`, `z`           – Sensor world-space position.
    /// * `qx`, `qy`, `qz`, `qw` – Sensor orientation as a unit quaternion.
    ///
    /// Returns a `Float32Array` view `[x,y,z, x,y,z, …]` directly into Wasm
    /// linear memory.  The view is valid until the next call to `perform_scan`.
    ///
    /// # Panics
    ///
    /// Panics if [`load_geometry`] has not been called first.
    ///
    /// # Safety
    ///
    /// The returned `Float32Array` is a direct view into Wasm linear memory.
    /// Do not call any Wasm-allocating function while the view is alive, as
    /// a memory grow could invalidate the underlying pointer.
    ///
    /// [`load_geometry`]: Simulator::load_geometry
    pub fn perform_scan(
        &mut self,
        x: f32,
        y: f32,
        z: f32,
        qx: f32,
        qy: f32,
        qz: f32,
        qw: f32,
    ) -> Float32Array {
        let bvh = self
            .bvh
            .as_ref()
            .expect("load_geometry must be called before perform_scan");
        let position = Vec3::new(x, y, z);
        let rotation = Quat::from_xyzw(qx, qy, qz, qw).normalize();
        let result = raycaster::scan(bvh, &self.config, position, rotation);
        // Assign the newly filled Vec.  On the next call the old allocation is
        // dropped; if both Vecs have the same capacity this is still a single
        // allocation per scan (the raycaster pre-sizes its output identically).
        self.hit_buffer = result.hits;
        // SAFETY: `hit_buffer` owns the backing allocation and is not resized
        // after this point within the same call frame.  The caller must consume
        // or copy the returned view before calling `perform_scan` again, as the
        // next call replaces the backing buffer and invalidates this view.
        unsafe { Float32Array::view(&self.hit_buffer) }
    }

    /// Replace the sensor configuration without rebuilding the BVH.
    pub fn set_config(&mut self, config: SensorConfig) {
        self.config = config;
    }

    /// Returns the number of valid hits from the last scan.
    pub fn last_hit_count(&self) -> usize {
        self.hit_buffer.len() / 3
    }
}
