use glam::Vec3;
use wasm_bindgen::prelude::*;

/// Type alias for [`SensorConfig`]. Refers to the same sensor configuration struct.
pub type LidarConfig = SensorConfig;

/// Sensor configuration mirroring real-world LiDARs (e.g., Velodyne VLP-16, Ouster).
#[wasm_bindgen]
#[derive(Clone, Debug)]
pub struct SensorConfig {
    /// Number of rays per full horizontal sweep (360°).
    pub horizontal_resolution: u32,
    /// Number of vertical laser channels / rings (e.g. 16, 32, 64).
    pub vertical_channels: u32,
    /// Upper vertical FOV limit in degrees (e.g. +15.0 for VLP-16).
    pub vertical_fov_upper: f32,
    /// Lower vertical FOV limit in degrees (e.g. -15.0 for VLP-16).
    pub vertical_fov_lower: f32,
    /// Minimum valid range in metres.
    pub min_range: f32,
    /// Maximum valid range in metres.
    pub max_range: f32,
    /// Standard deviation of Gaussian noise added to each hit distance (0 = no noise).
    pub noise_stddev: f32,
}

#[wasm_bindgen]
impl SensorConfig {
    /// Create a new sensor configuration.
    #[wasm_bindgen(constructor)]
    pub fn new(
        horizontal_resolution: u32,
        vertical_channels: u32,
        vertical_fov_upper: f32,
        vertical_fov_lower: f32,
        min_range: f32,
        max_range: f32,
        noise_stddev: f32,
    ) -> SensorConfig {
        SensorConfig {
            horizontal_resolution,
            vertical_channels,
            vertical_fov_upper,
            vertical_fov_lower,
            min_range,
            max_range,
            noise_stddev,
        }
    }

    /// Returns a preset matching the Velodyne VLP-16.
    pub fn vlp16() -> SensorConfig {
        SensorConfig::new(1800, 16, 15.0, -15.0, 0.1, 100.0, 0.0)
    }

    /// Returns a preset matching the Ouster OS1-32.
    pub fn ouster_os1_32() -> SensorConfig {
        SensorConfig::new(1024, 32, 22.5, -22.5, 0.1, 120.0, 0.0)
    }

    /// Returns a preset matching the Ouster OS1-64.
    pub fn ouster_os1_64() -> SensorConfig {
        SensorConfig::new(2048, 64, 22.5, -22.5, 0.1, 120.0, 0.0)
    }

    /// Total number of rays fired per scan.
    pub fn total_rays(&self) -> u32 {
        self.horizontal_resolution * self.vertical_channels
    }
}

impl SensorConfig {
    /// Generate all sensor-local ray directions for a full scan.
    ///
    /// Returns unit vectors in the sensor's own coordinate frame, with no
    /// pose transformation applied.  Use [`generate_ray_directions`] to obtain
    /// world-space directions for a specific sensor orientation.
    ///
    /// [`generate_ray_directions`]: SensorConfig::generate_ray_directions
    pub fn generate_local_ray_directions(&self) -> Vec<Vec3> {
        let total = (self.horizontal_resolution * self.vertical_channels) as usize;
        let mut directions = Vec::with_capacity(total);

        let v_step = if self.vertical_channels > 1 {
            (self.vertical_fov_upper - self.vertical_fov_lower)
                / (self.vertical_channels - 1) as f32
        } else {
            0.0
        };

        let h_step = 360.0 / self.horizontal_resolution as f32;

        for v in 0..self.vertical_channels {
            let elevation_deg = self.vertical_fov_lower + v as f32 * v_step;
            let elevation_rad = elevation_deg.to_radians();
            let cos_elev = elevation_rad.cos();
            let sin_elev = elevation_rad.sin();

            for h in 0..self.horizontal_resolution {
                let azimuth_rad = (h as f32 * h_step).to_radians();
                directions.push(Vec3::new(
                    cos_elev * azimuth_rad.cos(),
                    sin_elev,
                    cos_elev * azimuth_rad.sin(),
                ));
            }
        }
        directions
    }

    /// Generate all ray directions for a full scan given a sensor orientation.
    ///
    /// Applies `rotation` to each local direction returned by
    /// [`generate_local_ray_directions`], producing world-space unit vectors.
    /// `rotation` must be a unit quaternion.
    ///
    /// [`generate_local_ray_directions`]: SensorConfig::generate_local_ray_directions
    pub fn generate_ray_directions(&self, rotation: glam::Quat) -> Vec<Vec3> {
        self.generate_local_ray_directions()
            .into_iter()
            .map(|dir| rotation * dir)
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sensor_config_total_rays() {
        let cfg = SensorConfig::new(1800, 16, 15.0, -15.0, 0.1, 100.0, 0.0);
        assert_eq!(cfg.total_rays(), 1800 * 16);
    }

    #[test]
    fn test_vlp16_preset() {
        let cfg = SensorConfig::vlp16();
        assert_eq!(cfg.horizontal_resolution, 1800);
        assert_eq!(cfg.vertical_channels, 16);
        assert!((cfg.vertical_fov_upper - 15.0).abs() < f32::EPSILON);
        assert!((cfg.vertical_fov_lower + 15.0).abs() < f32::EPSILON);
        assert!((cfg.min_range - 0.1).abs() < f32::EPSILON);
        assert!((cfg.max_range - 100.0).abs() < f32::EPSILON);
    }

    #[test]
    fn test_generate_ray_directions_count() {
        let cfg = SensorConfig::new(360, 8, 15.0, -15.0, 0.1, 100.0, 0.0);
        let dirs = cfg.generate_ray_directions(glam::Quat::IDENTITY);
        assert_eq!(dirs.len(), (360 * 8) as usize);
    }

    #[test]
    fn test_generate_ray_directions_unit_length() {
        let cfg = SensorConfig::new(36, 4, 10.0, -10.0, 0.1, 50.0, 0.0);
        let dirs = cfg.generate_ray_directions(glam::Quat::IDENTITY);
        for d in &dirs {
            let len = d.length();
            assert!((len - 1.0).abs() < 1e-5, "Direction not normalised: length={len}");
        }
    }

    // ── TDD: LidarConfig / 16-channel 360-degree scan ──────────────────────

    /// `LidarConfig` is a type alias for `SensorConfig`; use it interchangeably.
    #[test]
    fn test_lidar_config_alias() {
        let cfg: LidarConfig = LidarConfig::new(360, 16, 15.0, -15.0, 0.1, 100.0, 0.0);
        assert_eq!(cfg.vertical_channels, 16);
        assert_eq!(cfg.horizontal_resolution, 360);
    }

    /// A 16-channel, 360-degree config must produce exactly 16 × 360 = 5 760 local
    /// direction vectors.
    #[test]
    fn test_generate_local_ray_directions_16ch_360deg_count() {
        let cfg = LidarConfig::new(360, 16, 15.0, -15.0, 0.1, 100.0, 0.0);
        let dirs = cfg.generate_local_ray_directions();
        assert_eq!(dirs.len(), 360 * 16, "Expected 5760 local ray directions");
    }

    /// At azimuth index 0 (0° horizontal) the ray must lie in the XY-plane (Z ≈ 0)
    /// and point toward +X (X > 0).
    #[test]
    fn test_generate_local_ray_0deg_horizontal() {
        let cfg = LidarConfig::new(360, 16, 15.0, -15.0, 0.1, 100.0, 0.0);
        let dirs = cfg.generate_local_ray_directions();
        // Channel v=0, azimuth h=0 → index 0
        let dir = dirs[0];
        assert!(dir.z.abs() < 1e-5, "Z must be ~0 for 0° azimuth, got {}", dir.z);
        assert!(dir.x > 0.0, "X must be positive for 0° azimuth, got {}", dir.x);
    }

    /// The last vertical channel (v = channels-1) at azimuth 0 must have a Y
    /// component equal to sin(vertical_fov_upper).
    #[test]
    fn test_generate_local_ray_max_vertical_fov() {
        let cfg = LidarConfig::new(360, 16, 15.0, -15.0, 0.1, 100.0, 0.0);
        let dirs = cfg.generate_local_ray_directions();
        // Channel v=15 (fov_upper = 15°), azimuth h=0 → index 15 * 360 + 0
        let dir = dirs[15 * 360];
        let expected_y = 15.0_f32.to_radians().sin();
        assert!(
            (dir.y - expected_y).abs() < 1e-5,
            "Y must equal sin(fov_upper)={}; got {}",
            expected_y,
            dir.y
        );
        assert!(dir.z.abs() < 1e-5, "Z must be ~0 for 0° azimuth, got {}", dir.z);
    }

    /// `generate_ray_directions` must apply the quaternion rotation to each local
    /// direction produced by `generate_local_ray_directions`.
    #[test]
    fn test_generate_ray_directions_applies_rotation() {
        let cfg = LidarConfig::new(360, 16, 15.0, -15.0, 0.1, 100.0, 0.0);
        // 180° rotation around Y-axis flips X → -X and Z → -Z
        let rot = glam::Quat::from_rotation_y(std::f32::consts::PI);
        let local_dirs = cfg.generate_local_ray_directions();
        let world_dirs = cfg.generate_ray_directions(rot);
        assert_eq!(local_dirs.len(), world_dirs.len());
        for (local, world) in local_dirs.iter().zip(world_dirs.iter()) {
            assert!(
                (world.x + local.x).abs() < 1e-5,
                "180° Y-rotation must negate X: local.x={}, world.x={}",
                local.x,
                world.x
            );
            assert!(
                (world.y - local.y).abs() < 1e-5,
                "180° Y-rotation must preserve Y: local.y={}, world.y={}",
                local.y,
                world.y
            );
        }
    }
}
