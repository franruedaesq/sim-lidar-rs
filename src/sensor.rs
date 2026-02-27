use glam::Vec3;
use wasm_bindgen::prelude::*;

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
    /// Generate all ray directions for a full scan given a pose.
    ///
    /// `position` and `rotation` together define the sensor pose.
    /// `rotation` is a unit quaternion as `[x, y, z, w]`.
    pub fn generate_ray_directions(&self, rotation: glam::Quat) -> Vec<Vec3> {
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
                let local_dir = Vec3::new(
                    cos_elev * azimuth_rad.cos(),
                    sin_elev,
                    cos_elev * azimuth_rad.sin(),
                );
                directions.push(rotation * local_dir);
            }
        }
        directions
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
}
