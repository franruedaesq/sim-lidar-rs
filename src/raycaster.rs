use glam::{Quat, Vec3};
use rand::SeedableRng;
use rand::rngs::StdRng;
use rand_distr::{Distribution, Normal};

use crate::bvh::Bvh;
use crate::sensor::SensorConfig;

/// Output of a single scan.
pub struct ScanResult {
    /// Flat `[x, y, z, x, y, z, ...]` buffer of hit world-space coordinates.
    /// Only valid hits (within min/max range) are included.
    pub hits: Vec<f32>,
    /// Number of valid hits.
    pub hit_count: usize,
}

/// Execute a single LiDAR scan using a pre-built BVH.
///
/// * `bvh`      – The precomputed spatial index of the environment.
/// * `config`   – Sensor parameters.
/// * `position` – World-space sensor origin.
/// * `rotation` – Sensor orientation as a unit quaternion.
pub fn scan(bvh: &Bvh, config: &SensorConfig, position: Vec3, rotation: Quat) -> ScanResult {
    let directions = config.generate_ray_directions(rotation);
    let total = directions.len();
    let mut hits: Vec<f32> = Vec::with_capacity(total * 3);
    let mut hit_count = 0usize;

    // Set up optional noise RNG
    let use_noise = config.noise_stddev > 0.0;
    let mut rng: Option<StdRng> = if use_noise {
        Some(StdRng::from_entropy())
    } else {
        None
    };
    let noise_dist: Option<Normal<f32>> = if use_noise {
        Some(Normal::new(0.0, config.noise_stddev).expect("valid stddev"))
    } else {
        None
    };

    for dir in &directions {
        if let Some(mut t) = bvh.cast_ray(position, *dir, config.max_range) {
            if t < config.min_range {
                continue;
            }
            // Apply Gaussian noise to the range measurement if configured
            if let (Some(ref mut rng), Some(ref dist)) = (rng.as_mut(), noise_dist.as_ref()) {
                let noise: f32 = dist.sample(rng as &mut StdRng);
                t = (t + noise).max(0.0);
            }
            let hit = position + *dir * t;
            hits.push(hit.x);
            hits.push(hit.y);
            hits.push(hit.z);
            hit_count += 1;
        }
    }

    ScanResult { hits, hit_count }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bvh::Bvh;
    use crate::sensor::SensorConfig;

    fn ground_plane_bvh() -> Bvh {
        // A 20x20 ground plane at y=0
        let vertices: Vec<f32> = vec![
            -10.0, 0.0, -10.0,
             10.0, 0.0, -10.0,
             10.0, 0.0,  10.0,
            -10.0, 0.0,  10.0,
        ];
        let indices: Vec<u32> = vec![0, 1, 2, 0, 2, 3];
        Bvh::build(&vertices, &indices)
    }

    #[test]
    fn test_scan_hits_ground() {
        let bvh = ground_plane_bvh();
        // Single downward-pointing ray
        let config = SensorConfig::new(1, 1, -89.9, -89.9, 0.1, 50.0, 0.0);
        let result = scan(&bvh, &config, Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY);
        assert!(result.hit_count > 0, "Expected at least one ground hit");
    }

    #[test]
    fn test_scan_max_range_filters_hits() {
        let bvh = ground_plane_bvh();
        // Sensor 5m above, max range 3m — the ground is out of range
        let config = SensorConfig::new(1, 1, -89.9, -89.9, 0.1, 3.0, 0.0);
        let result = scan(&bvh, &config, Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY);
        assert_eq!(result.hit_count, 0, "Ground is beyond max range, no hits expected");
    }

    #[test]
    fn test_scan_output_buffer_length() {
        let bvh = ground_plane_bvh();
        let config = SensorConfig::new(36, 1, -89.0, -89.0, 0.1, 100.0, 0.0);
        let result = scan(&bvh, &config, Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY);
        assert_eq!(result.hits.len(), result.hit_count * 3);
    }
}
