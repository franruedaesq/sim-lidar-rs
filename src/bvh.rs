use glam::Vec3;

/// A ray defined by an origin and a direction.
///
/// The direction should be normalised (unit-length) for the intersection
/// distances returned by [`Ray::cast`] to represent metres. Callers are
/// responsible for normalisation.
#[derive(Clone, Debug)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}

impl Ray {
    pub fn new(origin: Vec3, direction: Vec3) -> Self {
        Self { origin, direction }
    }

    /// Cast this ray against a BVH and return the closest intersection within `t_max`.
    pub fn cast(&self, bvh: &Bvh, t_max: f32) -> Option<Intersection> {
        bvh.cast_ray(self.origin, self.direction, t_max)
            .map(|distance| Intersection { distance })
    }
}

/// The result of a successful ray-geometry intersection.
#[derive(Clone, Debug, PartialEq)]
pub struct Intersection {
    /// Distance along the ray from the origin to the hit point.
    pub distance: f32,
}

/// An axis-aligned bounding box (AABB)
#[derive(Clone, Debug)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    pub fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    pub fn empty() -> Self {
        Self {
            min: Vec3::splat(f32::INFINITY),
            max: Vec3::splat(f32::NEG_INFINITY),
        }
    }

    pub fn expand(&mut self, point: Vec3) {
        self.min = self.min.min(point);
        self.max = self.max.max(point);
    }

    pub fn merge(&self, other: &Aabb) -> Aabb {
        Aabb {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    pub fn centroid(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    /// Slab-method ray-AABB intersection test.
    /// Returns the entry distance, or None if no intersection.
    pub fn ray_intersect(&self, origin: Vec3, inv_dir: Vec3, t_max: f32) -> Option<f32> {
        let t1 = (self.min - origin) * inv_dir;
        let t2 = (self.max - origin) * inv_dir;
        let t_min_v = t1.min(t2);
        let t_max_v = t1.max(t2);
        let t_near = t_min_v.x.max(t_min_v.y).max(t_min_v.z);
        let t_far = t_max_v.x.min(t_max_v.y).min(t_max_v.z);
        if t_near <= t_far && t_far >= 0.0 && t_near <= t_max {
            Some(t_near.max(0.0))
        } else {
            None
        }
    }
}

/// A single triangle defined by three vertex indices into a flat vertex buffer.
#[derive(Clone, Debug)]
pub struct Triangle {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
}

impl Triangle {
    pub fn aabb(&self) -> Aabb {
        let mut aabb = Aabb::empty();
        aabb.expand(self.a);
        aabb.expand(self.b);
        aabb.expand(self.c);
        aabb
    }

    pub fn centroid(&self) -> Vec3 {
        (self.a + self.b + self.c) / 3.0
    }

    /// Möller–Trumbore ray-triangle intersection.
    /// Returns the hit distance, or None if no intersection.
    pub fn ray_intersect(&self, origin: Vec3, direction: Vec3, t_max: f32) -> Option<f32> {
        const EPSILON: f32 = 1e-7;
        let edge1 = self.b - self.a;
        let edge2 = self.c - self.a;
        let h = direction.cross(edge2);
        let det = edge1.dot(h);
        if det.abs() < EPSILON {
            return None;
        }
        let inv_det = 1.0 / det;
        let s = origin - self.a;
        let u = inv_det * s.dot(h);
        if !(0.0..=1.0).contains(&u) {
            return None;
        }
        let q = s.cross(edge1);
        let v = inv_det * direction.dot(q);
        if v < 0.0 || u + v > 1.0 {
            return None;
        }
        let t = inv_det * edge2.dot(q);
        if t > EPSILON && t <= t_max {
            Some(t)
        } else {
            None
        }
    }
}

/// A node in the BVH tree.
enum BvhNode {
    Leaf {
        aabb: Aabb,
        triangle_indices: Vec<usize>,
    },
    Interior {
        aabb: Aabb,
        left: Box<BvhNode>,
        right: Box<BvhNode>,
    },
}

impl BvhNode {
    fn aabb(&self) -> &Aabb {
        match self {
            BvhNode::Leaf { aabb, .. } => aabb,
            BvhNode::Interior { aabb, .. } => aabb,
        }
    }

    /// Traverse the BVH and return the closest hit distance along a ray.
    fn intersect(&self, triangles: &[Triangle], origin: Vec3, direction: Vec3, inv_dir: Vec3, t_max: f32) -> Option<f32> {
        let node_aabb = self.aabb();
        if node_aabb.ray_intersect(origin, inv_dir, t_max).is_none() {
            return None;
        }
        match self {
            BvhNode::Leaf { triangle_indices, .. } => {
                let mut closest = None::<f32>;
                for &idx in triangle_indices {
                    let limit = closest.unwrap_or(t_max);
                    if let Some(t) = triangles[idx].ray_intersect(origin, direction, limit) {
                        closest = Some(t);
                    }
                }
                closest
            }
            BvhNode::Interior { left, right, .. } => {
                let t_left = left.intersect(triangles, origin, direction, inv_dir, t_max);
                let limit = t_left.unwrap_or(t_max);
                let t_right = right.intersect(triangles, origin, direction, inv_dir, limit);
                match (t_left, t_right) {
                    (Some(a), Some(b)) => Some(a.min(b)),
                    (Some(a), None) => Some(a),
                    (None, Some(b)) => Some(b),
                    (None, None) => None,
                }
            }
        }
    }
}

/// Bounding Volume Hierarchy accelerator.
pub struct Bvh {
    root: Option<BvhNode>,
    pub triangles: Vec<Triangle>,
}

const MAX_LEAF_TRIANGLES: usize = 4;

impl Bvh {
    /// Build a BVH from a flat array of vertices and indices.
    ///
    /// `vertices` is a flat `[x0,y0,z0, x1,y1,z1, ...]` slice.
    /// `indices` is a flat `[i0,i1,i2, i3,i4,i5, ...]` triangle index slice.
    pub fn build(vertices: &[f32], indices: &[u32]) -> Self {
        assert!(indices.len() % 3 == 0, "indices length must be a multiple of 3");
        let triangles: Vec<Triangle> = indices
            .chunks_exact(3)
            .map(|tri| {
                let a = Vec3::new(
                    vertices[(tri[0] * 3) as usize],
                    vertices[(tri[0] * 3 + 1) as usize],
                    vertices[(tri[0] * 3 + 2) as usize],
                );
                let b = Vec3::new(
                    vertices[(tri[1] * 3) as usize],
                    vertices[(tri[1] * 3 + 1) as usize],
                    vertices[(tri[1] * 3 + 2) as usize],
                );
                let c = Vec3::new(
                    vertices[(tri[2] * 3) as usize],
                    vertices[(tri[2] * 3 + 1) as usize],
                    vertices[(tri[2] * 3 + 2) as usize],
                );
                Triangle { a, b, c }
            })
            .collect();

        let mut indices: Vec<usize> = (0..triangles.len()).collect();
        let root = if triangles.is_empty() {
            None
        } else {
            Some(Self::build_recursive(&triangles, &mut indices))
        };
        Self { root, triangles }
    }

    fn build_recursive(triangles: &[Triangle], indices: &mut [usize]) -> BvhNode {
        let mut aabb = Aabb::empty();
        for &i in indices.iter() {
            aabb = aabb.merge(&triangles[i].aabb());
        }

        if indices.len() <= MAX_LEAF_TRIANGLES {
            return BvhNode::Leaf {
                aabb,
                triangle_indices: indices.to_vec(),
            };
        }

        // Find the longest axis to split along
        let extent = aabb.max - aabb.min;
        let axis = if extent.x >= extent.y && extent.x >= extent.z {
            0
        } else if extent.y >= extent.z {
            1
        } else {
            2
        };

        // Sort indices by centroid along the chosen axis
        indices.sort_unstable_by(|&a, &b| {
            let ca = triangles[a].centroid();
            let cb = triangles[b].centroid();
            let va = [ca.x, ca.y, ca.z][axis];
            let vb = [cb.x, cb.y, cb.z][axis];
            va.partial_cmp(&vb).unwrap_or(std::cmp::Ordering::Equal)
        });

        let mid = indices.len() / 2;
        let (left_indices, right_indices) = indices.split_at_mut(mid);

        let left = Box::new(Self::build_recursive(triangles, left_indices));
        let right = Box::new(Self::build_recursive(triangles, right_indices));

        BvhNode::Interior { aabb, left, right }
    }

    /// Rebuild the BVH in-place with updated geometry.
    ///
    /// Call this when dynamic objects have moved to keep the spatial index consistent.
    /// `vertices` and `indices` follow the same conventions as [Bvh::build].
    pub fn update(&mut self, vertices: &[f32], indices: &[u32]) {
        *self = Self::build(vertices, indices);
    }

    /// Cast a ray and return the closest hit distance, or None.
    pub fn cast_ray(&self, origin: Vec3, direction: Vec3, t_max: f32) -> Option<f32> {
        let root = self.root.as_ref()?;
        let inv_dir = Vec3::new(1.0 / direction.x, 1.0 / direction.y, 1.0 / direction.z);
        root.intersect(&self.triangles, origin, direction, inv_dir, t_max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat_box_mesh() -> (Vec<f32>, Vec<u32>) {
        // A simple flat quad (two triangles forming a 2x2 square at y=0)
        let vertices: Vec<f32> = vec![
            -1.0, 0.0, -1.0,
             1.0, 0.0, -1.0,
             1.0, 0.0,  1.0,
            -1.0, 0.0,  1.0,
        ];
        let indices: Vec<u32> = vec![0, 1, 2, 0, 2, 3];
        (vertices, indices)
    }

    #[test]
    fn test_bvh_build_empty() {
        let bvh = Bvh::build(&[], &[]);
        assert!(bvh.triangles.is_empty());
        assert!(bvh.cast_ray(Vec3::ZERO, Vec3::Y, 100.0).is_none());
    }

    #[test]
    fn test_bvh_hit() {
        let (vertices, indices) = flat_box_mesh();
        let bvh = Bvh::build(&vertices, &indices);
        // Ray pointing straight down, should hit the quad at y=0
        let hit = bvh.cast_ray(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 100.0);
        assert!(hit.is_some());
        let t = hit.unwrap();
        assert!((t - 5.0).abs() < 1e-4, "Expected t≈5.0, got {t}");
    }

    #[test]
    fn test_bvh_miss() {
        let (vertices, indices) = flat_box_mesh();
        let bvh = Bvh::build(&vertices, &indices);
        // Ray pointing away from the quad
        let hit = bvh.cast_ray(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 100.0);
        assert!(hit.is_none());
    }

    #[test]
    fn test_bvh_range_limit() {
        let (vertices, indices) = flat_box_mesh();
        let bvh = Bvh::build(&vertices, &indices);
        // Ray hits at t=5 but max range is 3 — should miss
        let hit = bvh.cast_ray(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 3.0);
        assert!(hit.is_none());
    }

    #[test]
    fn test_aabb_ray_intersect() {
        let aabb = Aabb::new(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(1.0, 1.0, 1.0));
        let inv_dir = Vec3::new(0.0, -1.0, 0.0);
        // Avoid NaN by using large values for zero-component inverse
        let inv_dir_safe = Vec3::new(f32::INFINITY, -1.0, f32::INFINITY);
        let hit = aabb.ray_intersect(Vec3::new(0.0, 5.0, 0.0), inv_dir_safe, 100.0);
        assert!(hit.is_some());
    }

    #[test]
    fn test_triangle_ray_intersect() {
        let tri = Triangle {
            a: Vec3::new(-1.0, 0.0, -1.0),
            b: Vec3::new(1.0, 0.0, -1.0),
            c: Vec3::new(0.0, 0.0, 1.0),
        };
        let hit = tri.ray_intersect(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 100.0);
        assert!(hit.is_some());
        let t = hit.unwrap();
        assert!((t - 5.0).abs() < 1e-4, "Expected t≈5.0, got {t}");
    }

    // ── Möller–Trumbore miss cases ──────────────────────────────────────────

    #[test]
    fn test_triangle_ray_parallel_miss() {
        // Ray travelling parallel to the triangle plane must not intersect.
        let tri = Triangle {
            a: Vec3::new(-1.0, 0.0, -1.0),
            b: Vec3::new(1.0, 0.0, -1.0),
            c: Vec3::new(0.0, 0.0, 1.0),
        };
        let hit = tri.ray_intersect(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 100.0);
        assert!(hit.is_none(), "Parallel ray must not intersect");
    }

    #[test]
    fn test_triangle_ray_outside_uv_miss() {
        // Ray points toward the plane of the triangle but outside its bounds.
        let tri = Triangle {
            a: Vec3::new(-1.0, 0.0, -1.0),
            b: Vec3::new(1.0, 0.0, -1.0),
            c: Vec3::new(0.0, 0.0, 1.0),
        };
        // Shoot from far to the side so u/v are out of range.
        let hit = tri.ray_intersect(Vec3::new(10.0, 5.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 100.0);
        assert!(hit.is_none(), "Ray beside triangle must not intersect");
    }

    #[test]
    fn test_triangle_ray_behind_origin_miss() {
        // Triangle is behind the ray origin — t would be negative.
        let tri = Triangle {
            a: Vec3::new(-1.0, 0.0, -1.0),
            b: Vec3::new(1.0, 0.0, -1.0),
            c: Vec3::new(0.0, 0.0, 1.0),
        };
        // Ray starts below the plane shooting further down — triangle at y=0 is behind.
        let hit = tri.ray_intersect(Vec3::new(0.0, -5.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 100.0);
        assert!(hit.is_none(), "Triangle behind origin must not intersect");
    }

    // ── Cube BVH tests ──────────────────────────────────────────────────────

    /// Build a unit cube (side length 1, centred at the origin) from 12 triangles.
    fn unit_cube_bvh() -> Bvh {
        #[rustfmt::skip]
        let vertices: Vec<f32> = vec![
            // 8 corners of the unit cube
            -0.5, -0.5, -0.5, // 0
             0.5, -0.5, -0.5, // 1
             0.5,  0.5, -0.5, // 2
            -0.5,  0.5, -0.5, // 3
            -0.5, -0.5,  0.5, // 4
             0.5, -0.5,  0.5, // 5
             0.5,  0.5,  0.5, // 6
            -0.5,  0.5,  0.5, // 7
        ];
        #[rustfmt::skip]
        let indices: Vec<u32> = vec![
            // bottom (y = -0.5)
            0, 1, 5,  0, 5, 4,
            // top    (y =  0.5)
            3, 7, 6,  3, 6, 2,
            // front  (z =  0.5)
            4, 5, 6,  4, 6, 7,
            // back   (z = -0.5)
            1, 0, 3,  1, 3, 2,
            // left   (x = -0.5)
            0, 4, 7,  0, 7, 3,
            // right  (x =  0.5)
            5, 1, 2,  5, 2, 6,
        ];
        Bvh::build(&vertices, &indices)
    }

    #[test]
    fn test_bvh_cube_ray_hit_top_face() {
        let bvh = unit_cube_bvh();
        // Ray from above shooting straight down; top face is at y = 0.5.
        // Origin is at y = 2.0, so expected t = 2.0 - 0.5 = 1.5.
        let hit = bvh.cast_ray(Vec3::new(0.0, 2.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 100.0);
        assert!(hit.is_some(), "Ray aimed at cube top must hit");
        let t = hit.unwrap();
        assert!((t - 1.5).abs() < 1e-4, "Expected t≈1.5, got {t}");
    }

    #[test]
    fn test_bvh_cube_ray_miss() {
        let bvh = unit_cube_bvh();
        // Ray shooting upward from above the cube — misses entirely.
        let hit = bvh.cast_ray(Vec3::new(0.0, 2.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 100.0);
        assert!(hit.is_none(), "Ray pointing away from cube must miss");
    }

    #[test]
    fn test_bvh_cube_ray_beside_miss() {
        let bvh = unit_cube_bvh();
        // Ray beside the cube pointing downward — misses.
        let hit = bvh.cast_ray(Vec3::new(5.0, 2.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 100.0);
        assert!(hit.is_none(), "Ray beside cube must miss");
    }

    #[test]
    fn test_bvh_cube_ray_t_max_too_small() {
        let bvh = unit_cube_bvh();
        // Top face is at t = 1.5, but t_max = 1.0 — should miss.
        let hit = bvh.cast_ray(Vec3::new(0.0, 2.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 1.0);
        assert!(hit.is_none(), "t_max too small should produce no hit");
    }

    // ── Ray / Intersection struct tests ────────────────────────────────────

    #[test]
    fn test_ray_cast_hit() {
        let bvh = unit_cube_bvh();
        let ray = Ray::new(Vec3::new(0.0, 2.0, 0.0), Vec3::new(0.0, -1.0, 0.0));
        let isect = ray.cast(&bvh, 100.0);
        assert!(isect.is_some());
        let isect = isect.unwrap();
        assert!((isect.distance - 1.5).abs() < 1e-4, "Expected distance≈1.5, got {}", isect.distance);
    }

    #[test]
    fn test_ray_cast_miss() {
        let bvh = unit_cube_bvh();
        let ray = Ray::new(Vec3::new(0.0, 2.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
        assert!(ray.cast(&bvh, 100.0).is_none());
    }

    // ── Bvh::update (dynamic objects) ──────────────────────────────────────

    #[test]
    fn test_bvh_update_moves_geometry() {
        let (vertices, indices) = flat_box_mesh();
        let mut bvh = Bvh::build(&vertices, &indices);

        // Initial quad is at y = 0; ray from y = 5 hits at t = 5.
        let t_before = bvh.cast_ray(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 100.0);
        assert!(t_before.is_some());
        assert!((t_before.unwrap() - 5.0).abs() < 1e-4);

        // Move the quad to y = -2 and rebuild via update.
        let moved_vertices: Vec<f32> = vec![
            -1.0, -2.0, -1.0,
             1.0, -2.0, -1.0,
             1.0, -2.0,  1.0,
            -1.0, -2.0,  1.0,
        ];
        bvh.update(&moved_vertices, &indices);

        // Now the hit distance from y = 5 should be 7 (5 - (-2)).
        let t_after = bvh.cast_ray(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 100.0);
        assert!(t_after.is_some());
        assert!((t_after.unwrap() - 7.0).abs() < 1e-4, "Expected t≈7.0 after update, got {}", t_after.unwrap());
    }

    #[test]
    fn test_bvh_update_to_empty() {
        let (vertices, indices) = flat_box_mesh();
        let mut bvh = Bvh::build(&vertices, &indices);
        // Remove all geometry.
        bvh.update(&[], &[]);
        assert!(bvh.triangles.is_empty());
        assert!(bvh.cast_ray(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 100.0).is_none());
    }
}
