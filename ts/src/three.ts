/**
 * sim-lidar-rs/three – Optional Three.js adapter
 *
 * This module does **not** import from the `three` package at runtime, so it
 * carries zero extra bundle weight for projects that do not use Three.js.
 * The exported helpers use structural typing to accept real Three.js objects.
 *
 * @example
 * ```ts
 * import * as THREE from 'three';
 * import { extractGeometry, hitsToPoints } from 'sim-lidar-rs/three';
 * import { SimLidar, VLP16_CONFIG } from 'sim-lidar-rs';
 *
 * // ── Build the LiDAR environment from your scene ──────────────────────────
 * scene.updateMatrixWorld();
 * const geometry = extractGeometry(scene);
 *
 * const lidar = new SimLidar(VLP16_CONFIG);
 * await lidar.init();
 * await lidar.updateEnvironment(geometry);
 *
 * // ── Visualise scan results in the same scene ──────────────────────────────
 * const hits = await lidar.scan({ position: { x: 0, y: 1.5, z: 0 } });
 * const points = hitsToPoints(hits, THREE);
 * scene.add(points);
 * ```
 */

import type { Geometry } from "./types.js";

// ─── Minimal structural interfaces ───────────────────────────────────────────
// These mirror the Three.js API surface we need without importing from 'three'.

interface BufferAttribute {
  /** Underlying flat typed array (e.g. Float32Array, Uint32Array). */
  readonly array: ArrayLike<number>;
  /** Number of elements (vertices / indices). */
  readonly count: number;
  /** Components per element (3 for position, 1 for scalar index). */
  readonly itemSize: number;
}

interface BufferGeometry {
  readonly attributes: Readonly<Record<string, BufferAttribute | undefined>>;
  /** `null` for non-indexed geometries. */
  readonly index: BufferAttribute | null;
}

interface Matrix4Like {
  /** Column-major 16-element array, matching `THREE.Matrix4.elements`. */
  readonly elements: ArrayLike<number>;
}

/** Structural type covering both `THREE.Scene` and `THREE.Mesh`. */
interface Object3D {
  /** `true` for `THREE.Mesh` instances. */
  readonly isMesh?: boolean;
  /** Present on mesh objects. */
  readonly geometry?: BufferGeometry;
  /** Child objects in the scene graph. */
  readonly children: readonly Object3D[];
  /**
   * World-space transformation matrix.
   * Call `scene.updateMatrixWorld()` before passing the scene to ensure
   * matrices are current.
   */
  readonly matrixWorld?: Matrix4Like;
}

// ─── extractGeometry ─────────────────────────────────────────────────────────

/**
 * Recursively traverse a `THREE.Scene` or `THREE.Mesh` and merge every mesh's
 * vertex positions and triangle indices into a single flat {@link Geometry}
 * ready for {@link SimLidar.updateEnvironment} / {@link LidarClient}.
 *
 * - **Matrix transforms**: if a mesh exposes a `matrixWorld` property its
 *   positions are converted to world space (column-major, Three.js convention).
 *   Call `scene.updateMatrixWorld()` before calling this function.
 * - **Non-indexed geometries**: auto-generated sequential indices are added.
 * - Multiple meshes are merged into a single vertex/index buffer pair.
 *
 * @param object - A `THREE.Scene`, `THREE.Mesh`, or any object that structurally
 *                 matches the {@link Object3D} interface.
 * @returns Flat {@link Geometry} with `vertices` and `indices` buffers.
 *
 * @example
 * ```ts
 * scene.updateMatrixWorld();
 * const { vertices, indices } = extractGeometry(scene);
 * await lidar.updateEnvironment({ vertices, indices });
 * ```
 */
export function extractGeometry(object: Object3D): Geometry {
  const allVertices: number[] = [];
  const allIndices: number[] = [];

  function visit(obj: Object3D): void {
    if (obj.isMesh && obj.geometry) {
      const posAttr = obj.geometry.attributes["position"];
      if (posAttr) {
        const baseVertex = allVertices.length / 3;
        const mw = obj.matrixWorld?.elements;
        const posArray = posAttr.array as number[];

        for (let i = 0; i < posAttr.count; i++) {
          const base = i * posAttr.itemSize;
          let x = posArray[base];
          let y = posArray[base + 1];
          let z = posArray[base + 2];

          // Apply world matrix (column-major, Three.js convention):
          //   [ m[0]  m[4]  m[8]  m[12] ]
          //   [ m[1]  m[5]  m[9]  m[13] ]
          //   [ m[2]  m[6]  m[10] m[14] ]
          //   [ m[3]  m[7]  m[11] m[15] ]
          if (mw) {
            const ox = x, oy = y, oz = z;
            x = mw[0] * ox + mw[4] * oy + mw[8]  * oz + mw[12];
            y = mw[1] * ox + mw[5] * oy + mw[9]  * oz + mw[13];
            z = mw[2] * ox + mw[6] * oy + mw[10] * oz + mw[14];
          }

          allVertices.push(x, y, z);
        }

        if (obj.geometry.index) {
          const idx = obj.geometry.index;
          const idxArray = idx.array as number[];
          for (let i = 0; i < idx.count; i++) {
            allIndices.push(baseVertex + idxArray[i]);
          }
        } else {
          // Non-indexed: generate sequential indices (one per vertex).
          for (let i = 0; i < posAttr.count; i++) {
            allIndices.push(baseVertex + i);
          }
        }
      }
    }

    for (const child of obj.children) {
      visit(child);
    }
  }

  visit(object);

  return {
    vertices: new Float32Array(allVertices),
    indices: new Uint32Array(allIndices),
  };
}

// ─── hitsToPoints ────────────────────────────────────────────────────────────

/**
 * Convert a flat `Float32Array` of `[x,y,z, x,y,z, …]` hit coordinates (as
 * returned by {@link SimLidar.scan} or {@link LidarClient.scan}) into a
 * `THREE.Points` object for immediate browser visualisation.
 *
 * The Three.js namespace is accepted as the second argument so this helper
 * does **not** carry a hard dependency on the `three` npm package.
 * TypeScript will infer the return type as `THREE.Points` when you pass the
 * full `THREE` namespace.
 *
 * @param hits - Flat `Float32Array` of hit coordinates `[x,y,z, …]`.
 * @param three - An object exposing `BufferGeometry`, `Float32BufferAttribute`,
 *                and `Points` constructors (pass `import * as THREE from 'three'`).
 * @returns A `THREE.Points` instance ready to be added to a scene.
 *
 * @example
 * ```ts
 * import * as THREE from 'three';
 * import { hitsToPoints } from 'sim-lidar-rs/three';
 *
 * const hits = await lidar.scan(pose);
 * const points = hitsToPoints(hits, THREE);
 * scene.add(points);
 * ```
 */
export function hitsToPoints<TPoints = unknown>(
  hits: Float32Array,
  three: {
    BufferGeometry: new () => { setAttribute(name: string, attribute: unknown): void };
    Float32BufferAttribute: new (array: ArrayLike<number>, itemSize: number) => unknown;
    Points: new (geometry: unknown, material?: unknown) => TPoints;
  }
): TPoints {
  const geometry = new three.BufferGeometry();
  geometry.setAttribute("position", new three.Float32BufferAttribute(hits, 3));
  return new three.Points(geometry);
}
