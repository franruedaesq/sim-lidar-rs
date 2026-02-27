/**
 * Unit tests for the optional Three.js adapter (`ts/src/three.ts`).
 *
 * All tests use plain JavaScript objects that structurally match the
 * Three.js API – no actual `three` npm package is required.
 */
import { describe, it, expect } from "vitest";
import { extractGeometry, hitsToPoints } from "../three.js";

// ─── Helpers ─────────────────────────────────────────────────────────────────

/** Build a minimal BufferAttribute from a flat array. */
function makeAttr(array: number[], itemSize: number) {
  return { array, count: array.length / itemSize, itemSize };
}

/** Build a minimal indexed mesh object. */
function makeMesh(positions: number[], indices?: number[]) {
  return {
    isMesh: true,
    geometry: {
      attributes: { position: makeAttr(positions, 3) },
      index: indices ? makeAttr(indices, 1) : null,
    },
    children: [],
  };
}

type AnyObject3D = {
  isMesh?: boolean;
  geometry?: ReturnType<typeof makeMesh>["geometry"];
  children: readonly AnyObject3D[];
  matrixWorld?: { elements: number[] };
};

/** Build a minimal scene-like container. */
function makeScene(children: AnyObject3D[]): AnyObject3D {
  return { children };
}

// Identity 4×4 matrix (column-major, matching Three.js convention).
const IDENTITY_MATRIX = [
  1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, 1, 0,
  0, 0, 0, 1,
];

// ─── extractGeometry ─────────────────────────────────────────────────────────

describe("extractGeometry", () => {
  it("returns empty buffers for an empty scene", () => {
    const geo = extractGeometry(makeScene([]));
    expect(geo.vertices).toBeInstanceOf(Float32Array);
    expect(geo.indices).toBeInstanceOf(Uint32Array);
    expect(geo.vertices.length).toBe(0);
    expect(geo.indices.length).toBe(0);
  });

  it("extracts vertices and indices from an indexed mesh", () => {
    // Two-triangle quad in the XZ plane.
    const positions = [
      -1, 0, -1,
       1, 0, -1,
       1, 0,  1,
      -1, 0,  1,
    ];
    const indices = [0, 1, 2, 0, 2, 3];

    const geo = extractGeometry(makeMesh(positions, indices));

    expect(geo.vertices).toBeInstanceOf(Float32Array);
    expect(geo.indices).toBeInstanceOf(Uint32Array);
    expect(geo.vertices.length).toBe(12); // 4 vertices × 3 components
    expect(geo.indices.length).toBe(6);

    // Vertices should match input exactly.
    expect(Array.from(geo.vertices)).toEqual(positions);
    // Indices should match input exactly.
    expect(Array.from(geo.indices)).toEqual(indices);
  });

  it("auto-generates sequential indices for a non-indexed mesh", () => {
    // One triangle (3 vertices, no index buffer).
    const positions = [0, 0, 0, 1, 0, 0, 0, 0, 1];

    const geo = extractGeometry(makeMesh(positions)); // no indices

    expect(geo.vertices.length).toBe(9);  // 3 × 3
    expect(geo.indices.length).toBe(3);   // one index per vertex
    expect(Array.from(geo.indices)).toEqual([0, 1, 2]);
  });

  it("merges multiple meshes in a scene into one geometry", () => {
    const mesh1 = makeMesh([0, 0, 0, 1, 0, 0, 0, 0, 1], [0, 1, 2]);
    const mesh2 = makeMesh([2, 0, 0, 3, 0, 0, 2, 0, 1], [0, 1, 2]);

    const geo = extractGeometry(makeScene([mesh1, mesh2]));

    expect(geo.vertices.length).toBe(18); // 6 vertices × 3
    expect(geo.indices.length).toBe(6);   // 2 triangles × 3

    // mesh2 indices should be offset by 3 (baseVertex of mesh2).
    expect(Array.from(geo.indices)).toEqual([0, 1, 2, 3, 4, 5]);
  });

  it("recursively traverses nested children", () => {
    const leaf = makeMesh([0, 0, 0, 1, 0, 0, 0, 0, 1], [0, 1, 2]);
    const inner = { ...makeScene([leaf]), isMesh: false };
    const root = makeScene([inner]);

    const geo = extractGeometry(root);
    expect(geo.vertices.length).toBe(9);
    expect(geo.indices.length).toBe(3);
  });

  it("ignores non-mesh objects (isMesh = false / missing)", () => {
    // A group-like node with no geometry.
    const group = { children: [], isMesh: false };
    const geo = extractGeometry(group);
    expect(geo.vertices.length).toBe(0);
    expect(geo.indices.length).toBe(0);
  });

  it("applies an identity matrixWorld without changing vertex positions", () => {
    const positions = [1, 2, 3, 4, 5, 6];
    const mesh = {
      isMesh: true,
      geometry: {
        attributes: { position: makeAttr(positions, 3) },
        index: makeAttr([0, 1], 1),
      },
      children: [],
      matrixWorld: { elements: IDENTITY_MATRIX },
    };

    const geo = extractGeometry(mesh);
    expect(Array.from(geo.vertices)).toEqual(positions);
  });

  it("applies a translation matrixWorld to vertex positions", () => {
    // Translation matrix that moves by (10, 20, 30).
    const translationMatrix = [
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      10, 20, 30, 1,
    ];
    const positions = [1, 2, 3];
    const mesh = {
      isMesh: true,
      geometry: {
        attributes: { position: makeAttr(positions, 3) },
        index: null,
      },
      children: [],
      matrixWorld: { elements: translationMatrix },
    };

    const geo = extractGeometry(mesh);
    // Expected: (1+10, 2+20, 3+30) = (11, 22, 33)
    expect(geo.vertices[0]).toBeCloseTo(11);
    expect(geo.vertices[1]).toBeCloseTo(22);
    expect(geo.vertices[2]).toBeCloseTo(33);
  });

  it("applies a uniform scale matrixWorld to vertex positions", () => {
    const scale = 2;
    const scaleMatrix = [
      scale, 0, 0, 0,
      0, scale, 0, 0,
      0, 0, scale, 0,
      0, 0, 0, 1,
    ];
    const positions = [1, 2, 3];
    const mesh = {
      isMesh: true,
      geometry: {
        attributes: { position: makeAttr(positions, 3) },
        index: null,
      },
      children: [],
      matrixWorld: { elements: scaleMatrix },
    };

    const geo = extractGeometry(mesh);
    expect(geo.vertices[0]).toBeCloseTo(2);
    expect(geo.vertices[1]).toBeCloseTo(4);
    expect(geo.vertices[2]).toBeCloseTo(6);
  });
});

// ─── hitsToPoints ────────────────────────────────────────────────────────────

describe("hitsToPoints", () => {
  it("calls BufferGeometry.setAttribute with 'position' and the correct attribute", () => {
    const hits = new Float32Array([1, 2, 3, 4, 5, 6]);

    let capturedName: string | undefined;
    let capturedAttr: unknown;

    // Minimal mock of THREE constructors.
    const mockTHREE = {
      BufferGeometry: class {
        setAttribute(name: string, attr: unknown) {
          capturedName = name;
          capturedAttr = attr;
        }
      },
      Float32BufferAttribute: class {
        constructor(public array: ArrayLike<number>, public itemSize: number) {}
      },
      Points: class {
        constructor(public geometry: unknown) {}
      },
    };

    const result = hitsToPoints(hits, mockTHREE);

    expect(capturedName).toBe("position");
    // The attribute should be a Float32BufferAttribute with our hits data.
    const attr = capturedAttr as { array: ArrayLike<number>; itemSize: number };
    expect(attr.array).toBe(hits);
    expect(attr.itemSize).toBe(3);

    // The returned object should be a Points instance holding the geometry.
    expect(result).toBeInstanceOf(mockTHREE.Points);
  });

  it("returns an instance created by the provided Points constructor", () => {
    const hits = new Float32Array([0, 0, 0]);

    class FakeGeometry {
      setAttribute() {}
    }
    class FakeAttr {
      constructor(public array: ArrayLike<number>, public itemSize: number) {}
    }
    class FakePoints {
      constructor(public geometry: unknown) {}
    }

    const result = hitsToPoints(hits, {
      BufferGeometry: FakeGeometry,
      Float32BufferAttribute: FakeAttr,
      Points: FakePoints,
    });

    expect(result).toBeInstanceOf(FakePoints);
    expect((result as FakePoints).geometry).toBeInstanceOf(FakeGeometry);
  });

  it("passes hits as-is to Float32BufferAttribute (no copy)", () => {
    const hits = new Float32Array([7, 8, 9]);
    let capturedArray: ArrayLike<number> | undefined;

    const mockTHREE = {
      BufferGeometry: class {
        setAttribute() {}
      },
      Float32BufferAttribute: class {
        constructor(array: ArrayLike<number>, _itemSize: number) {
          capturedArray = array;
        }
      },
      Points: class {
        constructor(_geo: unknown) {}
      },
    };

    hitsToPoints(hits, mockTHREE);
    // Same reference – no copy should be made.
    expect(capturedArray).toBe(hits);
  });
});
