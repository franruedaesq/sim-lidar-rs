import { defineConfig } from "vite";
import wasm from "vite-plugin-wasm";

export default defineConfig({
  plugins: [wasm()],
  build: {
    lib: {
      entry: {
        "sim-lidar-rs": "ts/src/index.ts",
        "three": "ts/src/three.ts",
      },
      formats: ["es"],
    },
    rollupOptions: {
      external: ["three"],
    },
  },
  test: {
    environment: "node",
    include: ["ts/src/__tests__/**/*.test.ts"],
  },
});
