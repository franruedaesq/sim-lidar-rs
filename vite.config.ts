import { defineConfig } from "vite";
import wasm from "vite-plugin-wasm";

export default defineConfig({
  plugins: [wasm()],
  build: {
    lib: {
      entry: "ts/src/index.ts",
      name: "SimLidarRs",
      fileName: "sim-lidar-rs",
      formats: ["es"],
    },
    rollupOptions: {
      external: [],
    },
  },
  test: {
    environment: "node",
    include: ["ts/src/__tests__/**/*.test.ts"],
  },
});
