import { defineConfig } from "vite";

export default defineConfig({
  base: "/",
  optimizeDeps: { exclude: ["@mujoco/mujoco"] },
  assetsInclude: ["**/*.wasm"],
  build: { target: "esnext", assetsInlineLimit: 0 }
});
