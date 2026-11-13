import { copyFile, mkdir, stat } from "node:fs/promises";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const root = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const source = resolve(root, "node_modules/@mujoco/mujoco/dist/mujoco.wasm");
const destination = resolve(root, "public/wasm/mujoco.wasm");

try {
  const info = await stat(source);
  if (!info.isFile() || info.size < 100_000) {
    throw new Error("The MuJoCo WASM binary looks missing or unexpectedly small.");
  }
  await mkdir(dirname(destination), { recursive: true });
  await copyFile(source, destination);
  console.log(`Copied MuJoCo WASM (${info.size.toLocaleString()} bytes) to public/wasm/mujoco.wasm`);
} catch (error) {
  console.error("Could not stage MuJoCo WebAssembly asset.", error);
  process.exitCode = 1;
}
