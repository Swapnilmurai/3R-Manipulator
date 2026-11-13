import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import loadMujoco from "@mujoco/mujoco";
import modelXml from "../models/three_r_browser.xml?raw";

const LENGTHS = [0.12, 0.10, 0.06];
const LIMITS = [[-2.967, 2.967], [-2.443, 2.443], [-2.967, 2.967]];
const clamp = (value, lo, hi) => Math.max(lo, Math.min(hi, value));

function fk(q) {
  const a1 = q[0], a2 = q[0] + q[1], a3 = a2 + q[2];
  return [
    LENGTHS[0] * Math.cos(a1) + LENGTHS[1] * Math.cos(a2) + LENGTHS[2] * Math.cos(a3),
    LENGTHS[0] * Math.sin(a1) + LENGTHS[1] * Math.sin(a2) + LENGTHS[2] * Math.sin(a3)
  ];
}

function jacobian(q) {
  const a = [q[0], q[0] + q[1], q[0] + q[1] + q[2]];
  const j = [[], []];
  for (let k = 0; k < 3; k++) {
    let dx = 0, dy = 0;
    for (let n = k; n < 3; n++) {
      dx -= LENGTHS[n] * Math.sin(a[n]);
      dy += LENGTHS[n] * Math.cos(a[n]);
    }
    j[0][k] = dx;
    j[1][k] = dy;
  }
  return j;
}

function solveIK(target, seed) {
  const q = seed.slice();
  for (let iteration = 0; iteration < 45; iteration++) {
    const p = fk(q);
    const ex = target[0] - p[0], ey = target[1] - p[1];
    if (Math.hypot(ex, ey) < 0.00025) break;
    const j = jacobian(q);
    const a = j[0].reduce((s, v) => s + v * v, 0) + 0.00004;
    const b = j[0].reduce((s, v, i) => s + v * j[1][i], 0);
    const c = j[1].reduce((s, v) => s + v * v, 0) + 0.00004;
    const det = a * c - b * b;
    if (Math.abs(det) < 1e-12) break;
    const vx = (c * ex - b * ey) / det;
    const vy = (-b * ex + a * ey) / det;
    const dq = j[0].map((v, i) => v * vx + j[1][i] * vy);
    const norm = Math.hypot(...dq);
    const scale = norm > 0.12 ? 0.12 / norm : 1;
    for (let i = 0; i < 3; i++) q[i] = clamp(q[i] + dq[i] * scale, LIMITS[i][0], LIMITS[i][1]);
  }
  return q;
}

function strokePoint(kind, u) {
  const cx = 0.252, cy = 0.0, r = 0.011;
  if (kind === "square") {
    const side = 8 * u;
    const corners = [[-r, -r], [r, -r], [r, r], [-r, r], [-r, -r]];
    const segment = Math.min(3, Math.floor(side));
    const alpha = side - segment;
    return [
      cx + corners[segment][0] * (1 - alpha) + corners[segment + 1][0] * alpha,
      cy + corners[segment][1] * (1 - alpha) + corners[segment + 1][1] * alpha
    ];
  }
  return [cx + r * Math.cos(2 * Math.PI * u), cy + r * Math.sin(2 * Math.PI * u)];
}

class BrowserMuJoCo {
  constructor() {
    this.canvasHost = document.getElementById("mujoco-viewer");
    this.status = document.getElementById("sim-status");
    this.timeLabel = document.getElementById("sim-time");
    this.positionLabel = document.getElementById("tool-position");
    this.jointLabels = [1, 2, 3].map(i => document.getElementById(`q${i}-value`));
    this.sliders = [1, 2, 3].map(i => document.getElementById(`q${i}-slider`));
    this.paused = false;
    this.mode = "manual";
    this.strokeStart = 0;
    this.strokeDuration = 7.0;
    this.strokeSeed = [0.4, -0.8, 0.2];
    this.lastTrailSample = -1;
    this.trailCapacity = 2200;
    this.trailCount = 0;
    this.geometries = new Map();
    this.renderMeshes = [];
    this.frameId = null;
  }

  async init() {
    this.setStatus("Loading MuJoCo WebAssembly…");
    this.mujoco = await loadMujoco({\n      locateFile: (path, prefix) => path.endsWith(".wasm") ? "/wasm/mujoco.wasm" : prefix + path\n    });
    this.model = this.mujoco.MjModel.from_xml_string(modelXml);
    if (!this.model) throw new Error("MuJoCo could not compile the 3R MJCF model.");
    this.data = new this.mujoco.MjData(this.model);
    this.option = new this.mujoco.MjvOption();
    this.perturb = new this.mujoco.MjvPerturb();
    this.cameraState = new this.mujoco.MjvCamera();
    this.sceneState = new this.mujoco.MjvScene(this.model, 2 ** 13);
    this.siteId = this.mujoco.mj_name2id(this.model, this.mujoco.mjtObj.mjOBJ_SITE.value, "tool0");
    if (this.siteId < 0) throw new Error("The tool0 site was not found in the MJCF.");

    this.setupRenderer();
    const initial = [0.4, -0.8, 0.2];
    this.data.qpos.set(initial);
    this.data.ctrl.set(initial);
    this.mujoco.mj_forward(this.model, this.data);
    this.setupTrail();
    this.bindControls();
    this.setStatus("MuJoCo WASM running in this browser");
    this.animate();
  }

  setupRenderer() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color("#0d1725");
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.canvasHost.replaceChildren(this.renderer.domElement);

    this.camera = new THREE.PerspectiveCamera(42, 1, 0.01, 10);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(0.32, -0.40, 0.34);
    this.camera.lookAt(0.13, 0, 0.01);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0.13, 0, 0.01);
    this.controls.enableDamping = true;
    this.controls.update();

    this.scene.add(new THREE.AmbientLight(0xffffff, 0.55));
    const key = new THREE.DirectionalLight(0xffffff, 2.2);
    key.position.set(0.2, -0.2, 0.6);
    key.castShadow = true;
    this.scene.add(key);
    const rim = new THREE.DirectionalLight(0x78bfe8, 0.9);
    rim.position.set(-0.3, 0.25, 0.3);
    this.scene.add(rim);

    const grid = new THREE.GridHelper(0.55, 28, 0x476078, 0x263a4e);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -0.005;
    this.scene.add(grid);

    this.resize();
    window.addEventListener("resize", () => this.resize());
  }

  resize() {
    if (!this.renderer || !this.canvasHost) return;
    const width = Math.max(320, this.canvasHost.clientWidth);
    const height = Math.max(240, this.canvasHost.clientHeight);
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  setupTrail() {
    const values = new Float32Array(this.trailCapacity * 3);
    this.trailGeometry = new THREE.BufferGeometry();
    this.trailGeometry.setAttribute("position", new THREE.BufferAttribute(values, 3));
    this.trailGeometry.setDrawRange(0, 0);
    this.trailLine = new THREE.Line(
      this.trailGeometry,
      new THREE.LineBasicMaterial({ color: 0x62d6b1, linewidth: 2 })
    );
    this.trailLine.frustumCulled = false;
    this.scene.add(this.trailLine);
  }

  clearTrail() {
    this.trailCount = 0;
    this.lastTrailSample = -1;
    this.trailGeometry.setDrawRange(0, 0);
    this.trailGeometry.attributes.position.needsUpdate = true;
  }

  recordTrail() {
    if (this.mode !== "circle" && this.mode !== "square") return;
    const t = Number(this.data.time);
    if (t - this.lastTrailSample < 0.018) return;
    const p = this.data.site_xpos;
    if (this.trailCount >= this.trailCapacity) return;
    const attr = this.trailGeometry.attributes.position;
    attr.setXYZ(this.trailCount, p[this.siteId * 3], p[this.siteId * 3 + 1], p[this.siteId * 3 + 2] + 0.001);
    this.trailCount++;
    this.lastTrailSample = t;
    this.trailGeometry.setDrawRange(0, this.trailCount);
    attr.needsUpdate = true;
  }

  geometryFor(geom) {
    const type = Number(geom.type);
    const s = Array.from(geom.size).map(Number);
    const key = JSON.stringify([type, s, Number(geom.dataid)]);
    if (this.geometries.has(key)) return this.geometries.get(key);

    const mjt = this.mujoco.mjtGeom;
    let g;
    if (type === mjt.mjGEOM_PLANE.value) {
      g = new THREE.PlaneGeometry(2 * (s[0] || 100), 2 * (s[1] || 100));
    } else if (type === mjt.mjGEOM_SPHERE.value) {
      g = new THREE.SphereGeometry(s[0], 20, 14);
    } else if (type === mjt.mjGEOM_CAPSULE.value) {
      g = new THREE.CapsuleGeometry(s[0], Math.max(0.001, 2 * s[2]), 6, 16);
      g.rotateX(Math.PI / 2);
    } else if (type === mjt.mjGEOM_CYLINDER.value) {
      g = new THREE.CylinderGeometry(s[0], s[1], 2 * s[2], 24);
      g.rotateX(Math.PI / 2);
    } else if (type === mjt.mjGEOM_BOX.value) {
      g = new THREE.BoxGeometry(2 * s[0], 2 * s[1], 2 * s[2]);
    } else if (type === mjt.mjGEOM_ELLIPSOID.value) {
      g = new THREE.SphereGeometry(1, 20, 14);
      g.scale(s[0], s[1], s[2]);
    } else {
      g = new THREE.SphereGeometry(0.001, 6, 6);
    }
    this.geometries.set(key, g);
    return g;
  }

  renderMuJoCoScene() {
    this.mujoco.mjv_updateScene(
      this.model, this.data, this.option, this.perturb,
      this.cameraState, this.mujoco.mjtCatBit.mjCAT_ALL.value, this.sceneState
    );

    const geoms = this.sceneState.geoms;
    const count = Number(geoms.size());
    for (let i = 0; i < count; i++) {
      const g = geoms.get(i);
      const rgba = Array.from(g.rgba).map(Number);
      const mat = Array.from(g.mat).map(Number);
      const pos = Array.from(g.pos).map(Number);
      let mesh = this.renderMeshes[i];
      if (!mesh) {
        mesh = new THREE.Mesh(this.geometryFor(g), new THREE.MeshPhongMaterial());
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        this.renderMeshes[i] = mesh;
        this.scene.add(mesh);
      }
      const material = mesh.material;
      material.color.setRGB(rgba[0], rgba[1], rgba[2]);
      material.opacity = rgba[3];
      material.transparent = rgba[3] < 0.999;
      material.depthWrite = rgba[3] >= 0.999;
      mesh.matrixAutoUpdate = false;
      mesh.matrix.set(
        mat[0], mat[1], mat[2], pos[0],
        mat[3], mat[4], mat[5], pos[1],
        mat[6], mat[7], mat[8], pos[2],
        0, 0, 0, 1
      );
      mesh.matrixWorldNeedsUpdate = true;
      g.delete();
    }
    geoms.delete();

    for (let i = count; i < this.renderMeshes.length; i++) {
      this.renderMeshes[i].visible = false;
    }
    for (let i = 0; i < count; i++) this.renderMeshes[i].visible = true;
  }

  setStatus(message) {
    if (this.status) this.status.textContent = message;
  }

  updateReadout() {
    const q = this.data.qpos;
    const p = this.data.site_xpos;
    const x = p[this.siteId * 3];
    const y = p[this.siteId * 3 + 1];
    const z = p[this.siteId * 3 + 2];
    this.timeLabel.textContent = `${Number(this.data.time).toFixed(2)} s`;
    this.positionLabel.textContent = `X ${(x * 1000).toFixed(1)} · Y ${(y * 1000).toFixed(1)} · Z ${(z * 1000).toFixed(1)} mm`;
    for (let i = 0; i < 3; i++) {
      const value = Number(q[i]);
      this.jointLabels[i].textContent = `${(value * 180 / Math.PI).toFixed(1)}°`;
      if (document.activeElement !== this.sliders[i]) {
        this.sliders[i].value = value.toFixed(3);
      }
    }
  }

  setManualTarget(index, value) {
    this.mode = "manual";
    this.strokeSeed = this.data.qpos.slice(0, 3);
    this.data.ctrl[index] = clamp(Number(value), LIMITS[index][0], LIMITS[index][1]);
    this.setStatus("Manual joint-position control · MuJoCo WASM");
  }

  startStroke(kind) {
    this.mode = kind;
    this.strokeStart = Number(this.data.time);
    this.strokeSeed = Array.from(this.data.qpos).slice(0, 3);
    this.strokeDuration = kind === "circle" ? 7.0 : 8.0;
    this.clearTrail();
    this.paused = false;
    this.setStatus(`Drawing a ${kind} path using DLS joint targets`);
    this.setPauseButton();
  }

  setPauseButton() {
    const button = document.getElementById("pause-sim");
    if (button) button.textContent = this.paused ? "Resume" : "Pause";
  }

  bindControls() {
    document.getElementById("pause-sim").addEventListener("click", () => {
      this.paused = !this.paused;
      this.setPauseButton();
      this.setStatus(this.paused ? "Paused · physics state retained" : "MuJoCo WASM running in this browser");
    });
    document.getElementById("reset-sim").addEventListener("click", () => {
      this.mode = "manual";
      this.paused = false;
      this.mujoco.mj_resetData(this.model, this.data);
      this.data.qpos.set([0.4, -0.8, 0.2]);
      this.data.ctrl.set([0.4, -0.8, 0.2]);
      this.mujoco.mj_forward(this.model, this.data);
      this.clearTrail();
      this.setPauseButton();
      this.setStatus("Reset to the nominal three-link pose");
    });
    document.getElementById("draw-circle").addEventListener("click", () => this.startStroke("circle"));
    document.getElementById("draw-square").addEventListener("click", () => this.startStroke("square"));
    this.sliders.forEach((slider, index) => {
      slider.addEventListener("input", event => this.setManualTarget(index, event.target.value));
    });
  }

  updateController() {
    if (this.mode !== "circle" && this.mode !== "square") return;
    const u = (Number(this.data.time) - this.strokeStart) / this.strokeDuration;
    if (u >= 1) {
      this.mode = "manual";
      this.setStatus("Stroke complete · MuJoCo simulation continues");
      return;
    }
    const target = strokePoint(this.mode, Math.max(0, u));
    this.strokeSeed = solveIK(target, this.strokeSeed);
    for (let i = 0; i < 3; i++) this.data.ctrl[i] = this.strokeSeed[i];
  }

  animate() {
    if (this.frameId !== null) return;
    const tick = () => {
      try {
        if (!this.paused) {
          const start = Number(this.data.time);
          const targetTime = start + 1 / 60;
          let steps = 0;
          while (Number(this.data.time) < targetTime && steps < 12) {
            this.updateController();
            this.mujoco.mj_step(this.model, this.data);
            steps++;
          }
          this.recordTrail();
        }
        this.renderMuJoCoScene();
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
        this.updateReadout();
      } catch (error) {
        console.error("MuJoCo browser simulation error:", error);
        this.setStatus("Simulation stopped · see diagnostic message");
      }
      this.frameId = requestAnimationFrame(tick);
    };
    this.frameId = requestAnimationFrame(tick);
  }
}

async function start() {
  const sim = new BrowserMuJoCo();
  try {
    await sim.init();
    window.addEventListener("beforeunload", () => {
      if (sim.frameId !== null) cancelAnimationFrame(sim.frameId);
      sim.sceneState?.delete();
      sim.cameraState?.delete();
      sim.perturb?.delete();
      sim.option?.delete();
      sim.data?.delete();
      sim.model?.delete();
      sim.controls?.dispose();
      sim.renderer?.dispose();
      for (const geometry of sim.geometries.values()) geometry.dispose();
    }, { once: true });
  } catch (error) {
    console.error(error);
    sim.setStatus("MuJoCo could not initialize");
    const detail = document.getElementById("sim-error");
    if (detail) detail.textContent = error?.message || String(error);
  }
}

start();
