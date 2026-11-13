import "./style.css";

const app = document.querySelector("#app");
app.innerHTML = `
<header class="top">
  <div class="brand"><span class="mark">3R</span><div><b>MANIPULATOR LAB</b><small>CAD-derived digital twin · Browser simulation</small></div></div>
  <div class="head-right"><span class="pill">MuJoCo 3.17 · WebAssembly</span><a href="https://github.com/Swapnilmurai/3R-Manipulator" target="_blank" rel="noreferrer">GitHub ↗</a></div>
</header>
<main>
  <section class="hero">
    <div>
      <p class="eyebrow">PLANAR ROBOTICS · WRITING AUTOMATION</p>
      <h1>Teach a robot<br><em>to write.</em></h1>
      <p class="lead">A live MuJoCo physics simulation running in your browser with WebAssembly. Adjust the three joints or command a circle and square drawing stroke.</p>
      <div class="dimension-row"><span><b>120 mm</b><small>Link 1</small></span><span><b>100 mm</b><small>Link 2</small></span><span><b>60 mm</b><small>Link 3</small></span></div>
    </div>
    <div class="hero-note"><span class="live-dot"></span><div><b>Physics runs in this tab</b><small>MuJoCo WebAssembly · 2 ms model timestep</small></div></div>
  </section>

  <section class="sim-layout">
    <div class="viewer-card">
      <div class="viewer-head"><div><span class="eyebrow">LIVE DIGITAL TWIN</span><h2>3R manipulator simulation</h2></div><span id="sim-status" class="sim-status">Loading MuJoCo WebAssembly…</span></div>
      <div id="mujoco-viewer" class="viewer"><div class="viewer-loading">Initializing MuJoCo physics engine…</div></div>
      <div class="viewer-foot"><span>Drag to orbit · Scroll to zoom</span><span>XY planar motion · Z-up</span></div>
      <div id="sim-error" class="sim-error" role="status"></div>
    </div>

    <aside class="control-card">
      <div class="control-title"><div><span class="eyebrow">SIMULATION CONTROL</span><h2>Experiment panel</h2></div><span class="engine-badge">WASM</span></div>
      <div class="transport">
        <button id="pause-sim" class="primary">Pause</button>
        <button id="reset-sim" class="secondary">Reset</button>
      </div>
      <div class="stroke-block">
        <label>DRAWING DEMONSTRATION</label>
        <div class="stroke-actions"><button id="draw-circle">Draw circle</button><button id="draw-square">Draw square</button></div>
        <small>Uses a DLS inverse-kinematics target generator to drive the MuJoCo position actuators.</small>
      </div>
      <div class="readouts">
        <div><small>SIMULATION TIME</small><b id="sim-time">0.00 s</b></div>
        <div><small>TOOL POSITION</small><b id="tool-position">—</b></div>
      </div>
      <div class="joint-controls">
        <div class="joint-heading"><label>JOINT POSITION TARGETS</label><small>Radians</small></div>
        <div class="joint-row"><div><b>J1</b><span id="q1-value">0.0°</span></div><input id="q1-slider" type="range" min="-2.967" max="2.967" step="0.01" value="0.4"/></div>
        <div class="joint-row"><div><b>J2</b><span id="q2-value">0.0°</span></div><input id="q2-slider" type="range" min="-2.443" max="2.443" step="0.01" value="-0.8"/></div>
        <div class="joint-row"><div><b>J3</b><span id="q3-value">0.0°</span></div><input id="q3-slider" type="range" min="-2.967" max="2.967" step="0.01" value="0.2"/></div>
      </div>
      <div class="note-block"><b>Model status</b><p>This embedded MJCF uses CAD-measured pivot spacings and estimated inertias. Link visuals and collision geometry are simplified capsules; the full STL-mesh MJCF and URDF remain in the repository for native simulation.</p></div>
    </aside>
  </section>

  <section class="metrics">
    <article><small>DEGREES OF FREEDOM</small><b>03</b><span>Revolute joints</span></article>
    <article><small>PHYSICS ENGINE</small><b>MuJoCo</b><span>Official JavaScript WASM bindings</span></article>
    <article><small>CONTROL BASELINE</small><b>DLS IK</b><span>Cartesian path to joint targets</span></article>
    <article><small>RESEARCH GOAL</small><b>Writing AI</b><span>Trajectory learning and validation</span></article>
  </section>

  <section class="roadmap">
    <div class="section-head"><div><p class="eyebrow">RESEARCH PIPELINE</p><h2>From simulation to handwriting.</h2></div><p>The browser demo is a simulation and visualization stage—not a hardware controller.</p></div>
    <div class="cards">
      <article><span>01 / GEOMETRY</span><h3>CAD assembly</h3><p>Five STL components inspected; the link pivots measure approximately 120 / 100 / 60 mm.</p><b class="status complete">Meshes inspected</b></article>
      <article><span>02 / DIGITAL TWIN</span><h3>MuJoCo + ROS 2</h3><p>CAD-mesh MJCF and URDF/Xacro descriptions are maintained for native simulation and RViz.</p><b class="status complete">Model scaffold committed</b></article>
      <article><span>03 / TRAJECTORIES</span><h3>Letters and shapes</h3><p>Generate vector strokes, solve IK, and measure tracking, continuity, and smoothness.</p><b class="status active">DLS demo enabled</b></article>
      <article><span>04 / LEARNING</span><h3>Train to write</h3><p>Build trajectory datasets, compare ANN and hybrid approaches, then validate on hardware.</p><b class="status next">Next research stage</b></article>
    </div>
  </section>
  <footer>3R Manipulator Research Workspace <span>Browser MuJoCo physics · no robot hardware connection</span></footer>
</main>`;


// Load the simulator only after its DOM elements have been inserted.
import("./mujoco-sim.js").catch((error) => {
  console.error("Could not load the MuJoCo simulator module:", error);
  const status = document.getElementById("sim-status");
  const detail = document.getElementById("sim-error");
  if (status) status.textContent = "Simulator module failed to load";
  if (detail) detail.textContent = error?.message || String(error);
});
