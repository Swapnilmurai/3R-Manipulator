import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import loadMujoco from "@mujoco/mujoco";
import modelXml from "../models/three_r_browser.xml?raw";

const LENGTHS=[0.12,0.10,0.06];
const LIMITS=[[-2.967,2.967],[-2.443,2.443],[-2.967,2.967]];
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const mat=(color,metalness=0.2,roughness=0.58)=>new THREE.MeshStandardMaterial({color,metalness,roughness});
const M={base:mat("#596574",0.62),lid:mat("#8493a3",0.42),blue:mat("#367fad",0.45),teal:mat("#3297a0",0.4),gold:mat("#d69b4a",0.35),motor:mat("#313a44",0.42),gear:mat("#b7c1c9",0.65),shaft:mat("#dce3e9",0.78,0.24),dark:mat("#202a33",0.25),pen:mat("#20242a",0.28)};
function box(parent,name,size,pos,material){const mesh=new THREE.Mesh(new THREE.BoxGeometry(...size),material);mesh.name=name;mesh.position.set(...pos);mesh.castShadow=true;mesh.receiveShadow=true;parent.add(mesh);return mesh;}
function cyl(parent,name,r,h,pos,material,segments=40){const mesh=new THREE.Mesh(new THREE.CylinderGeometry(r,r,h,segments),material);mesh.name=name;mesh.rotation.x=Math.PI/2;mesh.position.set(...pos);mesh.castShadow=true;mesh.receiveShadow=true;parent.add(mesh);return mesh;}
function motor(parent,name){
  const g=new THREE.Group();g.name=name;parent.add(g);
  box(g,name+"_mount_flange",[0.035,0.040,0.002],[-0.008,0,-0.001],M.gear);
  cyl(g,name+"_gearbox",0.014,0.013,[0,0,-0.0085],M.gear);
  cyl(g,name+"_stator",0.0132,0.018,[0,0,-0.024],M.motor);
  box(g,name+"_connector",[0.009,0.012,0.011],[-0.015,0,-0.022],M.dark);
  for(const y of [-0.0175,0.0175]){
    cyl(g,name+"_mount_screw",0.0022,0.0018,[-0.008,y,0.0004],M.dark,20);
  }
  g.userData.spec="28BYJ-48 5V envelope; dimensional visual approximation";
  return g;
}
function linkMesh(parent,name,length,width,thickness,material,xStart=0,z=0){
  const center=xStart+length/2;
  box(parent,name+"_plate",[length,width,thickness],[center,0,z],material);
  box(parent,name+"_proximal_hub",[0.024,0.024,thickness*1.35],[xStart,0,z],M.gear);
}
function makeAssembly(scene){
  const root=new THREE.Group();root.name="3R_CAD_derived_28BYJ48_assembly";scene.add(root);
  cyl(root,"base_ring",0.070,0.036,[0,0,0.018],M.base,64);
  cyl(root,"base_lid",0.070,0.007,[0,0,0.0395],M.lid,64);
  cyl(root,"base_joint_plate",0.026,0.004,[0,0,0.0445],M.gear,48);
  motor(root,"motor1_28BYJ48");
  const j1=new THREE.Group();j1.name="link1";j1.position.set(0,0,0.043);root.add(j1);
  linkMesh(j1,"link1",0.150,0.048,0.008,M.blue,0,-0.004);
  cyl(j1,"joint1_hub",0.013,0.009,[0,0,0],M.gear,40);
  cyl(j1,"joint1_motor_shaft",0.0025,0.010,[0,0,-0.002],M.shaft,24);
  motor(j1,"motor2_28BYJ48").position.set(0.120,0,0);
  const j2=new THREE.Group();j2.name="link2";j2.position.set(0.120,0,0.008);j1.add(j2);
  linkMesh(j2,"link2",0.130,0.048,0.008,M.teal,0,-0.004);
  cyl(j2,"joint2_hub",0.012,0.009,[0,0,0],M.gear,40);
  cyl(j2,"joint2_motor_shaft",0.0025,0.010,[0,0,-0.003],M.shaft,24);
  motor(j2,"motor3_28BYJ48").position.set(0.100,0,0);
  const j3=new THREE.Group();j3.name="link3_penholder";j3.position.set(0.100,0,0.008);j2.add(j3);
  linkMesh(j3,"link3_penholder",0.090,0.032,0.022,M.gold,0,-0.011);
  cyl(j3,"joint3_motor_shaft",0.0025,0.010,[0,0,-0.003],M.shaft,24);
  cyl(j3,"pen_holder_hub",0.011,0.018,[0.060,0,-0.014],M.gear,32);
  cyl(j3,"pen_barrel",0.0045,0.040,[0.060,0,-0.038],M.dark,24);
  cyl(j3,"pen_tip",0.0015,0.010,[0.060,0,-0.063],M.pen,20);
  return {root,joints:[j1,j2,j3]};
}
function fk(q){const a=[q[0],q[0]+q[1],q[0]+q[1]+q[2]];return [LENGTHS.reduce((s,l,i)=>s+l*Math.cos(a[i]),0),LENGTHS.reduce((s,l,i)=>s+l*Math.sin(a[i]),0)];}
function jacobian(q){const a=[q[0],q[0]+q[1],q[0]+q[1]+q[2]],j=[[],[]];for(let k=0;k<3;k++){let dx=0,dy=0;for(let n=k;n<3;n++){dx-=LENGTHS[n]*Math.sin(a[n]);dy+=LENGTHS[n]*Math.cos(a[n]);}j[0][k]=dx;j[1][k]=dy;}return j;}
function solveIK(target,seed){const q=seed.slice();for(let it=0;it<50;it++){const p=fk(q),ex=target[0]-p[0],ey=target[1]-p[1];if(Math.hypot(ex,ey)<0.00025)break;const j=jacobian(q),a=j[0].reduce((s,v)=>s+v*v,0)+0.00004,b=j[0].reduce((s,v,i)=>s+v*j[1][i],0),c=j[1].reduce((s,v)=>s+v*v,0)+0.00004,det=a*c-b*b;if(Math.abs(det)<1e-12)break;const vx=(c*ex-b*ey)/det,vy=(-b*ex+a*ey)/det,dq=j[0].map((v,i)=>v*vx+j[1][i]*vy),norm=Math.hypot(...dq),scale=norm>0.12?0.12/norm:1;for(let i=0;i<3;i++)q[i]=clamp(q[i]+dq[i]*scale,LIMITS[i][0],LIMITS[i][1]);}return q;}
function strokePoint(kind,u){const cx=0.252,cy=0,r=0.010;if(kind==="square"){const s=4*u,c=[[-r,-r],[r,-r],[r,r],[-r,r],[-r,-r]],idx=Math.min(3,Math.floor(s)),t=s-idx;return [cx+c[idx][0]*(1-t)+c[idx+1][0]*t,cy+c[idx][1]*(1-t)+c[idx+1][1]*t];}return [cx+r*Math.cos(2*Math.PI*u),cy+r*Math.sin(2*Math.PI*u)];}

class BrowserMuJoCo{
  constructor(){this.host=document.getElementById("mujoco-viewer");this.status=document.getElementById("sim-status");this.timeLabel=document.getElementById("sim-time");this.positionLabel=document.getElementById("tool-position");this.jointLabels=[1,2,3].map(i=>document.getElementById(`q${i}-value`));this.sliders=[1,2,3].map(i=>document.getElementById(`q${i}-slider`));this.paused=false;this.mode="manual";this.strokeStart=0;this.strokeDuration=7;this.seed=[0.4,-0.8,0.2];this.lastTrail=-1;this.trailCount=0;this.trailCapacity=2200;this.frameId=null;}
  async init(){this.setStatus("Loading MuJoCo WebAssembly…");this.mujoco=await loadMujoco({locateFile:(path,prefix)=>path.endsWith(".wasm")?"/wasm/mujoco.wasm":prefix+path});this.model=this.mujoco.MjModel.from_xml_string(modelXml);if(!this.model)throw new Error("MuJoCo failed to compile the 3R model.");this.data=new this.mujoco.MjData(this.model);this.siteId=this.mujoco.mj_name2id(this.model,this.mujoco.mjtObj.mjOBJ_SITE.value,"tool0");if(this.siteId<0)throw new Error("tool0 site not found.");this.setupScene();const assembly=makeAssembly(this.scene);this.cadRoot=assembly.root;this.cadJoints=assembly.joints;this.data.qpos.set(this.seed);this.data.ctrl.set(this.seed);this.mujoco.mj_forward(this.model,this.data);this.setupTrail();this.bindControls();this.setStatus("MuJoCo physics active · 3 joints + 3 motor envelopes");this.animate();}
  setupScene(){this.scene=new THREE.Scene();this.scene.background=new THREE.Color("#0d1725");this.renderer=new THREE.WebGLRenderer({antialias:true,alpha:false});this.renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,2));this.renderer.shadowMap.enabled=true;this.renderer.shadowMap.type=THREE.PCFSoftShadowMap;this.host.replaceChildren(this.renderer.domElement);this.camera=new THREE.PerspectiveCamera(42,1,0.01,3);this.camera.up.set(0,0,1);this.camera.position.set(0.37,-0.43,0.36);this.camera.lookAt(0.14,0,0.035);this.controls=new OrbitControls(this.camera,this.renderer.domElement);this.controls.target.set(0.14,0,0.035);this.controls.enableDamping=true;this.controls.update();this.scene.add(new THREE.AmbientLight(0xffffff,0.75));const key=new THREE.DirectionalLight(0xffffff,2.0);key.position.set(0.2,-0.3,0.6);key.castShadow=true;this.scene.add(key);const rim=new THREE.DirectionalLight(0x78bfe8,0.8);rim.position.set(-0.3,0.25,0.35);this.scene.add(rim);const ground=new THREE.Mesh(new THREE.PlaneGeometry(1.2,1.2),mat("#e7ebef",0,0.9));ground.position.z=-0.003;ground.receiveShadow=true;this.scene.add(ground);const grid=new THREE.GridHelper(0.65,32,0x476078,0x263a4e);grid.rotation.x=Math.PI/2;grid.position.z=-0.002;this.scene.add(grid);this.resize();window.addEventListener("resize",()=>this.resize());}
  resize(){if(!this.renderer||!this.host)return;const w=Math.max(320,this.host.clientWidth),h=Math.max(240,this.host.clientHeight);this.renderer.setSize(w,h,false);this.camera.aspect=w/h;this.camera.updateProjectionMatrix();}
  setupTrail(){this.trailGeometry=new THREE.BufferGeometry();this.trailGeometry.setAttribute("position",new THREE.BufferAttribute(new Float32Array(this.trailCapacity*3),3));this.trailGeometry.setDrawRange(0,0);this.trailLine=new THREE.Line(this.trailGeometry,new THREE.LineBasicMaterial({color:0x62d6b1,linewidth:2}));this.trailLine.frustumCulled=false;this.scene.add(this.trailLine);}
  clearTrail(){this.trailCount=0;this.lastTrail=-1;this.trailGeometry.setDrawRange(0,0);this.trailGeometry.attributes.position.needsUpdate=true;}
  recordTrail(){if(this.mode!=="circle"&&this.mode!=="square")return;const t=Number(this.data.time);if(t-this.lastTrail<0.018||this.trailCount>=this.trailCapacity)return;const p=this.data.site_xpos,a=this.trailGeometry.attributes.position,i=this.siteId*3;a.setXYZ(this.trailCount,p[i],p[i+1],p[i+2]+0.001);this.trailCount++;this.lastTrail=t;this.trailGeometry.setDrawRange(0,this.trailCount);a.needsUpdate=true;}
  setStatus(s){if(this.status)this.status.textContent=s;}
  setCadJointAngles(q){for(let i=0;i<3;i++)this.cadJoints[i].rotation.z=Number(q[i]);this.cadRoot.updateMatrixWorld(true);}
  updateReadout(){const q=this.data.qpos,p=this.data.site_xpos,i=this.siteId*3,x=p[i],y=p[i+1],z=p[i+2];this.timeLabel.textContent=`${Number(this.data.time).toFixed(2)} s`;this.positionLabel.textContent=`X ${(x*1000).toFixed(1)} · Y ${(y*1000).toFixed(1)} · Z ${(z*1000).toFixed(1)} mm`;for(let k=0;k<3;k++){const v=Number(q[k]);this.jointLabels[k].textContent=`${(v*180/Math.PI).toFixed(1)}°`;if(document.activeElement!==this.sliders[k])this.sliders[k].value=v.toFixed(3);}}
  setManualTarget(i,v){this.mode="manual";this.seed=Array.from(this.data.qpos).slice(0,3);this.data.ctrl[i]=clamp(Number(v),LIMITS[i][0],LIMITS[i][1]);this.setStatus("Manual joint target · MuJoCo physics + articulated motor view");}
  startStroke(kind){this.mode=kind;this.strokeStart=Number(this.data.time);this.seed=Array.from(this.data.qpos).slice(0,3);this.strokeDuration=kind==="circle"?7:8;this.clearTrail();this.paused=false;this.setPauseButton();this.setStatus(`Drawing ${kind} · DLS IK + MuJoCo physics`);}
  setPauseButton(){const b=document.getElementById("pause-sim");if(b)b.textContent=this.paused?"Resume":"Pause";}
  bindControls(){document.getElementById("pause-sim").addEventListener("click",()=>{this.paused=!this.paused;this.setPauseButton();this.setStatus(this.paused?"Paused · physics state retained":"MuJoCo physics active · articulated motor assembly");});document.getElementById("reset-sim").addEventListener("click",()=>{this.mode="manual";this.paused=false;this.mujoco.mj_resetData(this.model,this.data);this.data.qpos.set([0.4,-0.8,0.2]);this.data.ctrl.set([0.4,-0.8,0.2]);this.mujoco.mj_forward(this.model,this.data);this.clearTrail();this.setPauseButton();this.setStatus("Reset to nominal three-link pose");});document.getElementById("draw-circle").addEventListener("click",()=>this.startStroke("circle"));document.getElementById("draw-square").addEventListener("click",()=>this.startStroke("square"));this.sliders.forEach((s,i)=>s.addEventListener("input",e=>this.setManualTarget(i,e.target.value)));}
  updateController(){if(this.mode!=="circle"&&this.mode!=="square")return;const u=(Number(this.data.time)-this.strokeStart)/this.strokeDuration;if(u>=1){this.mode="manual";this.setStatus("Stroke complete · MuJoCo simulation continues");return;}this.seed=solveIK(strokePoint(this.mode,Math.max(0,u)),this.seed);for(let i=0;i<3;i++)this.data.ctrl[i]=this.seed[i];}
  animate(){if(this.frameId!==null)return;const tick=()=>{try{if(!this.paused){const end=Number(this.data.time)+1/60;let n=0;while(Number(this.data.time)<end&&n<12){this.updateController();this.mujoco.mj_step(this.model,this.data);n++;}this.recordTrail();}this.setCadJointAngles(this.data.qpos);this.controls.update();this.renderer.render(this.scene,this.camera);this.updateReadout();}catch(e){console.error("MuJoCo simulation error",e);this.setStatus("Simulation stopped · see diagnostic message");}this.frameId=requestAnimationFrame(tick);};this.frameId=requestAnimationFrame(tick);}
  dispose(){if(this.frameId!==null)cancelAnimationFrame(this.frameId);this.controls?.dispose();this.renderer?.dispose();this.trailGeometry?.dispose();this.data?.delete();this.model?.delete();}
}
async function start(){const sim=new BrowserMuJoCo();try{await sim.init();window.addEventListener("beforeunload",()=>sim.dispose(),{once:true});}catch(error){console.error(error);sim.setStatus("MuJoCo initialization failed");const d=document.getElementById("sim-error");if(d)d.textContent=error?.message||String(error);}}
start();
