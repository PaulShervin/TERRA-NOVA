// The cave environment: independent of the industrial facility (own renderer, scene, camera, HUD, robot, mapper).
// Created lazily by caveMain.js; nothing in the industrial code imports this file.
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { EffectComposer } from 'three/examples/jsm/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/examples/jsm/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/examples/jsm/postprocessing/UnrealBloomPass.js';
import { OutputPass } from 'three/examples/jsm/postprocessing/OutputPass.js';
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js';
import { buildCaveWorld, exportCaveJSON } from './CaveGeometry.js';
import { toThreeGeometries } from './CaveMesh.js';
import { createRockMaterial, computeVertexColors } from './CaveMaterials.js';
import { createCaveLighting } from './CaveLighting.js';
import { createCaveObjects } from './CaveObjects.js';
import { createCaveRobot } from './CaveRobot.js';
import { createCaveMapper } from './CaveMapper.js';
import { createCaveSlamLayer } from './CaveSlamLayer.js';
import { createCaveHud } from './CaveHud.js';
import { createCaveSlamTraj } from './CaveSlamTraj.js';
import { segmentPoints, evaluateSegmentation } from '../slam/Segmenter.js';
import { createLidarViz } from '../LidarViz.js';
import { createMiniMap } from '../MiniMap.js';
import { createRobotController } from '../RobotController.js';
import { rosToThree } from '../core/coords.js';
import { CATEGORY_COLORS } from '../core/materials.js';
import { SENSOR_FRAMES } from '../Robot.js';

// Semantic colours for the cave classes (added at runtime; the industrial palette is untouched on disk)
Object.assign(CATEGORY_COLORS, { ceiling: '#a8a8ff', boulder: '#ff8a1f', stalactite: '#ffe14a' });

const VIEWS = {
  isometric: { pos: [-32, -78, 64], target: [1, -3, -2], fov: 40 },
  top: { pos: [1, -2, 150], target: [1, -3, 0], fov: 40 },
};
const nextPaint = () => new Promise((r) => requestAnimationFrame(() => setTimeout(r, 0)));

export async function createCaveEnvironment(container) {
  let hud;
  const loadingHud = createCaveHud({ view() {}, reset() {}, export() {}, debug() {}, roof() {}, lidarviz() {}, mode() {}, speed() {}, mapPoints() {}, mapColor() {}, tint() {} });
  loadingHud.setLoading(true, 'Generating procedural cave (signed-distance field + surface nets)...');
  await nextPaint();
  const t0 = performance.now();
  const world = buildCaveWorld({ withMesh: true });
  loadingHud.dispose();

  const size = () => [container.clientWidth, container.clientHeight];
  const [w, h] = size();
  const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(w, h);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.shadowMap.autoUpdate = false; // static geometry: refreshed on demand (roof toggle)
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.15;
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#0b0a09');
  scene.fog = new THREE.FogExp2('#0d0b0a', 0.0055);
  const pmrem = new THREE.PMREMGenerator(renderer);
  const envScene = new RoomEnvironment();
  scene.environment = pmrem.fromScene(envScene, 0.04).texture;
  scene.environmentIntensity = 0.18;
  envScene.dispose();

  // ---- cave rock ----
  const rock = createRockMaterial();
  const colors = computeVertexColors(world);
  const baseColors = new Float32Array(colors);
  const { bodyGeo, roofGeo, triangles } = toThreeGeometries(world.mesh, colors);
  // the roof's faces point down (into the cave), so give it a double-sided material to read as solid rock from above
  const roofRock = createRockMaterial();
  roofRock.side = THREE.DoubleSide;
  const bodyMesh = new THREE.Mesh(bodyGeo, rock), roofMesh = new THREE.Mesh(roofGeo, roofRock);
  bodyMesh.name = 'cave_rock_body'; roofMesh.name = 'cave_rock_roof';
  for (const m of [bodyMesh, roofMesh]) { m.castShadow = true; m.receiveShadow = true; m.userData.category = 'wall'; }
  scene.add(bodyMesh, roofMesh);

  const lighting = createCaveLighting(world);
  const objects = createCaveObjects(world);
  const robot = createCaveRobot(world, world.config.robot.spawn, world.config.robot.yaw);
  const lidarViz = createLidarViz({ intensity: 0.45, pointSize: 3.4 });
  const driver = createRobotController(robot, [], { spawn: [world.config.robot.spawn[0], world.config.robot.spawn[1], 0], yaw: world.config.robot.yaw });
  const mapper = createCaveMapper({ world, controller: driver, onScan: (s) => lidarViz.addScan(s.rays, s.pose, s.time, s.floorZ) });
  mapper.cloud.mainPoints.material.clippingPlanes = []; // no height clipping in the cave
  const slam = createCaveSlamLayer({ world, getGrid: () => mapper.grid, getTrajectory: () => mapper.trajectory, robot, bodyGeo, baseColors, vertsRos: world.mesh.positions });
  const slamTraj = createCaveSlamTraj(world);
  scene.add(slamTraj.group, lighting.group, objects.mesh, robot.root, lidarViz.group, mapper.cloud.mainPoints, slam.group);

  // ---- camera ----
  const camera = new THREE.PerspectiveCamera(40, w / h, 0.1, 500);
  const orbit = new OrbitControls(camera, renderer.domElement);
  orbit.enableDamping = true; orbit.dampingFactor = 0.08; orbit.screenSpacePanning = true; orbit.minDistance = 1.5; orbit.maxDistance = 260;
  let camMode = 'orbit'; // orbit | chase | fpv
  const goOrbit = (name) => {
    camMode = 'orbit'; orbit.enabled = true;
    const v = VIEWS[name] ?? VIEWS.isometric;
    camera.fov = v.fov; camera.updateProjectionMatrix();
    camera.position.copy(rosToThree(v.pos)); orbit.target.copy(rosToThree(v.target)); orbit.update();
  };
  goOrbit('isometric');
  const setView = (name) => {
    if (name === 'isometric' || name === 'top' || name === 'reset') { goOrbit(name === 'reset' ? 'isometric' : name); return; }
    camMode = name; orbit.enabled = false; camera.fov = name === 'fpv' ? 75 : 55; camera.updateProjectionMatrix();
  };
  const tmpA = new THREE.Vector3(), tmpB = new THREE.Vector3();
  function updateRobotCamera() {
    robot.root.updateMatrixWorld(true);
    if (camMode === 'fpv') {
      const p = robot.root.localToWorld(tmpA.copy(rosToThree([SENSOR_FRAMES.camera_link[0] + 0.03, 0, SENSOR_FRAMES.camera_link[2]])));
      camera.position.copy(p);
      camera.lookAt(robot.root.localToWorld(tmpB.copy(rosToThree([6, 0, SENSOR_FRAMES.camera_link[2] - 0.25]))));
    } else if (camMode === 'chase') {
      camera.position.copy(robot.root.localToWorld(tmpA.copy(rosToThree([-3.2, 0.6, 2.0]))));
      camera.lookAt(robot.root.localToWorld(tmpB.copy(rosToThree([2.5, 0, 0.5]))));
    }
  }

  // ---- post (bloom for the LiDAR glow + lamps) ----
  const composer = new EffectComposer(renderer);
  composer.addPass(new RenderPass(scene, camera));
  const bloom = new UnrealBloomPass(new THREE.Vector2(w, h), 0.22, 0.5, 0.92);
  composer.addPass(bloom);
  composer.addPass(new OutputPass());

  // ---- HUD ----
  hud = createCaveHud({
    view: setView,
    reset: () => { mapper.reset(); lidarViz.clear(); slam.clearTrail(); slamTraj.reset(); hud.segResult.textContent = ''; mapper.cloud.mainPoints.visible = true; },
    slamTraj: (k, on) => slamTraj.setPart(k, on),
    segView: (on) => { slamTraj.setSegmentationVisible(on); mapper.cloud.mainPoints.visible = !(on && slamTraj.hasSegmentation); },
    segment: () => runSegmentation(),
    export: () => {
      const blob = new Blob([JSON.stringify(exportCaveJSON(world), null, 2)], { type: 'application/json' });
      const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = 'cave.json'; a.click(); URL.revokeObjectURL(a.href);
    },
    debug: (k, on) => slam.setPart(k, on),
    roof: (on) => { roofMesh.visible = on; renderer.shadowMap.needsUpdate = true; },
    lidarviz: (on) => { lidarViz.group.visible = on; },
    mode: (m) => mapper.setMode(m),
    speed: (v) => mapper.setSimSpeed(v),
    mapPoints: (on) => { mapper.cloud.mainPoints.visible = on; },
    mapColor: (m) => mapper.cloud.setColorMode(m),
    tint: (on) => slam.setTint(on),
  });
  for (const [k, on] of hud.debugDefaults) slam.setPart(k, on);
  for (const k of ['est', 'odom', 'loops']) slamTraj.setPart(k, true);
  hud.slamCard.hidden = false;
  const CLASS_NAMES = ['floor', 'ceiling', 'wall', 'object'];
  function runSegmentation() {
    const hs = mapper.hitStore, n = hs.x.length;
    if (n < 500) { hud.segResult.textContent = 'Not enough map yet - let the robot explore first.'; return; }
    const pts = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) { pts[3 * i] = hs.x[i]; pts[3 * i + 1] = hs.y[i]; pts[3 * i + 2] = hs.z[i]; }
    const vps = new Float32Array(mapper.trajectory.length * 3);
    mapper.trajectory.forEach((q, i) => { vps[3 * i] = q.x; vps[3 * i + 1] = q.y; vps[3 * i + 2] = (world.floor.floorAt(q.x, q.y) ?? 0) + 1; });
    const t0 = performance.now();
    const { labels } = segmentPoints(pts, { viewpoints: vps });
    const ev = evaluateSegmentation(labels, Uint8Array.from(hs.cat));
    slamTraj.showSegmentation(pts, labels);
    mapper.cloud.mainPoints.visible = false;
    const lines = ev.perClass.map((c) => `${c.name.padEnd(8)} IoU ${((c.iou ?? 0) * 100).toFixed(0)}%`);
    hud.segResult.textContent = [`${n} points, ${(performance.now() - t0).toFixed(0)} ms (geometry only)`, `accuracy ${(ev.accuracy * 100).toFixed(0)}%  mean IoU ${(ev.meanIoU * 100).toFixed(0)}%`, ...lines,
      'tan floor, violet ceiling, blue wall, orange object', 'scored vs simulator labels (not used to segment)'].join('\n');
  }
  const miniMap = createMiniMap(hud.minimap, hud.mapStatus);
  roofMesh.visible = false; // start as a cut-away so the whole network is inspectable; the HUD checkbox shows the roof
  renderer.shadowMap.needsUpdate = true;

  // ---- loop ----
  let raf = 0, miniTimer = 1, lastPath = null, frames = 0, acc = 0, fps = 0;
  const clock = new THREE.Clock();
  const loop = () => {
    raf = requestAnimationFrame(loop);
    const dt = clock.getDelta();
    acc += dt; frames++;
    if (acc >= 0.5) { fps = frames / acc; frames = 0; acc = 0; }
    mapper.update(dt);
    mapper.cloud.flush();
    lidarViz.frame(mapper.simTime);
    const p = driver.pose, ls = mapper.lastScan;
    slam.update(mapper.simTime, p, ls ? ls.floorZ : 0);
    slamTraj.update(mapper.slam);
    slam.group.visible = camMode === 'orbit'; // debug labels would clutter the robot-eye views
    if (camMode === 'orbit') orbit.update(); else updateRobotCamera();
    composer.render();
    miniTimer += dt;
    if (miniTimer > 0.4) {
      miniTimer = 0;
      miniMap.draw(mapper, p, mapper.explorer.getPath(), mapper.explorer.getGoal());
      const s = mapper.stats();
      const sm = mapper.slam.metrics();
      hud.slamStats.textContent = ['pose error (ATE, RMSE)', `  odometry only  ${sm.ateOdom.toFixed(2)} m`, `  SLAM estimate  ${sm.ate.toFixed(2)} m`, 'current error', `  odometry only  ${sm.finalErrorOdom.toFixed(2)} m`, `  SLAM estimate  ${sm.finalError.toFixed(2)} m`, `keyframes ${sm.keyframes} - loop closures ${sm.loopClosures}`, '(true pose used only to score)'].join('\n');
      hud.readout.textContent = `${fps.toFixed(0)} fps · ${renderer.info.render.calls} draw calls\nrobot (${p.x.toFixed(1)}, ${p.y.toFixed(1)}) m · ${s.scans} sweeps`;
    }
  };
  loop();

  const onResize = () => {
    const [nw, nh] = size();
    renderer.setSize(nw, nh); composer.setSize(nw, nh);
    camera.aspect = nw / nh; camera.updateProjectionMatrix();
  };
  window.addEventListener('resize', onResize);

  console.info(`[cave] generated in ${(performance.now() - t0).toFixed(0)} ms — ${world.mesh.positions.length / 3} vertices, ${triangles.body + triangles.roof} triangles, ${objects.count} rubble instances`);

  return {
    world, scene, camera, mapper, driver, robot, slam, roofMesh, setView, stats: () => ({ fps }),
    dispose() {
      cancelAnimationFrame(raf);
      window.removeEventListener('resize', onResize);
      orbit.dispose(); hud.dispose();
      slam.dispose(); slamTraj.dispose(); lidarViz.dispose(); mapper.dispose(); lighting.dispose(); objects.dispose(); robot.dispose();
      bodyGeo.dispose(); roofGeo.dispose(); rock.userData.dispose?.(); rock.dispose(); roofRock.userData.dispose?.(); roofRock.dispose();
      bloom.dispose(); composer.dispose(); pmrem.dispose(); renderer.dispose(); renderer.domElement.remove();
    },
  };
}
