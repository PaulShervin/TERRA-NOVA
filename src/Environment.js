// Main scene: renderer, layout -> objects, lighting, robot, debug layer, optional AO post-process.
import * as THREE from 'three';
import { EffectComposer } from 'three/examples/jsm/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/examples/jsm/postprocessing/RenderPass.js';
import { GTAOPass } from 'three/examples/jsm/postprocessing/GTAOPass.js';
import { OutputPass } from 'three/examples/jsm/postprocessing/OutputPass.js';
import { UnrealBloomPass } from 'three/examples/jsm/postprocessing/UnrealBloomPass.js';
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js';
import { createDecals } from './Decals.js';
import { buildLayout } from './layout/index.js';
import { createMaterials, disposeMaterials } from './core/materials.js';
import { buildFactory } from './FactoryLayout.js';
import { createLighting } from './Lighting.js';
import { createRobot } from './Robot.js';
import { createCameraControls } from './CameraControls.js';
import { createDebugLayer } from './DebugLayer.js';
import { createSemanticView } from './SemanticView.js';
import { createScanOverlay } from './ScanOverlay.js';
import { createMapper } from './mapping/Mapper.js';
import { createMiniMap } from './MiniMap.js';
import { createLidarViz } from './LidarViz.js';
import { createInsets } from './Insets.js';
import { createRobotController } from './RobotController.js';

const RECEIVE_ONLY = new Set(['floor', 'pathway']);

export function createEnvironment(container) {
  const layout = buildLayout();
  const size = () => [container.clientWidth, container.clientHeight];
  const [w, h] = size();

  const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(w, h);
  renderer.localClippingEnabled = true;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.shadowMap.autoUpdate = false; // refreshed once per frame; insets reuse it
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#0b0e12');
  scene.fog = new THREE.Fog('#0b0e12', 110, 260);

  // Image-based lighting from a neutral studio-style environment gives metals/paint believable reflections.
  const pmrem = new THREE.PMREMGenerator(renderer);
  const envScene = new RoomEnvironment();
  scene.environment = pmrem.fromScene(envScene, 0.04).texture;
  scene.environmentIntensity = 0.35;
  envScene.dispose();

  const materials = createMaterials();
  const factory = buildFactory(layout, materials);
  const lighting = createLighting(layout.lights, materials, layout.meta.bounds);
  const robot = createRobot(layout.robot.spawn, layout.robot.yaw);
  const debug = createDebugLayer(layout);
  const cameraRig = createCameraControls(renderer.domElement, w / h);
  cameraRig.resize(w, h);
  const driver = createRobotController(robot, [], layout.robot);
  const lidarViz = createLidarViz();
  const mapper = createMapper({ layout, controller: driver, robot, onScan: ({ rays, pose, time }) => lidarViz.addScan(rays, pose, time) });
  const miniMap = createMiniMap(document.getElementById('minimap'), document.getElementById('map-status'));
  const scan = createScanOverlay(5.5);
  const decals = createDecals();
  scene.add(decals.group);
  scene.add(factory.root, lighting.group, robot.root, debug.root, scan.mesh, mapper.cloud.mainPoints, lidarViz.group);
  scan.mesh.visible = false; // replaced by the occlusion-aware LiDAR visualisation

  // Roof (ceiling plane + beams) lives on layer 1: only the robot RGB camera sees it unless "Roof" is toggled on.
  const b = layout.meta.bounds;
  const ceiling = new THREE.Mesh(new THREE.PlaneGeometry(b.xMax - b.xMin, b.yMax - b.yMin).rotateX(Math.PI / 2), materials.ceiling);
  ceiling.position.y = 6.6;
  ceiling.name = 'ceiling';
  scene.add(ceiling);
  const beams = scene.getObjectByName('box|steel|pillar');
  const roofParts = [ceiling, beams].filter(Boolean);
  roofParts.forEach((o) => { o.layers.set(1); });
  const setRoof = (on) => roofParts.forEach((o) => { if (on) o.layers.enable(0); else o.layers.disable(0); });

  scene.traverse((o) => {
    if (!o.isMesh) return;
    o.castShadow = !RECEIVE_ONLY.has(o.userData.category) && o !== ceiling;
    o.receiveShadow = true;
  });

  const semantic = createSemanticView(scene);

  // Optional ambient occlusion (GTAO); kept off by default for performance.
  const composer = new EffectComposer(renderer);
  composer.addPass(new RenderPass(scene, cameraRig.camera));
  const ao = new GTAOPass(scene, cameraRig.camera, w, h);
  ao.output = GTAOPass.OUTPUT.Default;
  ao.enabled = false;
  composer.addPass(ao);
  const bloom = new UnrealBloomPass(new THREE.Vector2(w, h), 0.4, 0.55, 0.92);
  composer.addPass(bloom);
  composer.addPass(new OutputPass());

  const insets = createInsets({
    renderer, scene, mainCamera: cameraRig.camera, robot, semantic,
    pointCloud: { points: mapper.cloud.insetPoints, dispose() {} }, hideDuring: [debug.root, scan.mesh, mapper.cloud.mainPoints, lidarViz.group],
  });

  let raf = 0;
  let follow = false;
  let lastPath = null, miniTimer = 1;
  const clock = new THREE.Clock();
  const stats = { fps: 0, calls: 0, triangles: 0 };
  let frames = 0, acc = 0;
  const loop = () => {
    raf = requestAnimationFrame(loop);
    const dt = clock.getDelta();
    acc += dt; frames += 1;
    if (acc >= 0.5) { stats.fps = frames / acc; frames = 0; acc = 0; }
    cameraRig.update();
    const before = robot.root.position.clone();
    mapper.update(dt);
    mapper.cloud.flush();
    lidarViz.frame(mapper.simTime);
    const path = mapper.explorer.getPath();
    if (path !== lastPath) { lastPath = path; debug.setPath(path); }
    miniTimer += dt;
    if (miniTimer > 0.4) { miniTimer = 0; miniMap.draw(mapper, driver.pose, path, mapper.explorer.getGoal()); }
    if (follow) { // chase mode: shift camera + orbit target by the robot's displacement
      const delta = robot.root.position.clone().sub(before);
      cameraRig.camera.position.add(delta);
      cameraRig.controls.target.add(delta);
    }
    debug.followRobot(robot.root);
    scan.follow(robot.root);
    scan.update(clock.elapsedTime);
    renderer.shadowMap.needsUpdate = true;
    composer.render();
    insets.render(clock.elapsedTime);
    stats.calls = renderer.info.render.calls;
    stats.triangles = renderer.info.render.triangles;
  };
  loop();

  const onResize = () => {
    const [nw, nh] = size();
    renderer.setSize(nw, nh);
    composer.setSize(nw, nh);
    cameraRig.resize(nw, nh);
  };
  window.addEventListener('resize', onResize);

  const dispose = () => {
    cancelAnimationFrame(raf);
    window.removeEventListener('resize', onResize);
    cameraRig.dispose();
    insets.dispose(); lidarViz.dispose(); mapper.dispose(); scan.dispose(); ceiling.geometry.dispose();
    factory.dispose(); lighting.dispose(); robot.dispose(); debug.dispose(); semantic.dispose();
    ao.dispose(); bloom.dispose(); decals.dispose(); pmrem.dispose(); composer.dispose();
    disposeMaterials(materials);
    renderer.dispose();
    renderer.domElement.remove();
  };

  return {
    layout, scene, camera: cameraRig.camera, cameraRig, robot, debug, stats, dispose,
    setSemantic: semantic.set,
    setAO: (on) => { ao.enabled = on; },
    setRoof,
    driver, mapper,
    setMapVisible: (on) => { mapper.cloud.mainPoints.visible = on; },
    setFollow: (on) => { follow = on; },
    setOverlay: (on) => { lidarViz.group.visible = on; },
    getCameraRos: () => [cameraRig.camera.position.x, -cameraRig.camera.position.z, cameraRig.camera.position.y],
  };
}
