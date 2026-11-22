// Renders the dashboard insets (RGB camera, LiDAR point cloud, semantic view, axis gizmo) into DOM-defined
// rectangles of the single WebGL canvas using scissor viewports. Insets always render AFTER the main view.
import * as THREE from 'three';
import { rosToThree } from './core/coords.js';
import { rosAxes } from './core/axes.js';
import { SENSOR_FRAMES } from './Robot.js';

const q = (id) => document.getElementById(id);

export function createInsets({ renderer, scene, mainCamera, robot, semantic, pointCloud, hideDuring }) {
  const canvas = renderer.domElement;

  // RGB camera mounted on the robot (sees the ceiling via layer 1).
  const rgbCam = new THREE.PerspectiveCamera(78, 1.6, 0.05, 120);
  rgbCam.layers.enable(1);

  // Fixed isometric camera for the semantic inset.
  const semCam = new THREE.PerspectiveCamera(36, 1.6, 0.5, 300);
  semCam.position.copy(rosToThree([-6, -62, 42]));
  semCam.lookAt(rosToThree([0, 0, 0]));

  // Orbiting camera for the point cloud inset.
  const cloudScene = new THREE.Scene();
  cloudScene.background = new THREE.Color('#05070a');
  cloudScene.add(pointCloud.points);
  const cloudMarker = new THREE.Mesh(new THREE.SphereGeometry(0.45, 16, 12), new THREE.MeshBasicMaterial({ color: '#ffd60a' }));
  cloudScene.add(cloudMarker);
  const cloudCam = new THREE.PerspectiveCamera(34, 1.6, 0.5, 300);

  // Axis gizmo
  const gizmoScene = new THREE.Scene();
  gizmoScene.add(rosAxes(1, 0.22));
  const gizmoCam = new THREE.PerspectiveCamera(30, 1, 0.1, 20);
  const gizmoLabels = { X: q('gizmo-x'), Y: q('gizmo-y'), Z: q('gizmo-z') };
  const tips = { X: new THREE.Vector3(1.25, 0, 0), Y: new THREE.Vector3(0, 0, -1.25), Z: new THREE.Vector3(0, 1.25, 0) };

  const region = (el) => {
    if (!el) return null;
    const r = el.getBoundingClientRect(), c = canvas.getBoundingClientRect();
    if (r.width < 4 || r.height < 4) return null;
    return { x: r.left - c.left, y: c.height - (r.bottom - c.top), w: r.width, h: r.height };
  };
  const begin = (rg, clearColor = true) => {
    renderer.setViewport(rg.x, rg.y, rg.w, rg.h);
    renderer.setScissor(rg.x, rg.y, rg.w, rg.h);
    if (clearColor) renderer.clear(); else renderer.clearDepth();
  };

  function renderRgb() {
    const rg = region(q('inset-rgb'));
    if (!rg) return;
    robot.root.updateMatrixWorld(true);
    const p = rosToThree(SENSOR_FRAMES.camera_link).add(new THREE.Vector3(0.02, 0.0, 0));
    rgbCam.position.copy(robot.root.localToWorld(p));
    const ahead = robot.root.localToWorld(p.clone().add(new THREE.Vector3(5, -0.15, 0)));
    rgbCam.lookAt(ahead);
    rgbCam.aspect = rg.w / rg.h;
    rgbCam.updateProjectionMatrix();
    const wasVisible = robot.root.visible;
    robot.root.visible = false;
    begin(rg);
    renderer.render(scene, rgbCam);
    robot.root.visible = wasVisible;
  }

  function renderSemantic() {
    const rg = region(q('inset-sem'));
    if (!rg) return;
    const prior = semantic.isEnabled();
    semantic.set(true);
    semCam.aspect = rg.w / rg.h;
    semCam.updateProjectionMatrix();
    begin(rg);
    renderer.render(scene, semCam);
    semantic.set(prior);
  }

  function renderCloud(time) {
    const rg = region(q('inset-lidar'));
    if (!rg) return;
    const a = -0.9 + Math.sin(time * 0.15) * 0.5;
    cloudCam.position.copy(rosToThree([Math.sin(a) * 74, -Math.cos(a) * 74, 48]));
    cloudCam.lookAt(rosToThree([0, 0, 1]));
    cloudCam.aspect = rg.w / rg.h;
    cloudCam.updateProjectionMatrix();
    cloudMarker.position.copy(robot.root.position).setY(0.6);
    begin(rg);
    renderer.render(cloudScene, cloudCam);
  }

  function renderGizmo() {
    const rg = region(q('gizmo'));
    if (!rg) return;
    const dir = new THREE.Vector3();
    mainCamera.getWorldDirection(dir);
    gizmoCam.position.copy(dir).multiplyScalar(-4.5);
    gizmoCam.up.copy(mainCamera.up);
    gizmoCam.lookAt(0, 0, 0);
    gizmoCam.updateMatrixWorld();
    begin(rg, false);
    renderer.render(gizmoScene, gizmoCam);
    for (const [k, tip] of Object.entries(tips)) {
      const v = tip.clone().project(gizmoCam);
      const el = gizmoLabels[k];
      if (el) { el.style.left = `${(v.x * 0.5 + 0.5) * rg.w}px`; el.style.top = `${(-v.y * 0.5 + 0.5) * rg.h}px`; }
    }
  }

  return {
    render(time) {
      const prevAuto = renderer.autoClear;
      const prevTone = renderer.toneMapping;
      renderer.autoClear = false;
      renderer.setScissorTest(true);
      const restore = hideDuring.map((o) => [o, o.visible]);
      restore.forEach(([o]) => { o.visible = false; });
      renderRgb(); renderSemantic();
      restore.forEach(([o, v]) => { o.visible = v; });
      renderCloud(time);
      renderGizmo();
      renderer.setScissorTest(false);
      renderer.setViewport(0, 0, canvas.clientWidth, canvas.clientHeight);
      renderer.autoClear = prevAuto;
      renderer.toneMapping = prevTone;
    },
    dispose() { pointCloud.dispose(); gizmoScene.traverse((o) => o.geometry?.dispose?.()); },
  };
}
