// 3D overlays for the shadow SLAM estimator and the label-free segmentation:
//   red   = odometry only (dead reckoning, drifts)
//   green = LiDAR odometry + pose graph (corrected after every loop closure)
//   gold  = accepted loop-closure edges (between the two keyframes that were matched)
// Everything is drawn from the estimator's own output; the true path is the blue trail of the debug layer.
import * as THREE from 'three';

const MAX_POINTS = 2500;
const SEG_COLORS = ['#d9a441', '#8f7bff', '#3f8cff', '#ff6a00'].map((h) => new THREE.Color(h)); // floor, ceiling, wall, object

export function createCaveSlamTraj(world) {
  const group = new THREE.Group();
  group.name = 'cave_slam_estimate';
  const floorAt = (x, y, fallback) => world.floor.floorAt(x, y) ?? fallback;

  const makeLine = (color) => {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
    geo.setDrawRange(0, 0); geo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 500);
    const line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color, depthTest: false, transparent: true, opacity: 0.95 }));
    line.renderOrder = 16; line.frustumCulled = false; line.visible = false;
    group.add(line);
    return { line, geo };
  };
  const est = makeLine('#35ff8c'), odom = makeLine('#ff4d4d');
  const loopLines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: '#ffd23a', depthTest: false, transparent: true }));
  loopLines.renderOrder = 17; loopLines.frustumCulled = false; loopLines.visible = false;
  group.add(loopLines);

  // heights are display-only: the floor under the TRUE position (drifted estimates may lie inside rock where no floor exists)
  function fill(l, pts, truthPts) {
    const arr = l.geo.attributes.position.array;
    let n = 0, lastZ = 0;
    for (let i = 0; i < pts.length; i++) {
      lastZ = floorAt(truthPts[i][0], truthPts[i][1], lastZ) + 0.45;
      arr[3 * n] = pts[i][0]; arr[3 * n + 1] = lastZ; arr[3 * n + 2] = -pts[i][1]; n++;
    }
    l.geo.setDrawRange(0, n); l.geo.attributes.position.needsUpdate = true;
  }

  let lastVersion = -1, lastSamples = -1;
  function update(slam) {
    const n = slam.samples.length;
    if (!n || (slam.version === lastVersion && n === lastSamples)) return;
    lastVersion = slam.version; lastSamples = n;
    const stride = Math.max(1, Math.ceil(n / MAX_POINTS));
    const truth = slam.truthTrajectory(stride);
    fill(est, slam.trajectory(stride), truth); fill(odom, slam.odometryTrajectory(stride), truth);
    const seg = [];
    for (const c of slam.loopClosures) {
      const a = slam.keyframes[c.from].pose, b = slam.keyframes[c.to].pose;
      const ta = slam.keyframes[c.from].truth, tb = slam.keyframes[c.to].truth;
      const za = floorAt(ta[0], ta[1], 0) + 0.45, zb = floorAt(tb[0], tb[1], 0) + 0.45;
      seg.push(a[0], za, -a[1], b[0], zb, -b[1]);
    }
    loopLines.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(seg), 3));
  }

  // ---- label-free segmentation view ----
  let segPoints = null;
  function showSegmentation(pts, labels) {
    clearSegmentation();
    const n = labels.length, pos = new Float32Array(n * 3), col = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
      pos[3 * i] = pts[3 * i]; pos[3 * i + 1] = pts[3 * i + 2]; pos[3 * i + 2] = -pts[3 * i + 1];
      const c = SEG_COLORS[labels[i]]; col[3 * i] = c.r; col[3 * i + 1] = c.g; col[3 * i + 2] = c.b;
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3)); geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
    segPoints = new THREE.Points(geo, new THREE.PointsMaterial({ size: 0.16, vertexColors: true, sizeAttenuation: true }));
    segPoints.frustumCulled = false;
    group.add(segPoints);
  }
  function clearSegmentation() { if (segPoints) { group.remove(segPoints); segPoints.geometry.dispose(); segPoints.material.dispose(); segPoints = null; } }

  return {
    group, update, showSegmentation, clearSegmentation,
    get hasSegmentation() { return !!segPoints; },
    setSegmentationVisible(on) { if (segPoints) segPoints.visible = on; },
    setPart(kind, on) { ({ est: est.line, odom: odom.line, loops: loopLines })[kind].visible = on; },
    reset() { lastVersion = -1; lastSamples = -1; est.geo.setDrawRange(0, 0); odom.geo.setDrawRange(0, 0); loopLines.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3)); clearSegmentation(); },
    dispose() { clearSegmentation(); for (const l of [est, odom]) { l.geo.dispose(); l.line.material.dispose(); } loopLines.geometry.dispose(); loopLines.material.dispose(); },
  };
}
