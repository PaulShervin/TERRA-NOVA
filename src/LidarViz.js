// Cinematic LiDAR visualisation (purely visual; fed by the real simulated scans):
//  - sweep reveal: each return lights up at the moment the spinning head passes its azimuth, then fades
//  - glowing return points coloured by range (near = magenta/pink -> far = cyan), additive so bloom picks them up
//  - thin beam lines from the sensor to a subset of returns
//  - occlusion-aware "visibility fan" on the floor (what the sensor can actually see, blocked by walls)
//  - bright rotating scan line + sensor halo
import * as THREE from 'three';
import { LIDAR } from './sensing/Lidar.js';

const SCAN_PERIOD = 0.2; // s, one full sweep (5 Hz)
const FAN_RANGE = 14; // m, clamp for the floor visibility fan
const FADE = 1.3; // s, return afterglow

const POINT_VERT = /* glsl */`
  attribute float birth;
  attribute float range01;
  uniform float uTime;
  uniform float uFade;
  uniform float uSize;
  varying float vAlpha;
  varying vec3 vColor;
  void main() {
    float age = uTime - birth;
    float live = step(0.0, age) * step(age, uFade);
    float flash = exp(-max(age, 0.0) * 9.0);
    vAlpha = live * (1.0 - age / uFade) * (0.55 + 0.45 * flash);
    vec3 near = vec3(1.0, 0.25, 0.75);
    vec3 mid = vec3(0.55, 0.35, 1.0);
    vec3 far = vec3(0.2, 0.9, 1.0);
    vec3 c = range01 < 0.5 ? mix(near, mid, range01 * 2.0) : mix(mid, far, (range01 - 0.5) * 2.0);
    vColor = mix(c, vec3(1.0), flash * 0.8);
    vec4 mv = modelViewMatrix * vec4(position, 1.0);
    gl_Position = projectionMatrix * mv;
    gl_PointSize = live * uSize * (1.0 + 2.2 * flash);
  }`;
const POINT_FRAG = /* glsl */`
  uniform float uIntensity;
  varying float vAlpha;
  varying vec3 vColor;
  void main() {
    float r = length(gl_PointCoord - 0.5);
    float a = smoothstep(0.5, 0.05, r) * vAlpha;
    if (a < 0.01) discard;
    gl_FragColor = vec4(vColor * a * 2.2 * uIntensity, a * uIntensity);
  }`;

const LINE_VERT = /* glsl */`
  attribute float birth;
  attribute float along; // 0 at the sensor, 1 at the return
  uniform float uTime;
  uniform float uFade;
  varying float vAlpha;
  void main() {
    float age = uTime - birth;
    float live = step(0.0, age) * step(age, uFade * 0.55);
    vAlpha = live * (1.0 - age / (uFade * 0.55)) * mix(0.08, 0.5, along);
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }`;
const LINE_FRAG = /* glsl */`
  uniform float uIntensity;
  varying float vAlpha;
  void main() { gl_FragColor = vec4(vec3(0.62, 0.45, 1.0) * vAlpha * 1.8 * uIntensity, vAlpha * uIntensity); }`;

const toThree = (x, y, z) => [x, z, -y];

/** intensity / pointSize let tight environments (cave tunnels) tone the glow down; defaults match the industrial look. */
export function createLidarViz({ maxHits = 16000, lineEvery = { beam: 2, az: 5 }, intensity = 1, pointSize = 5.5 } = {}) {
  const group = new THREE.Group();
  group.name = 'lidar_viz';

  // --- return points ---
  const pPos = new Float32Array(maxHits * 3), pBirth = new Float32Array(maxHits).fill(1e9), pRange = new Float32Array(maxHits);
  const pGeo = new THREE.BufferGeometry();
  pGeo.setAttribute('position', new THREE.BufferAttribute(pPos, 3));
  pGeo.setAttribute('birth', new THREE.BufferAttribute(pBirth, 1));
  pGeo.setAttribute('range01', new THREE.BufferAttribute(pRange, 1));
  pGeo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 500);
  const pointUniforms = { uTime: { value: 0 }, uFade: { value: FADE }, uSize: { value: pointSize }, uIntensity: { value: intensity } };
  const pMat = new THREE.ShaderMaterial({
    uniforms: pointUniforms, vertexShader: POINT_VERT, fragmentShader: POINT_FRAG,
    transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, toneMapped: false,
  });
  const points = new THREE.Points(pGeo, pMat);
  points.frustumCulled = false;
  group.add(points);
  let pHead = 0;

  // --- beam lines ---
  const maxLines = Math.ceil(maxHits / 4);
  const lPos = new Float32Array(maxLines * 6), lBirth = new Float32Array(maxLines * 2).fill(1e9), lAlong = new Float32Array(maxLines * 2);
  for (let i = 0; i < maxLines; i++) { lAlong[i * 2] = 0; lAlong[i * 2 + 1] = 1; }
  const lGeo = new THREE.BufferGeometry();
  lGeo.setAttribute('position', new THREE.BufferAttribute(lPos, 3));
  lGeo.setAttribute('birth', new THREE.BufferAttribute(lBirth, 1));
  lGeo.setAttribute('along', new THREE.BufferAttribute(lAlong, 1));
  lGeo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 500);
  const lMat = new THREE.ShaderMaterial({
    uniforms: { uTime: pointUniforms.uTime, uFade: pointUniforms.uFade, uIntensity: pointUniforms.uIntensity }, vertexShader: LINE_VERT, fragmentShader: LINE_FRAG,
    transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, toneMapped: false,
  });
  const lines = new THREE.LineSegments(lGeo, lMat);
  lines.frustumCulled = false;
  group.add(lines);
  let lHead = 0;

  // --- floor visibility fan + rim ---
  const azCount = Math.round(360 / LIDAR.azStepDeg);
  const fanPos = new Float32Array((azCount + 1) * 3), fanCol = new Float32Array((azCount + 1) * 3);
  const fanIdx = [];
  for (let i = 0; i < azCount; i++) fanIdx.push(0, 1 + i, 1 + ((i + 1) % azCount));
  const fanGeo = new THREE.BufferGeometry();
  fanGeo.setAttribute('position', new THREE.BufferAttribute(fanPos, 3));
  fanGeo.setAttribute('color', new THREE.BufferAttribute(fanCol, 3));
  fanGeo.setIndex(fanIdx);
  fanGeo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 500);
  const fan = new THREE.Mesh(fanGeo, new THREE.MeshBasicMaterial({
    vertexColors: true, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, toneMapped: false, side: THREE.DoubleSide,
    polygonOffset: true, polygonOffsetFactor: -3, polygonOffsetUnits: -3,
  }));
  fan.frustumCulled = false;
  fan.renderOrder = 4;
  group.add(fan);
  const rimPos = new Float32Array(azCount * 3);
  const rimGeo = new THREE.BufferGeometry();
  rimGeo.setAttribute('position', new THREE.BufferAttribute(rimPos, 3));
  rimGeo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 500);
  const rim = new THREE.LineLoop(rimGeo, new THREE.LineBasicMaterial({ color: '#b48cff', transparent: true, opacity: 0.9, toneMapped: false, blending: THREE.AdditiveBlending, depthWrite: false }));
  rim.frustumCulled = false; rim.renderOrder = 5;
  group.add(rim);

  // --- rotating scan line + halo ---
  const sweepGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(1, 0, 0)]);
  const sweepLine = new THREE.Line(sweepGeo, new THREE.LineBasicMaterial({ color: '#ffffff', transparent: true, opacity: 0.95, toneMapped: false, blending: THREE.AdditiveBlending, depthWrite: false }));
  sweepLine.frustumCulled = false; sweepLine.renderOrder = 6;
  group.add(sweepLine);
  const haloTex = (() => {
    const c = document.createElement('canvas'); c.width = c.height = 64;
    const g = c.getContext('2d'); const gr = g.createRadialGradient(32, 32, 0, 32, 32, 32);
    gr.addColorStop(0, 'rgba(190,150,255,1)'); gr.addColorStop(0.35, 'rgba(140,90,255,0.35)'); gr.addColorStop(1, 'rgba(140,90,255,0)');
    g.fillStyle = gr; g.fillRect(0, 0, 64, 64);
    return new THREE.CanvasTexture(c);
  })();
  const halo = new THREE.Sprite(new THREE.SpriteMaterial({ map: haloTex, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, toneMapped: false }));
  halo.scale.set(0.9, 0.9, 1); halo.renderOrder = 7;
  group.add(halo);

  const state = { t0: -1, origin: [0, 0, 0], yaw: 0, ring: new Float32Array(azCount).fill(FAN_RANGE), floorY: 0 };
  const fanBeam = Math.round((0 - LIDAR.elevMinDeg) / ((LIDAR.elevMaxDeg - LIDAR.elevMinDeg) / (LIDAR.beams - 1))); // beam closest to 0 deg

  /** Feed one finished sweep. pose = {x, y, z (sensor height), yaw}; time = sim time the sweep started. */
  function addScan(rays, pose, time, floorZ = 0) {
    state.t0 = time; state.origin = [pose.x, pose.y, pose.z]; state.yaw = pose.yaw; state.floorY = floorZ;
    let any = false;
    for (let b = 0; b < LIDAR.beams; b++) {
      for (let a = 0; a < azCount; a++) {
        const r = rays[b * azCount + a];
        const birth = time + (a / azCount) * SCAN_PERIOD;
        if (b === fanBeam) {
          const hr = Math.min(r.range * Math.hypot(r.dx, r.dy), FAN_RANGE);
          state.ring[a] = r.hit ? hr : FAN_RANGE;
        }
        if (!r.hit) continue;
        const ex = pose.x + r.dx * r.range, ey = pose.y + r.dy * r.range, ez = pose.z + r.dz * r.range;
        const o = pHead * 3;
        [pPos[o], pPos[o + 1], pPos[o + 2]] = toThree(ex, ey, ez);
        pBirth[pHead] = birth; pRange[pHead] = Math.min(r.range / 18, 1);
        pHead = (pHead + 1) % maxHits; any = true;
        if (b % lineEvery.beam === 0 && a % lineEvery.az === 0) {
          const lo = lHead * 6;
          [lPos[lo], lPos[lo + 1], lPos[lo + 2]] = toThree(pose.x, pose.y, pose.z);
          [lPos[lo + 3], lPos[lo + 4], lPos[lo + 5]] = toThree(ex, ey, ez);
          lBirth[lHead * 2] = birth; lBirth[lHead * 2 + 1] = birth;
          lHead = (lHead + 1) % maxLines;
        }
      }
    }
    if (any) for (const attr of [pGeo.attributes.position, pGeo.attributes.birth, pGeo.attributes.range01, lGeo.attributes.position, lGeo.attributes.birth]) attr.needsUpdate = true;

    // visibility fan + rim at floor level (centre = sensor footprint)
    const fy = floorZ + 0.05;
    const c0 = toThree(pose.x, pose.y, fy);
    [fanPos[0], fanPos[1], fanPos[2]] = c0;
    fanCol[0] = 0.35; fanCol[1] = 0.2; fanCol[2] = 0.7;
    for (let a = 0; a < azCount; a++) {
      const ang = pose.yaw + (a * LIDAR.azStepDeg * Math.PI) / 180;
      const r = state.ring[a];
      const p = toThree(pose.x + Math.cos(ang) * r, pose.y + Math.sin(ang) * r, fy);
      const o = (a + 1) * 3;
      [fanPos[o], fanPos[o + 1], fanPos[o + 2]] = p;
      [rimPos[a * 3], rimPos[a * 3 + 1], rimPos[a * 3 + 2]] = p;
      const k = Math.pow(1 - r / FAN_RANGE, 0.8) * 0.16 + 0.025;
      fanCol[o] = 0.55 * k * 2.4 * intensity; fanCol[o + 1] = 0.3 * k * 2.4 * intensity; fanCol[o + 2] = 1.0 * k * 2.4 * intensity;
    }
    fanGeo.attributes.position.needsUpdate = true; fanGeo.attributes.color.needsUpdate = true; rimGeo.attributes.position.needsUpdate = true;
  }

  /** Per-frame: advance shader time and place the rotating scan line + halo. */
  function frame(simTime) {
    pointUniforms.uTime.value = simTime;
    if (state.t0 < 0) { sweepLine.visible = false; return; }
    sweepLine.visible = true;
    const phase = ((simTime - state.t0) / SCAN_PERIOD) % 1;
    const ang = state.yaw + phase * Math.PI * 2;
    const idx = Math.min(azCount - 1, Math.floor(phase * azCount));
    const r = state.ring[idx];
    const [ox, oy, oz] = state.origin;
    const a = toThree(ox, oy, oz), b = toThree(ox + Math.cos(ang) * r, oy + Math.sin(ang) * r, state.floorY + 0.05);
    const pos = sweepGeo.attributes.position;
    pos.setXYZ(0, ...a); pos.setXYZ(1, ...b); pos.needsUpdate = true;
    halo.position.set(a[0], a[1] + 0.05, a[2]);
  }

  return {
    group, addScan, frame,
    clear() { pBirth.fill(1e9); lBirth.fill(1e9); pGeo.attributes.birth.needsUpdate = true; lGeo.attributes.birth.needsUpdate = true; state.t0 = -1; },
    dispose() { pGeo.dispose(); pMat.dispose(); lGeo.dispose(); lMat.dispose(); fanGeo.dispose(); fan.material.dispose(); rimGeo.dispose(); rim.material.dispose(); sweepGeo.dispose(); sweepLine.material.dispose(); halo.material.dispose(); haloTex.dispose(); },
  };
}
