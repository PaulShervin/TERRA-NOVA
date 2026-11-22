// Purple LiDAR-sweep graphic drawn on the floor around the robot (visual only, not sensor data).
import * as THREE from 'three';

function scanTexture() {
  const S = 1024, c = document.createElement('canvas');
  c.width = c.height = S;
  const g = c.getContext('2d');
  const cx = S / 2;
  const grad = g.createRadialGradient(cx, cx, 0, cx, cx, cx);
  grad.addColorStop(0, 'rgba(150,90,255,0.55)');
  grad.addColorStop(0.7, 'rgba(120,70,255,0.18)');
  grad.addColorStop(1, 'rgba(120,70,255,0)');
  g.fillStyle = grad; g.fillRect(0, 0, S, S);
  g.strokeStyle = 'rgba(190,150,255,0.55)';
  g.lineWidth = 2;
  for (let r = 40; r < cx; r += 38) { g.beginPath(); g.arc(cx, cx, r, 0, Math.PI * 2); g.stroke(); }
  g.lineWidth = 1.2;
  for (let a = 0; a < 180; a++) { // radial rays
    const th = (a / 180) * Math.PI * 2;
    g.beginPath(); g.moveTo(cx + Math.cos(th) * 30, cx + Math.sin(th) * 30);
    g.lineTo(cx + Math.cos(th) * cx * 0.98, cx + Math.sin(th) * cx * 0.98); g.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

export function createScanOverlay(radius = 7) {
  const tex = scanTexture();
  const mat = new THREE.MeshBasicMaterial({
    map: tex, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, toneMapped: false,
  });
  const mesh = new THREE.Mesh(new THREE.CircleGeometry(radius, 64).rotateX(-Math.PI / 2), mat);
  mesh.position.y = 0.04;
  mesh.renderOrder = 5;
  mesh.name = 'lidar_scan_overlay';

  // Expanding pulse ring
  const pulseMat = new THREE.MeshBasicMaterial({ color: '#b48cff', transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, toneMapped: false });
  const pulse = new THREE.Mesh(new THREE.RingGeometry(0.97, 1, 64).rotateX(-Math.PI / 2), pulseMat);
  pulse.position.y = 0.05;
  pulse.renderOrder = 6;
  mesh.add(pulse);

  return {
    mesh,
    follow: (robotRoot) => { mesh.position.x = robotRoot.position.x; mesh.position.z = robotRoot.position.z; },
    update: (time) => {
      mesh.rotation.y = time * 0.6;
      const k = (time * 0.35) % 1;
      pulse.scale.setScalar(Math.max(0.01, k * radius));
      pulse.rotation.y = -mesh.rotation.y; // keep ring axis-aligned (circle anyway)
      pulseMat.opacity = 0.7 * (1 - k);
    },
    dispose: () => { tex.dispose(); mat.dispose(); pulseMat.dispose(); mesh.geometry.dispose(); pulse.geometry.dispose(); },
  };
}
