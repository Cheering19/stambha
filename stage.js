// The scroll-driven reveal: a USB stick plugs into a laptop, the lid opens, Stambha's boot menu lights the screen,
// then macOS, and the camera pushes into the screen. Progress p (0..1) comes from how far the stage has scrolled.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

function drawPillar(x, cx, cy, size, fill) {
  const s = size / 1024;
  x.save(); x.translate(cx - 512 * s, cy - 512 * s); x.scale(s, s); x.fillStyle = fill;
  for (const d of PILLAR) x.fill(new Path2D(d));
  x.restore();
}
const PILLAR = [
  'M 425.0 169.0 A 87.0 56.3 0 0 1 599.0 169.0 Z',
  'M 291.8 169.0 H 732.1 V 250.9 H 291.8 Z',
  'M 681.0 264.2 L 671.5 268.6 L 665.6 273.1 L 660.6 277.6 L 656.1 282.0 L 651.9 286.5 L 647.9 290.9 L 644.1 295.4 L 640.5 299.8 L 637.0 304.3 L 633.7 308.7 L 630.4 313.2 L 627.2 317.6 L 624.2 322.1 L 621.1 326.6 L 618.2 331.0 L 615.3 335.5 L 612.4 339.9 L 609.6 344.4 L 606.9 348.8 L 604.2 353.3 L 596.0 353.3 L 614.4 762.9 L 409.6 762.9 L 428.0 353.3 L 419.8 353.3 L 417.1 348.8 L 414.4 344.4 L 411.6 339.9 L 408.7 335.5 L 405.8 331.0 L 402.9 326.6 L 399.8 322.1 L 396.8 317.6 L 393.6 313.2 L 390.3 308.7 L 387.0 304.3 L 383.5 299.8 L 379.9 295.4 L 376.1 290.9 L 372.1 286.5 L 367.9 282.0 L 363.4 277.6 L 358.4 273.1 L 352.5 268.6 L 343.0 264.2 Z',
  'M 343.0 776.2 H 680.9 V 819.2 H 343.0 Z',
  'M 281.6 819.2 H 742.4 V 870.4 H 281.6 Z',
];

const root = document.documentElement;
const stage = document.getElementById('top');
const sticky = stage.querySelector('.sticky');
const canvas = document.getElementById('scene');
const caps = [...stage.querySelectorAll('[data-out]')];
const reduced = matchMedia('(prefers-reduced-motion: reduce)').matches;
if (reduced) root.classList.add('reduced');

const clamp = (v, a = 0, b = 1) => Math.min(b, Math.max(a, v));
const smooth = (a, b, v) => { const t = clamp((v - a) / (b - a)); return t * t * (3 - 2 * t); };
const lerp = (a, b, t) => a + (b - a) * t;
const easeOut = t => 1 - Math.pow(1 - t, 3);

function setCaps(p) {
  for (const el of caps) {
    const tin = el.dataset.in === undefined ? -1 : +el.dataset.in;
    const tout = +el.dataset.out;
    const o = (tin < 0 ? 1 : smooth(tin, tin + 0.025, p)) * (1 - smooth(tout - 0.025, tout, p));
    el.style.opacity = o.toFixed(3);
    const y = el.classList.contains('hero') ? -(1 - o) * 30 : (1 - o) * 16;
    el.style.transform = `translate(-50%, ${y.toFixed(1)}px)`;
    el.style.pointerEvents = o > 0.5 && el.classList.contains('hero') ? 'auto' : 'none';
  }
}

let renderer = null;
try {
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, powerPreference: 'high-performance' });
} catch (e) {
  root.classList.add('no-webgl');
}
if (renderer) await start();

async function start() {
  try { await Promise.all([document.fonts.load('700 40px Geist'), document.fonts.load('500 26px Geist')]); } catch (e) { /* system font */ }

  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;
  const maxAniso = renderer.capabilities.getMaxAnisotropy();
  const tex = c => { const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = maxAniso; return t; };

  const scene = new THREE.Scene();
  renderer.setClearColor(0x000000, 0);
  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
  const camera = new THREE.PerspectiveCamera(30, 1, 0.02, 60);

  const key = new THREE.DirectionalLight(0xfff1e0, 1.5); key.position.set(3, 6, 4);
  const rim = new THREE.DirectionalLight(0xF07A28, 2.6); rim.position.set(-5, 2.2, -4);
  const rim2 = new THREE.DirectionalLight(0x8FA2FF, 1.1); rim2.position.set(5, 1.6, -3);
  scene.add(key, rim, rim2, new THREE.AmbientLight(0xffffff, 0.06));

  // ---------- laptop ----------
  const W = 3.6, D = 2.5, BASE_H = 0.16, LID_H = 0.085, LD = 2.45, SW = 3.3, SH = 2.06;
  const shell = new THREE.MeshPhysicalMaterial({ color: 0x2b2f36, metalness: 0.7, roughness: 0.4, clearcoat: 0.4, clearcoatRoughness: 0.35, envMapIntensity: 0.85 });
  const black = new THREE.MeshStandardMaterial({ color: 0x050506, metalness: 0.1, roughness: 0.35 });
  const laptop = new THREE.Group();
  scene.add(laptop);

  const base = new THREE.Mesh(new RoundedBoxGeometry(W, BASE_H, D, 5, 0.06), shell);
  base.position.y = BASE_H / 2;
  laptop.add(base);

  const [deckMap, deckGlow] = deckTextures(W - 0.14, D - 0.14);
  const deckMat = new THREE.MeshStandardMaterial({ map: tex(deckMap), emissiveMap: tex(deckGlow), emissive: 0xffffff, emissiveIntensity: 0, metalness: 0.45, roughness: 0.55 });
  const deck = new THREE.Mesh(new THREE.PlaneGeometry(W - 0.14, D - 0.14), deckMat);
  deck.rotation.x = -Math.PI / 2; deck.position.y = BASE_H + 0.0015;
  laptop.add(deck);

  const hinge = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, W - 0.7, 24), new THREE.MeshStandardMaterial({ color: 0x15171b, metalness: 0.6, roughness: 0.55, envMapIntensity: 0.4 }));
  hinge.rotation.z = Math.PI / 2; hinge.position.set(0, BASE_H - 0.005, -D / 2 + 0.03);
  laptop.add(hinge);

  const port = new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.06, 0.16), black);
  port.position.set(W / 2 + 0.001, BASE_H / 2, 0.35);
  laptop.add(port);

  const lidPivot = new THREE.Group();
  lidPivot.position.set(0, BASE_H + 0.004, -D / 2 + 0.03);
  laptop.add(lidPivot);
  const lid = new THREE.Mesh(new RoundedBoxGeometry(W, LID_H, LD, 5, 0.05), shell);
  lid.position.set(0, LID_H / 2, LD / 2);
  lidPivot.add(lid);
  const bezel = new THREE.Mesh(new THREE.PlaneGeometry(W - 0.1, LD - 0.1), new THREE.MeshStandardMaterial({ color: 0x030304, metalness: 0, roughness: 0.85, envMapIntensity: 0.08 }));
  bezel.rotation.x = Math.PI / 2; bezel.position.set(0, -0.001, LD / 2);
  lidPivot.add(bezel);

  const screenCanvas = document.createElement('canvas');
  screenCanvas.width = 1600; screenCanvas.height = 1000;
  const screenTex = tex(screenCanvas);
  const screen = new THREE.Mesh(new THREE.PlaneGeometry(SW, SH), new THREE.MeshBasicMaterial({ map: screenTex, toneMapped: false }));
  screen.rotation.x = Math.PI / 2; screen.position.set(0, -0.002, LD / 2 + 0.05);
  lidPivot.add(screen);
  const glass = new THREE.Mesh(new THREE.PlaneGeometry(SW, SH), new THREE.MeshPhysicalMaterial({ color: 0x000000, transparent: true, opacity: 0.12, roughness: 0.08, metalness: 0, envMapIntensity: 1.0 }));
  glass.rotation.x = Math.PI / 2; glass.position.set(0, -0.003, LD / 2 + 0.05);
  lidPivot.add(glass);

  const logo = new THREE.Mesh(new THREE.PlaneGeometry(0.5, 0.5), new THREE.MeshStandardMaterial({ map: tex(logoTexture()), transparent: true, color: 0xb8bec8, metalness: 0.9, roughness: 0.22, opacity: 0.6 }));
  logo.rotation.x = -Math.PI / 2; logo.position.set(0, LID_H + 0.001, LD / 2);
  lidPivot.add(logo);

  const screenLight = new THREE.PointLight(0xc8d6ff, 0, 5, 1.6);
  scene.add(screenLight);

  // ---------- USB stick ----------
  const stick = new THREE.Group();
  const stickBody = new THREE.Mesh(new RoundedBoxGeometry(0.62, 0.075, 0.2, 4, 0.03),
    new THREE.MeshPhysicalMaterial({ color: 0xF07A28, metalness: 0.25, roughness: 0.32, clearcoat: 1, clearcoatRoughness: 0.15 }));
  stickBody.position.x = 0.48;
  const plug = new THREE.Mesh(new THREE.BoxGeometry(0.17, 0.045, 0.13), new THREE.MeshStandardMaterial({ color: 0xd9dde3, metalness: 1, roughness: 0.22 }));
  plug.position.x = 0.085;
  const ledMat = new THREE.MeshStandardMaterial({ color: 0x331a00, emissive: 0xFFD27A, emissiveIntensity: 0 });
  const led = new THREE.Mesh(new THREE.SphereGeometry(0.014, 16, 10), ledMat);
  led.position.set(0.72, 0.04, 0);
  stick.add(stickBody, plug, led);
  laptop.add(stick);
  const DOCK = new THREE.Vector3(W / 2 - 0.14, BASE_H / 2, 0.35);
  const PRE = new THREE.Vector3(W / 2 + 0.75, BASE_H / 2, 0.35);
  const FAR = new THREE.Vector3(W / 2 + 3.4, BASE_H / 2 + 1.1, 1.8);

  // ---------- floor glow ----------
  const glow = new THREE.Mesh(new THREE.PlaneGeometry(9, 9), new THREE.MeshBasicMaterial({ map: tex(radial('rgba(240,122,40,0.22)')), transparent: true, depthWrite: false, blending: THREE.AdditiveBlending }));
  glow.rotation.x = -Math.PI / 2; glow.position.y = -0.002;
  laptop.add(glow);
  const shadow = new THREE.Mesh(new THREE.PlaneGeometry(5.2, 4), new THREE.MeshBasicMaterial({ map: tex(radial('rgba(0,0,0,0.85)')), transparent: true, depthWrite: false }));
  shadow.rotation.x = -Math.PI / 2; shadow.position.y = -0.001;
  laptop.add(shadow);

  // ---------- screen content ----------
  const macCanvas = macPlaceholder();
  const real = new Image();
  real.onload = () => { const x = macCanvas.getContext('2d'); x.drawImage(real, 0, 0, 1600, 1000); lastKey = ''; };
  real.src = 'media/macos-desktop.jpg';
  let lastKey = '';
  function drawScreen(p) {
    const boot = smooth(0.47, 0.53, p), starting = smooth(0.56, 0.645, p), mac = smooth(0.64, 0.7, p);
    const k = [boot, starting, mac].map(v => Math.round(v * 120)).join(',');
    if (k === lastKey) return;
    lastKey = k;
    const x = screenCanvas.getContext('2d');
    x.globalAlpha = 1; x.fillStyle = '#000'; x.fillRect(0, 0, 1600, 1000);
    if (boot > 0 && mac < 1) { x.globalAlpha = boot; drawBoot(x, starting); }
    if (mac > 0) { x.globalAlpha = mac; x.drawImage(macCanvas, 0, 0); }
    x.globalAlpha = 1;
    screenTex.needsUpdate = true;
  }

  // ---------- camera path ----------
  const KEYS = [
    [0.00, [3.9, 3.3, 6.2], [0, 1.0, 0.3]],
    [0.07, [3.6, 2.9, 5.5], [0.25, 0.9, 0.3]],
    [0.18, [5.3, 1.05, 2.3], [1.6, 0.12, 0.35]],
    [0.28, [4.7, 1.45, 2.7], [1.2, 0.2, 0.25]],
    [0.45, [0.0, 2.35, 6.4], [0, 0.95, -0.5]],
    [0.62, [0.0, 1.85, 4.9], [0, 1.2, -1.2]],
    [0.82, [0.0, 1.62, 3.7], [0, 1.33, -1.45]],
  ];
  const sPos = new THREE.Vector3(), sNorm = new THREE.Vector3(), sQuat = new THREE.Quaternion();
  const pos = new THREE.Vector3(), look = new THREE.Vector3(), tmpA = new THREE.Vector3(), tmpB = new THREE.Vector3();
  function cameraAt(p, t) {
    const far = camera.aspect < 1 ? clamp(1.05 / camera.aspect, 1, 2.3) : 1;
    let i = KEYS.length - 1;
    while (i > 0 && KEYS[i][0] > p) i--;
    if (i === KEYS.length - 1) {
      // last leg: push in until the screen fills the view
      const [k0, p0, l0] = KEYS[i];
      screen.getWorldPosition(sPos);
      screen.getWorldQuaternion(sQuat);
      sNorm.set(0, 0, 1).applyQuaternion(sQuat);
      const half = Math.tan(THREE.MathUtils.degToRad(camera.fov / 2));
      const dist = Math.min((SH / 2) / half, (SW / 2) / (half * camera.aspect)) * 0.9;
      const u = smooth(k0, 0.99, p);
      tmpA.fromArray(p0).sub(tmpB.fromArray(l0)).multiplyScalar(far).add(tmpB);
      pos.copy(tmpA).lerp(tmpB.copy(sPos).addScaledVector(sNorm, dist), u);
      look.fromArray(l0).lerp(sPos, u);
    } else {
      const [k0, p0, l0] = KEYS[i], [k1, p1, l1] = KEYS[i + 1];
      const u = smooth(k0, k1, p);
      look.fromArray(l0).lerp(tmpA.fromArray(l1), u);
      pos.fromArray(p0).lerp(tmpB.fromArray(p1), u).sub(look).multiplyScalar(far).add(look);
    }
    const idle = 1 - smooth(0, 0.07, p);
    pos.x += Math.sin(t * 0.35) * 0.35 * idle;
    pos.y += Math.sin(t * 0.5) * 0.08 * idle;
    camera.position.copy(pos);
    camera.lookAt(look);
  }

  // ---------- frame ----------
  function pose(p, t) {
    lidPivot.rotation.x = -THREE.MathUtils.degToRad(105) * smooth(0.29, 0.45, p);
    const a = easeOut(smooth(0.09, 0.21, p)), b = smooth(0.21, 0.255, p);
    if (b > 0) stick.position.copy(PRE).lerp(DOCK, b); else stick.position.copy(FAR).lerp(PRE, a);
    stick.rotation.set(0, (1 - a) * 0.9, (1 - a) * 0.45);
    ledMat.emissiveIntensity = smooth(0.255, 0.265, p) * (2.4 + Math.sin(t * 6) * 0.6 * smooth(0.47, 0.6, p));
    deckMat.emissiveIntensity = smooth(0.47, 0.53, p) * 0.85;
    const lit = smooth(0.47, 0.53, p);
    screen.getWorldPosition(sPos); screen.getWorldQuaternion(sQuat);
    sNorm.set(0, 0, 1).applyQuaternion(sQuat);
    screenLight.position.copy(sPos).addScaledVector(sNorm, 0.7);
    screenLight.intensity = lit * 2.2;
    glass.material.opacity = 0.12 * (1 - smooth(0.8, 0.93, p));
    drawScreen(p);
    cameraAt(p, t);
  }

  function resize() {
    const w = sticky.clientWidth, h = sticky.clientHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  addEventListener('resize', resize);
  resize();

  const scrollP = () => reduced ? 0.7 : clamp(-stage.getBoundingClientRect().top / (stage.offsetHeight - innerHeight));
  let cur = scrollP(), visible = true;
  new IntersectionObserver(([e]) => { visible = e.isIntersecting; }, { rootMargin: '100px' }).observe(stage);
  const t0 = performance.now();
  canvas.classList.add('ready');
  function frame(now) {
    requestAnimationFrame(frame);
    if (!visible) return;
    const t = (now - t0) / 1000;
    const target = scrollP();
    cur += (target - cur) * (reduced ? 1 : 0.12);
    if (Math.abs(target - cur) < 0.0004) cur = target;
    laptop.position.y = reduced ? 0 : lerp(-0.45, 0, easeOut(clamp(t / 1.8)));
    pose(cur, reduced ? 0 : t);
    setCaps(reduced ? 0 : cur);
    renderer.render(scene, camera);
  }
  requestAnimationFrame(frame);

  // ---------- canvas drawings ----------
  function rr(x, a, b, w, h, r) { x.beginPath(); x.roundRect(a, b, w, h, r); }

  function deckTextures(dw, dd) {
    const w = 2048, h = Math.round(2048 * dd / dw);
    const c = document.createElement('canvas'), g = document.createElement('canvas');
    c.width = g.width = w; c.height = g.height = h;
    const x = c.getContext('2d'), y = g.getContext('2d');
    x.fillStyle = '#23262c'; x.fillRect(0, 0, w, h);
    for (let i = 0; i < 7000; i++) { x.fillStyle = `rgba(255,255,255,${Math.random() * 0.025})`; x.fillRect(Math.random() * w, Math.random() * h, 20 + Math.random() * 70, 1); }
    y.fillStyle = '#000'; y.fillRect(0, 0, w, h);
    const rows = [Array(16).fill(1), [...Array(13).fill(1), 2], [1.5, ...Array(12).fill(1), 1.5], [1.75, ...Array(11).fill(1), 2.25],
      [2.25, ...Array(10).fill(1), 2.75], [1.25, 1, 1.25, 1.25, 4.75, 1.25, 1.25, 1, 1, 1]];
    const kbW = w * 0.86, unit = kbW / 15, left = (w - kbW) / 2, gap = unit * 0.13;
    let top = h * 0.07;
    rows.forEach((row, ri) => {
      const rowH = ri === 0 ? unit * 0.62 : unit, scale = 15 / row.reduce((s, k) => s + k, 0);
      let cx = left;
      for (const k of row) {
        const kw = k * unit * scale, bx = cx + gap / 2, by = top + gap / 2, bw = kw - gap, bh = rowH - gap;
        rr(x, bx, by, bw, bh, unit * 0.12); x.fillStyle = '#111317'; x.fill();
        x.strokeStyle = 'rgba(255,255,255,0.06)'; x.lineWidth = 2; x.stroke();
        y.save(); y.shadowColor = 'rgba(240,122,40,0.95)'; y.shadowBlur = unit * 0.22;
        rr(y, bx, by, bw, bh, unit * 0.12); y.strokeStyle = 'rgba(240,122,40,0.32)'; y.lineWidth = 3; y.stroke();
        y.fillStyle = 'rgba(255,200,130,0.95)'; y.fillRect(bx + unit * 0.16, by + unit * 0.14, Math.min(unit * 0.24, bw * 0.3), unit * 0.06);
        y.restore();
        cx += kw;
      }
      top += rowH;
    });
    const tw = w * 0.34, th = h * 0.29;
    rr(x, (w - tw) / 2, h - th - h * 0.06, tw, th, 30); x.fillStyle = '#292d34'; x.fill();
    x.strokeStyle = 'rgba(255,255,255,0.09)'; x.lineWidth = 3; x.stroke();
    return [c, g];
  }

  function radial(color) {
    const c = document.createElement('canvas'); c.width = c.height = 512;
    const x = c.getContext('2d'), g = x.createRadialGradient(256, 256, 0, 256, 256, 256);
    g.addColorStop(0, color); g.addColorStop(1, 'rgba(0,0,0,0)');
    x.fillStyle = g; x.fillRect(0, 0, 512, 512);
    return c;
  }

  function logoTexture() {
    const c = document.createElement('canvas'); c.width = c.height = 512;
    drawPillar(c.getContext('2d'), 256, 256, 500, '#ffffff');
    return c;
  }

  function drawBoot(x, starting) {
    const g = x.createRadialGradient(800, 180, 40, 800, 400, 900);
    g.addColorStop(0, '#1c2458'); g.addColorStop(1, '#05060f');
    x.fillStyle = g; x.fillRect(0, 0, 1600, 1000);
    const lg = x.createLinearGradient(0, 130, 0, 330); lg.addColorStop(0, '#FFD27A'); lg.addColorStop(1, '#F07A28');
    drawPillar(x, 800, 230, 210, lg);
    x.textAlign = 'center'; x.textBaseline = 'middle';
    x.fillStyle = '#EDE6D6'; x.font = '600 40px Geist, sans-serif'; x.fillText('Stambha', 800, 372);
    if (starting <= 0.01) {
      rr(x, 520, 450, 560, 76, 18); x.fillStyle = '#F07A28'; x.fill();
      x.fillStyle = '#1a0d02'; x.font = '600 32px Geist, sans-serif'; x.fillText('macOS', 800, 489);
      x.fillStyle = '#c9c3b6'; x.font = '500 32px Geist, sans-serif'; x.fillText('Install macOS', 800, 575);
      x.fillStyle = '#7d7a90'; x.font = '500 24px Geist, sans-serif';
      x.fillText('Up and down to choose    Enter to start    W for Wi-Fi', 800, 890);
    } else {
      x.fillStyle = '#EDE6D6'; x.font = '500 30px Geist, sans-serif'; x.fillText('Starting macOS', 800, 500);
      rr(x, 560, 550, 480, 8, 4); x.fillStyle = 'rgba(255,255,255,0.12)'; x.fill();
      rr(x, 560, 550, Math.max(8, 480 * starting), 8, 4); x.fillStyle = '#FFD27A'; x.fill();
    }
  }

  function macPlaceholder() {
    // Stand-in until a real screenshot is at media/macos-desktop.jpg: a generic desktop, no Apple artwork.
    const c = document.createElement('canvas'); c.width = 1600; c.height = 1000;
    const x = c.getContext('2d');
    const g = x.createLinearGradient(0, 0, 1600, 1000);
    g.addColorStop(0, '#101a4a'); g.addColorStop(0.55, '#3b2a78'); g.addColorStop(1, '#e0743a');
    x.fillStyle = g; x.fillRect(0, 0, 1600, 1000);
    for (const [cx, cy, r, col] of [[1250, 780, 520, 'rgba(255,170,90,0.45)'], [300, 250, 480, 'rgba(90,120,255,0.35)'], [900, 450, 380, 'rgba(200,110,255,0.25)']]) {
      const rg = x.createRadialGradient(cx, cy, 0, cx, cy, r); rg.addColorStop(0, col); rg.addColorStop(1, 'rgba(0,0,0,0)');
      x.fillStyle = rg; x.fillRect(0, 0, 1600, 1000);
    }
    x.fillStyle = 'rgba(20,20,30,0.35)'; x.fillRect(0, 0, 1600, 36);
    x.fillStyle = 'rgba(255,255,255,0.85)';
    [40, 110, 175, 240, 300].forEach((px, i) => { rr(x, px, 13, i === 0 ? 46 : 42, 10, 5); x.fill(); });
    rr(x, 1380, 13, 190, 10, 5); x.fill();
    rr(x, 250, 170, 760, 500, 22); x.fillStyle = 'rgba(238,238,242,0.94)'; x.fill();
    rr(x, 250, 170, 760, 52, [22, 22, 0, 0]); x.fillStyle = 'rgba(220,220,226,0.98)'; x.fill();
    ['#9a9aa2', '#9a9aa2', '#9a9aa2'].forEach((col, i) => { x.beginPath(); x.arc(282 + i * 30, 196, 9, 0, Math.PI * 2); x.fillStyle = col; x.fill(); });
    x.fillStyle = 'rgba(0,0,0,0.14)';
    for (let i = 0; i < 7; i++) { rr(x, 290, 260 + i * 52, i % 3 === 0 ? 420 : 620, 18, 9); x.fill(); }
    rr(x, 430, 880, 740, 96, 28); x.fillStyle = 'rgba(255,255,255,0.22)'; x.fill();
    const icons = ['#2f8cff', '#36c759', '#ff9f0a', '#ff375f', '#bf5af2', '#64d2ff', '#ffd60a', '#ac8e68', '#30d158'];
    icons.forEach((col, i) => { rr(x, 456 + i * 78, 896, 64, 64, 15); x.fillStyle = col; x.fill(); });
    return c;
  }
}
