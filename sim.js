/*
 * Three Body Problem — web port
 *
 * Physics is a direct port of the C++ simulation in github.com/lemus-D/3BodyProblem:
 *   - position Verlet integration   (src/body.cpp  -> updateBody)
 *   - pairwise Newtonian gravity    (src/force.cpp -> calculateForce)
 *   - the same six presets          (src/presets.cpp)
 * Same dt, same substep count, same softening clamp, so it evolves identically.
 */

const DT = 0.001;          // timestep            (main.cpp)
const STEPS_PER_FRAME = 10; // physics substeps    (main.cpp)
const MIN_DIST = 0.0001;   // softening clamp     (force.cpp)

const PRESETS = [
  {
    name: 'Figure-8',
    blurb: 'Three equal masses chase each other around a figure-eight. A rare, exactly periodic solution.',
    G: 1.0,
    bodies: [
      { x: -0.97000436, y: 0.24308753, vx: 0.466203685, vy: 0.43236573, mass: 1.0 },
      { x: 0.0, y: 0.0, vx: -0.93240737, vy: -0.86473146, mass: 1.0 },
      { x: 0.97000436, y: -0.24308753, vx: 0.466203685, vy: 0.43236573, mass: 1.0 },
    ],
  },
  {
    name: 'Binary + Planet',
    blurb: 'Two heavy stars orbit each other while a light planet gets thrown around chaotically.',
    G: 1.0,
    bodies: [
      { x: -1.0, y: 0.0, vx: 0.0, vy: 0.5, mass: 3.0 },
      { x: 1.0, y: 0.0, vx: 0.0, vy: -0.5, mass: 3.0 },
      { x: 0.0, y: 3.0, vx: 1.2, vy: 0.0, mass: 0.2 },
    ],
  },
  {
    name: 'Solar System',
    blurb: 'One massive sun, three planets at different radii, each in a roughly stable orbit.',
    G: 1.0,
    bodies: [
      { x: 0.0, y: 0.0, vx: 0.0, vy: 0.0, mass: 10.0 },
      { x: 1.5, y: 0.0, vx: 0.0, vy: 0.85, mass: 0.5 },
      { x: 2.5, y: 0.0, vx: 0.0, vy: 0.65, mass: 0.5 },
      { x: 3.5, y: 0.0, vx: 0.0, vy: 0.52, mass: 0.5 },
    ],
  },
  {
    name: 'Chaotic Butterfly',
    blurb: 'Four bodies in a high-energy square. Small differences blow up fast — this one never repeats.',
    G: 1.0,
    bodies: [
      { x: -1.0, y: -1.0, vx: 0.3, vy: 0.3, mass: 1.0 },
      { x: 1.0, y: -1.0, vx: -0.3, vy: 0.3, mass: 1.0 },
      { x: -1.0, y: 1.0, vx: 0.3, vy: -0.3, mass: 1.0 },
      { x: 1.0, y: 1.0, vx: -0.3, vy: -0.3, mass: 1.0 },
    ],
  },
  {
    name: 'Lagrange Triangle',
    blurb: 'Three equal masses holding an equilateral triangle as it rotates. Stable, and it looks it.',
    G: 1.0,
    bodies: [
      { x: 1.0, y: 0.0, vx: 0.0, vy: 0.577, mass: 1.0 },
      { x: -0.5, y: 0.866, vx: -0.5, vy: -0.289, mass: 1.0 },
      { x: -0.5, y: -0.866, vx: 0.5, vy: -0.289, mass: 1.0 },
    ],
  },
  {
    name: 'Pythagorean',
    blurb: 'Masses in a 3-4-5 ratio released from rest-ish. Builds to a violent slingshot ejection.',
    G: 1.0,
    bodies: [
      { x: -1.0, y: 0.0, vx: 0.35, vy: 0.4, mass: 3.0 },
      { x: 0.5, y: 0.0, vx: 0.25, vy: 0.5, mass: 4.0 },
      { x: 1.5, y: 0.0, vx: -0.6, vy: -0.9, mass: 5.0 },
    ],
  },
];

const COLORS = [
  '#f0a830', '#ece5d8', '#e8734a', '#5fb8d4',
  '#b58cd6', '#7fd4a8', '#f2c76b', '#8fa6d9',
  '#d98cae', '#a8b2c8',
];

/** Newtonian pair force — port of calculateForce() in src/force.cpp */
function pairForce(a, b, G) {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  let dist = Math.sqrt(dx * dx + dy * dy);
  if (dist < MIN_DIST) dist = MIN_DIST;
  const F = (G * a.mass * b.mass) / (dist * dist);
  return { fx: F * (dx / dist), fy: F * (dy / dist) };
}

/** Position-Verlet step — port of updateBody() in src/body.cpp */
function integrate(body, fx, fy, dt) {
  const ax = fx / body.mass;
  const ay = fy / body.mass;
  const xNew = 2 * body.x - body.xPrev + ax * dt * dt;
  const yNew = 2 * body.y - body.yPrev + ay * dt * dt;
  body.xPrev = body.x;
  body.yPrev = body.y;
  body.x = xNew;
  body.y = yNew;
}

class ThreeBodySim {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d', { alpha: false });
    this.bodies = [];
    this.G = 1.0;
    this.scale = 100;
    this.running = true;
    this.visible = true;
    this.presetIndex = 0;
    this.drag = null;
    this.dpr = 1;

    this.resize();
    this.load(0);

    this._onResize = () => { this.resize(); this.clear(); };
    window.addEventListener('resize', this._onResize);

    // Pause when scrolled out of view so it isn't burning battery in a background tab.
    if ('IntersectionObserver' in window) {
      new IntersectionObserver(
        (entries) => { this.visible = entries[0].isIntersecting; },
        { threshold: 0.01 }
      ).observe(canvas);
    }

    this.bindInteraction();
    requestAnimationFrame(() => this.frame());
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    // Cap DPR at 2 — beyond that we're pushing pixels for no visible gain.
    this.dpr = Math.min(window.devicePixelRatio || 1, 2);
    this.canvas.width = Math.max(1, Math.round(rect.width * this.dpr));
    this.canvas.height = Math.max(1, Math.round(rect.height * this.dpr));
    this.ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    this.w = rect.width;
    this.h = rect.height;
  }

  /** Fit the preset's initial extent to the canvas so every preset frames nicely. */
  computeScale(bodies) {
    let maxR = 0;
    for (const b of bodies) maxR = Math.max(maxR, Math.hypot(b.x, b.y));
    if (maxR < 1e-6) maxR = 1;
    return (Math.min(this.w, this.h) * 0.36) / maxR;
  }

  load(index) {
    const preset = PRESETS[index];
    this.presetIndex = index;
    this.G = preset.G;
    // x_prev encodes the initial velocity, exactly as loadPreset() does in presets.cpp
    this.bodies = preset.bodies.map((b, i) => ({
      x: b.x,
      y: b.y,
      xPrev: b.x - b.vx * DT,
      yPrev: b.y - b.vy * DT,
      mass: b.mass,
      color: COLORS[i % COLORS.length],
    }));
    this.scale = this.computeScale(this.bodies);
    this.clear();
  }

  reset() { this.load(this.presetIndex); }

  clear() {
    const { ctx } = this;
    ctx.fillStyle = '#0a0e1a';
    ctx.fillRect(0, 0, this.w, this.h);
  }

  radius(mass) {
    // Cube-root so a 10x mass reads as bigger without swamping the frame.
    return Math.min(13, Math.max(3.5, 3.2 * Math.cbrt(mass) + 1.4));
  }

  step() {
    const n = this.bodies.length;
    const fx = new Float64Array(n);
    const fy = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        const f = pairForce(this.bodies[i], this.bodies[j], this.G);
        fx[i] += f.fx; fy[i] += f.fy;
        fx[j] -= f.fx; fy[j] -= f.fy;
      }
    }
    for (let i = 0; i < n; i++) integrate(this.bodies[i], fx[i], fy[i], DT);
  }

  draw() {
    const { ctx } = this;
    // Translucent wipe instead of a hard clear — this is what leaves the trails.
    ctx.fillStyle = 'rgba(10, 14, 26, 0.11)';
    ctx.fillRect(0, 0, this.w, this.h);

    const cx = this.w / 2;
    const cy = this.h / 2;

    for (const b of this.bodies) {
      const sx = cx + b.x * this.scale;
      const sy = cy + b.y * this.scale;
      if (!Number.isFinite(sx) || !Number.isFinite(sy)) continue;
      const r = this.radius(b.mass);

      const glow = ctx.createRadialGradient(sx, sy, 0, sx, sy, r * 4);
      glow.addColorStop(0, b.color);
      glow.addColorStop(1, 'rgba(0,0,0,0)');
      ctx.globalAlpha = 0.35;
      ctx.fillStyle = glow;
      ctx.beginPath();
      ctx.arc(sx, sy, r * 4, 0, Math.PI * 2);
      ctx.fill();

      ctx.globalAlpha = 1;
      ctx.fillStyle = b.color;
      ctx.beginPath();
      ctx.arc(sx, sy, r, 0, Math.PI * 2);
      ctx.fill();
    }

    // Aim line while the user is dragging out a new body.
    if (this.drag) {
      ctx.globalAlpha = 0.9;
      ctx.strokeStyle = '#f0a830';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(this.drag.x0, this.drag.y0);
      ctx.lineTo(this.drag.x1, this.drag.y1);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = '#f0a830';
      ctx.beginPath();
      ctx.arc(this.drag.x0, this.drag.y0, 5, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.globalAlpha = 1;
  }

  frame() {
    if (this.running && this.visible) {
      for (let s = 0; s < STEPS_PER_FRAME; s++) this.step();
    }
    if (this.visible) this.draw();
    requestAnimationFrame(() => this.frame());
  }

  /** Drag on the canvas to fling in an extra body: press = position, release = velocity. */
  bindInteraction() {
    const pos = (e) => {
      const r = this.canvas.getBoundingClientRect();
      const p = e.touches ? e.touches[0] : e;
      return { x: p.clientX - r.left, y: p.clientY - r.top };
    };

    const start = (e) => {
      const p = pos(e);
      this.drag = { x0: p.x, y0: p.y, x1: p.x, y1: p.y };
      e.preventDefault();
    };
    const move = (e) => {
      if (!this.drag) return;
      const p = pos(e);
      this.drag.x1 = p.x;
      this.drag.y1 = p.y;
      e.preventDefault();
    };
    const end = () => {
      if (!this.drag) return;
      const { x0, y0, x1, y1 } = this.drag;
      this.drag = null;
      if (this.bodies.length >= 10) return; // MAX_BODIES, same as the C++ build

      const wx = (x0 - this.w / 2) / this.scale;
      const wy = (y0 - this.h / 2) / this.scale;
      // Pull-back-to-launch: drag away from the spawn point to set speed.
      const vx = ((x0 - x1) / this.scale) * 0.6;
      const vy = ((y0 - y1) / this.scale) * 0.6;
      this.bodies.push({
        x: wx,
        y: wy,
        xPrev: wx - vx * DT,
        yPrev: wy - vy * DT,
        mass: 1.0,
        color: COLORS[this.bodies.length % COLORS.length],
      });
    };

    this.canvas.addEventListener('mousedown', start);
    window.addEventListener('mousemove', move);
    window.addEventListener('mouseup', end);
    this.canvas.addEventListener('touchstart', start, { passive: false });
    this.canvas.addEventListener('touchmove', move, { passive: false });
    window.addEventListener('touchend', end);
  }
}

// ---- wire up to the DOM ----------------------------------------------------
document.addEventListener('DOMContentLoaded', () => {
  const canvas = document.getElementById('sim-canvas');
  if (!canvas) return;

  const sim = new ThreeBodySim(canvas);

  const blurb = document.getElementById('sim-blurb');
  const tabs = document.getElementById('sim-presets');
  const playBtn = document.getElementById('sim-play');
  const resetBtn = document.getElementById('sim-reset');

  const setBlurb = () => { if (blurb) blurb.textContent = PRESETS[sim.presetIndex].blurb; };

  PRESETS.forEach((preset, i) => {
    const b = document.createElement('button');
    b.type = 'button';
    b.className = 'sim-tab' + (i === 0 ? ' is-active' : '');
    b.textContent = preset.name;
    b.addEventListener('click', () => {
      sim.load(i);
      setBlurb();
      tabs.querySelectorAll('.sim-tab').forEach((el) => el.classList.remove('is-active'));
      b.classList.add('is-active');
    });
    tabs.appendChild(b);
  });
  setBlurb();

  const syncPlay = () => {
    playBtn.textContent = sim.running ? 'Pause' : 'Play';
    playBtn.setAttribute('aria-pressed', String(!sim.running));
  };
  playBtn.addEventListener('click', () => { sim.running = !sim.running; syncPlay(); });
  resetBtn.addEventListener('click', () => sim.reset());

  // Don't autoplay motion at people who've asked the OS for less of it.
  if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
    sim.running = false;
  }
  syncPlay();
});
