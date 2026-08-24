/*
 * Three Body Problem — web port
 *
 * Physics is a direct port of the C++ simulation in github.com/lemus-D/3BodyProblem:
 *   - position Verlet integration   (src/body.cpp  -> updateBody)
 *   - pairwise Newtonian gravity    (src/force.cpp -> calculateForce)
 *   - the same six presets          (src/presets.cpp)
 * Same dt, same substep count, same softening clamp, so it evolves identically.
 */

const DT = 0.0001;           // timestep
const STEPS_PER_FRAME = 100; // substeps per frame — same 0.01 sim-time/frame
const MIN_DIST = 0.0001;     // softening clamp     (force.cpp)
const TRAIL_DEFAULT = 900;   // trail frames for the non-periodic presets

/*
 * dt was 1e-3 with 10 substeps. That's too coarse for the closer approaches in
 * the periodic orbits below — Butterfly I missed closing by 16 units at 1e-3
 * but closes to 5e-3 at 1e-4. Same wall-clock rate, 10x the resolution, and
 * still only ~300 pair-force evaluations a frame.
 */

/* Šuvakov–Dmitrašinović catalogue form: two unit masses at (∓1, 0) sharing a
   velocity, and a third at the origin moving opposite at twice the speed.
   Total momentum is zero by construction. */
function sdOrbit(p1, p2) {
  return [
    { x: -1, y: 0, vx: p1, vy: p2, mass: 1 },
    { x: 1, y: 0, vx: p1, vy: p2, mass: 1 },
    { x: 0, y: 0, vx: -2 * p1, vy: -2 * p2, mass: 1 },
  ];
}

const PRESETS = [
  {
    name: 'Figure-8',
    blurb: 'Three equal masses chasing each other down the same figure-eight track. Closes exactly.',
    G: 1.0,
    period: 6.324449,
    bodies: sdOrbit(0.347111, 0.532728),
  },
  {
    name: 'Butterfly',
    blurb: 'Butterfly I — another exact periodic solution, found by Šuvakov and Dmitrašinović in 2013.',
    G: 1.0,
    period: 6.235641,
    bodies: sdOrbit(0.306893, 0.125507),
  },
  {
    name: 'Moth',
    blurb: 'Moth I. Same three equal masses, wound into a much longer closed path.',
    G: 1.0,
    period: 14.893911,
    bodies: sdOrbit(0.464445, 0.396060),
  },
  {
    name: 'Yin-Yang',
    blurb: 'Yin-Yang I. Two bodies spiral around each other while the third loops the pair.',
    G: 1.0,
    period: 17.328380,
    bodies: sdOrbit(0.513938, 0.304736),
  },
  {
    name: 'Solar System',
    blurb: 'A dominant sun with three light planets, each launched at exactly its circular speed √(GM/r).',
    G: 1.0,
    bodies: [
      { x: 0, y: 0, vx: 0, vy: 0, mass: 10.0 },
      { x: 1.5, y: 0, vx: 0, vy: 2.58198890, mass: 0.01 },
      { x: 2.5, y: 0, vx: 0, vy: 2.0, mass: 0.01 },
      { x: -3.5, y: 0, vx: 0, vy: -1.69030851, mass: 0.01 },
    ],
  },
  {
    name: 'Lagrange Triangle',
    blurb: 'Three equal masses holding an equilateral triangle. Equal masses make it unstable — watch it eventually let go.',
    G: 1.0,
    bodies: [
      { x: 1, y: 0, vx: 0, vy: 0.75983569, mass: 1.0 },
      { x: -0.5, y: 0.86602540, vx: -0.65817928, vy: -0.37991784, mass: 1.0 },
      { x: -0.5, y: -0.86602540, vx: 0.65817928, vy: -0.37991784, mass: 1.0 },
    ],
  },
  {
    name: 'Binary + Planet',
    blurb: 'Two heavy stars in a circular binary, with a planet orbiting the pair from far enough out to survive.',
    G: 1.0,
    bodies: [
      { x: -1, y: 0, vx: 0, vy: 1.41421356, mass: 8.0 },
      { x: 1, y: 0, vx: 0, vy: -1.41421356, mass: 8.0 },
      { x: 0, y: 6, vx: 1.63299316, vy: 0, mass: 1.0 },
    ],
  },
];

const COLORS = [
  '#4b9cd3', '#ece5d8', '#f0a830', '#7bafd4',
  '#e8734a', '#8fd4c8', '#b58cd6', '#f2c76b',
  '#8fa6d9', '#a8b2c8',
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
    // Off-centre on wide screens so the orbit doesn't sit under the name.
    const wide = this.w / this.h > 1.2;
    this.cx = this.w * (wide ? 0.58 : 0.5);
    this.cy = this.h * (wide ? 0.42 : 0.38);
  }

  /** Fit the preset's initial extent to the canvas so every preset frames nicely. */
  computeScale(bodies) {
    let maxR = 0;
    for (const b of bodies) maxR = Math.max(maxR, Math.hypot(b.x, b.y));
    if (maxR < 1e-6) maxR = 1;
    const room = Math.min(this.cx, this.w - this.cx, this.cy, this.h - this.cy);
    return (room * 0.74) / maxR;
  }

  load(index) {
    const preset = PRESETS[index];
    this.presetIndex = index;
    this.G = preset.G;
    // Show exactly one closed period where there is one; otherwise a fixed window.
    this.trailMax = preset.period
      ? Math.round(preset.period / (DT * STEPS_PER_FRAME))
      : TRAIL_DEFAULT;
    // x_prev encodes the initial velocity, exactly as loadPreset() does in presets.cpp
    this.bodies = preset.bodies.map((b, i) => ({
      x: b.x,
      y: b.y,
      xPrev: b.x - b.vx * DT,
      yPrev: b.y - b.vy * DT,
      mass: b.mass,
      color: COLORS[i % COLORS.length],
      trail: [],
    }));
    this.zeroMomentum();
    this.scale = this.computeScale(this.bodies);
    this.clear();
  }

  /* Drift the whole system to rest at the origin. Without this a preset with
     net momentum slowly walks off screen, which reads as a bug. */
  zeroMomentum() {
    let M = 0, px = 0, py = 0, cx = 0, cy = 0;
    for (const b of this.bodies) {
      const vx = (b.x - b.xPrev) / DT;
      const vy = (b.y - b.yPrev) / DT;
      M += b.mass; px += b.mass * vx; py += b.mass * vy;
      cx += b.mass * b.x; cy += b.mass * b.y;
    }
    if (M === 0) return;
    cx /= M; cy /= M; px /= M; py /= M;
    for (const b of this.bodies) {
      const vx = (b.x - b.xPrev) / DT - px;
      const vy = (b.y - b.yPrev) / DT - py;
      b.x -= cx; b.y -= cy;
      b.xPrev = b.x - vx * DT;
      b.yPrev = b.y - vy * DT;
    }
  }

  reset() { this.load(this.presetIndex); }

  clear() {
    const { ctx } = this;
    ctx.fillStyle = '#08111c';
    ctx.fillRect(0, 0, this.w, this.h);
  }

  radius(mass) {
    // Cube-root so a 10x mass reads as bigger without swamping the frame.
    return Math.min(16, Math.max(4.5, 4.2 * Math.cbrt(mass) + 1.6));
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
    ctx.fillStyle = '#08111c';
    ctx.fillRect(0, 0, this.w, this.h);

    const cx = this.cx;
    const cy = this.cy;

    // Two strokes per body: the full path faint, the recent tail brighter.
    // Cheaper than per-segment alpha and reads the same.
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    for (const b of this.bodies) {
      const t = b.trail;
      if (t.length < 4) continue;

      ctx.strokeStyle = b.color;
      ctx.globalAlpha = 0.26;
      ctx.lineWidth = 1.1;
      ctx.beginPath();
      ctx.moveTo(cx + t[0] * this.scale, cy + t[1] * this.scale);
      for (let i = 2; i < t.length; i += 2) {
        ctx.lineTo(cx + t[i] * this.scale, cy + t[i + 1] * this.scale);
      }
      ctx.stroke();

      const recent = Math.max(0, t.length - 240);
      ctx.globalAlpha = 0.75;
      ctx.lineWidth = 1.7;
      ctx.beginPath();
      ctx.moveTo(cx + t[recent] * this.scale, cy + t[recent + 1] * this.scale);
      for (let i = recent + 2; i < t.length; i += 2) {
        ctx.lineTo(cx + t[i] * this.scale, cy + t[i + 1] * this.scale);
      }
      ctx.stroke();
    }
    ctx.globalAlpha = 1;

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
      ctx.strokeStyle = '#4b9cd3';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(this.drag.x0, this.drag.y0);
      ctx.lineTo(this.drag.x1, this.drag.y1);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = '#4b9cd3';
      ctx.beginPath();
      ctx.arc(this.drag.x0, this.drag.y0, 5, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.globalAlpha = 1;
  }

  frame() {
    if (this.running && this.visible) {
      for (let s = 0; s < STEPS_PER_FRAME; s++) this.step();
      for (const b of this.bodies) {
        b.trail.push(b.x, b.y);
        if (b.trail.length > this.trailMax * 2) b.trail.splice(0, 2);
      }
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

      const wx = (x0 - this.cx) / this.scale;
      const wy = (y0 - this.cy) / this.scale;
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
        trail: [],
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
