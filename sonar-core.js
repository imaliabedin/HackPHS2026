
(function (root) {
  'use strict';

  const GOLDEN_ANGLE = 137.50776;
  const DEG = Math.PI / 180;

  function mulberry32(seed) {
    let a = seed >>> 0;
    return function () {
      a = (a + 0x6D2B79F5) >>> 0;
      let t = a;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function halton(i, b) {
    let f = 1, r = 0;
    for (; i > 0; i = Math.floor(i / b)) { f /= b; r += f * (i % b); }
    return r;
  }

  const norm = v => { const l = Math.hypot(v[0], v[1], v[2]) || 1; return [v[0] / l, v[1] / l, v[2] / l]; };
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

  /** Unit vector for azimuth/elevation in degrees (azimuth 0 = +z, 90 = +x). */
  function dirFromAzEl(az, el) {
    const ca = Math.cos(el * DEG);
    return [ca * Math.sin(az * DEG), Math.sin(el * DEG), ca * Math.cos(az * DEG)];
  }

  /**
   * Ray against a box grown by `radius` (a sphere swept along the ray). Returns the entry distance
   * and face normal, or null. Rays that start inside are ignored, like a Unity SphereCast.
   */
  function rayBox(o, d, box, radius, maxT) {
    let tmin = 0, tmax = maxT, axis = -1, sign = 0;
    for (let k = 0; k < 3; k++) {
      const lo = box.min[k] - radius, hi = box.max[k] + radius;
      if (Math.abs(d[k]) < 1e-9) {
        if (o[k] < lo || o[k] > hi) return null;
        continue;
      }
      let t1 = (lo - o[k]) / d[k], t2 = (hi - o[k]) / d[k], s = -1;
      if (t1 > t2) { const tt = t1; t1 = t2; t2 = tt; s = 1; }
      if (t1 > tmin) { tmin = t1; axis = k; sign = s; }
      if (t2 < tmax) tmax = t2;
      if (tmin > tmax) return null;
    }
    if (axis < 0) return null;
    const n = [0, 0, 0];
    n[axis] = sign;
    return { t: tmin, normal: n };
  }

  // sensor

  const SENSOR_DEFAULTS = {
    horizontalBeams: 16,
    elevations: [-30, 0, 30],
    upAndDown: true,
    subRays: 4,
    subRayAngle: 11.25,
    rotatePattern: true,
    ditherPattern: true,
    radius: 0.3,            // fat-ray radius (the Unity sensor uses 0.1 m for a 0.4 m drone)
    maxRange: 16,
    pulseRate: 10,
    perfect: false,
    rangeNoise: 0.05,
    rangeNoisePerUnit: 0.01,
    missProbability: 0.02,
    smallTargetSize: 0.3,
    smallTargetMissProbability: 0.3,
    grazingAngle: 75,
    grazingMissProbability: 0.5,
    falseEchoProbability: 0.002,
    groundY: -0.5,
    seed: 12345,
  };

  class SonarSensor {
    constructor(options = {}) {
      this.settings = Object.assign({}, SENSOR_DEFAULTS, options);
      const s = this.settings;
      this.beams = [];
      for (const el of s.elevations)
        for (let i = 0; i < s.horizontalBeams; i++) this.beams.push([i * 360 / s.horizontalBeams, el]);
      if (s.upAndDown) { this.beams.push([0, 90]); this.beams.push([0, -90]); }
      this.rand = mulberry32(s.seed);
      this.phase = Math.floor(mulberry32(s.seed ^ 0x5f3759df)() * 100000);
      this.pulseCount = 0;
      this.clock = 0;
      this.nextPulse = 0;
    }

    get samplesPerBeam() { return 1 + this.settings.subRays; }
    get queriesPerPulse() { return this.beams.length * this.samplesPerBeam; }

    axis(azEl, offset) {
      const vertical = Math.abs(azEl[1]) >= 90;
      const el = vertical ? azEl[1] - Math.sign(azEl[1]) * Math.abs(offset[1]) : azEl[1] + offset[1];
      return dirFromAzEl(azEl[0] + offset[0] * (vertical ? 16 : 1), el);
    }

    dither(n) {
      const s = this.settings;
      if (!s.ditherPattern) return [0, 0];
      const azStep = 360 / s.horizontalBeams;
      const elStep = s.elevations.length > 1 ? Math.abs(s.elevations[1] - s.elevations[0]) : 30;
      return [(halton(n + 1, 2) - 0.5) * azStep, (halton(n + 1, 3) - 0.5) * elStep];
    }

    gaussian() {
      const u1 = 1 - this.rand(), u2 = this.rand();
      return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
    }

    /** Advances the clock; returns the pulses that fired (from `origin`). */
    tick(dt, origin, boxes) {
      const fired = [];
      this.clock += dt;
      if (this.clock - this.nextPulse > 3 / this.settings.pulseRate) this.nextPulse = this.clock;
      while (this.clock >= this.nextPulse) {
        fired.push(this.pulse(origin, boxes));
        this.nextPulse += 1 / this.settings.pulseRate;
      }
      return fired;
    }

    /** One pulse from `origin` against `boxes` ({min:[x,y,z], max:[x,y,z]}). */
    pulse(origin, boxes) {
      const s = this.settings;
      const n = this.pulseCount + this.phase;
      const spin = s.rotatePattern ? n * GOLDEN_ANGLE : 0;
      const dith = this.dither(n);
      const samples = [];
      const t0 = (typeof performance !== 'undefined' ? performance : Date).now();
      for (let b = 0; b < this.beams.length; b++) {
        const axis = this.axis(this.beams[b], dith);
        samples.push(this.sample(b, origin, axis, boxes));
        if (!s.subRays) continue;
        const side = Math.abs(axis[1]) > 0.99 ? [1, 0, 0] : norm(cross([0, 1, 0], axis));
        const up = cross(axis, side);
        const ca = Math.cos(s.subRayAngle * DEG), sa = Math.sin(s.subRayAngle * DEG);
        for (let k = 0; k < s.subRays; k++) {
          const phi = (spin + k * 360 / s.subRays) * DEG;
          const c = Math.cos(phi) * sa, si = Math.sin(phi) * sa;
          samples.push(this.sample(b, origin, norm([
            axis[0] * ca + side[0] * c + up[0] * si,
            axis[1] * ca + side[1] * c + up[1] * si,
            axis[2] * ca + side[2] * c + up[2] * si,
          ]), boxes));
        }
      }
      const pulse = {
        index: this.pulseCount, time: this.clock, origin: origin.slice(), samples,
        ms: (typeof performance !== 'undefined' ? performance : Date).now() - t0,
      };
      this.pulseCount++;
      return pulse;
    }

    sample(beam, o, d, boxes) {
      const s = this.settings;
      const m = { beam, origin: o, dir: d, hasReturn: false, range: Infinity, hit: null, strength: 0, source: null, box: null, trueRange: Infinity };

      let best = Infinity, bestBox = null, bestNormal = null;
      for (const box of boxes) {
        const h = rayBox(o, d, box, s.radius, s.maxRange);
        if (h && h.t < best) { best = h.t; bestBox = box; bestNormal = h.normal; }
      }
      let objectDist = Infinity;
      if (bestBox) {
        // Distance to where the swept sphere touches the real box (what a time-of-flight would measure).
        const p = [o[0] + d[0] * best, o[1] + d[1] * best, o[2] + d[2] * best];
        const q = p.map((v, k) => Math.min(bestBox.max[k], Math.max(bestBox.min[k], v)));
        objectDist = Math.hypot(q[0] - o[0], q[1] - o[1], q[2] - o[2]);
      }
      let groundDist = d[1] < -1e-4 ? (s.groundY - o[1]) / d[1] : Infinity;
      if (groundDist < 0 || groundDist > s.maxRange) groundDist = Infinity;

      let trueDist = Infinity, cosInc = 0, size = 0;
      if (bestBox && objectDist <= groundDist && objectDist <= s.maxRange) {
        trueDist = objectDist;
        m.source = 'object';
        m.box = bestBox;
        cosInc = Math.max(0, -(d[0] * bestNormal[0] + d[1] * bestNormal[1] + d[2] * bestNormal[2]));
        size = Math.min(bestBox.max[0] - bestBox.min[0], bestBox.max[1] - bestBox.min[1], bestBox.max[2] - bestBox.min[2]);
      } else if (groundDist < Infinity) {
        trueDist = groundDist;
        m.source = 'ground';
        cosInc = -d[1];
        size = Infinity;
      }

      if (trueDist < Infinity) {
        const sizeFactor = Math.min(1, size / s.smallTargetSize);
        let miss = s.missProbability + s.smallTargetMissProbability * (1 - sizeFactor);
        if (Math.acos(Math.min(1, cosInc)) / DEG > s.grazingAngle) miss += s.grazingMissProbability;
        if (s.perfect || this.rand() >= miss) {
          const noise = s.perfect ? 0 : this.gaussian() * (s.rangeNoise + s.rangeNoisePerUnit * trueDist);
          m.hasReturn = true;
          m.range = Math.max(0, trueDist + noise);
          m.trueRange = trueDist;
          m.strength = Math.min(1, Math.max(cosInc, 0.05) * Math.sqrt(Math.max(sizeFactor, 0.05)) * (1 - 0.5 * trueDist / s.maxRange));
        } else {
          m.source = null;
          m.box = null;
        }
      }

      if (!s.perfect && this.rand() < s.falseEchoProbability) {
        const r = 0.5 + this.rand() * (s.maxRange - 0.5);
        if (!m.hasReturn || r < m.range) {
          Object.assign(m, { hasReturn: true, range: r, strength: 0.2, source: 'false', box: null, trueRange: Infinity });
        }
      }
      if (m.hasReturn) m.hit = [o[0] + d[0] * m.range, o[1] + d[1] * m.range, o[2] + d[2] * m.range];
      return m;
    }
  }

  // ---------------------------------------------------------------- belief

  class SonarBelief {
    /**
     * bounds: {min:[x,y,z], max:[x,y,z]} in whole cells (inclusive).
     * clearance: per axis, how close a grid point may be to a confirmed echo point (the drone's
     * half-size + the fat-ray radius + a margin). Grid points closer than that on every axis are blocked.
     */
    constructor(bounds, options = {}) {
      Object.assign(this, {
        hitWeight: 1, multiHitWeight: 2, clearWeight: 0.2, maxClearPerPulse: 1,
        occupyThreshold: 2, releaseThreshold: 1, maxEvidence: 4,
        clearance: [3.8, 1.05, 3.8], groundY: -0.5, groundTolerance: 0.4,
      }, options);
      this.bounds = bounds;
      this.size = [0, 1, 2].map(k => bounds.max[k] - bounds.min[k] + 1);
      const total = this.size[0] * this.size[1] * this.size[2];
      this.evidence = new Float32Array(total);
      this.occupied = new Uint8Array(total);
      this.blocked = new Uint16Array(total);
      this.hitSum = new Float32Array(total * 4);   // x, y, z, count of echo points in each cell
      this.occupiedList = new Set();
      this.occupiedPoint = new Map();             // cell -> echo point used for its blocking
    }

    id(x, y, z) {
      const b = this.bounds;
      if (x < b.min[0] || y < b.min[1] || z < b.min[2] || x > b.max[0] || y > b.max[1] || z > b.max[2]) return -1;
      return ((x - b.min[0]) * this.size[1] + (y - b.min[1])) * this.size[2] + (z - b.min[2]);
    }

    cell(id) {
      const z = id % this.size[2];
      const rest = (id - z) / this.size[2];
      const y = rest % this.size[1];
      const x = (rest - y) / this.size[1];
      return [x + this.bounds.min[0], y + this.bounds.min[1], z + this.bounds.min[2]];
    }

    clear() {
      this.evidence.fill(0);
      this.occupied.fill(0);
      this.blocked.fill(0);
      this.hitSum.fill(0);
      this.occupiedList.clear();
      this.occupiedPoint.clear();
    }

    /** Planner query: too close to a confirmed echo point, or outside the known bounds. */
    isBlocked(x, y, z) {
      const i = this.id(x, y, z);
      return i < 0 || this.blocked[i] > 0;
    }

    isGroundEcho(m) {
      if (!m.hasReturn || m.dir[1] >= -1e-4) return false;
      const expected = (this.groundY - m.origin[1]) / m.dir[1];
      return Math.abs(m.range - expected) <= this.groundTolerance + 0.03 * expected;
    }

    /** Adds one pulse; returns the cells newly confirmed as obstacles. */
    integrate(pulse, maxRange) {
      const hits = new Map(), seen = new Map();
      for (const m of pulse.samples) {
        const ground = this.isGroundEcho(m);
        const until = m.hasReturn ? m.range - 1.0 : maxRange;
        let last = -2;
        for (let t = 0.5; t <= until; t += 0.5) {
          const i = this.id(Math.round(m.origin[0] + m.dir[0] * t), Math.round(m.origin[1] + m.dir[1] * t), Math.round(m.origin[2] + m.dir[2] * t));
          if (i < 0 || i === last) continue;
          last = i;
          seen.set(i, (seen.get(i) || 0) + 1);
        }
        if (m.hasReturn && !ground) {
          const c = m.hit.map(Math.round);
          const i = this.id(c[0], c[1], c[2]);
          // Only cells the sensor can observe again (otherwise they could never be cleared).
          if (i >= 0 && Math.hypot(c[0] - m.origin[0], c[1] - m.origin[1], c[2] - m.origin[2]) <= maxRange) {
            hits.set(i, (hits.get(i) || 0) + 1);
            this.hitSum[i * 4] += m.hit[0];
            this.hitSum[i * 4 + 1] += m.hit[1];
            this.hitSum[i * 4 + 2] += m.hit[2];
            this.hitSum[i * 4 + 3] += 1;
          }
        }
      }
      const touched = new Set();
      for (const [i, n] of hits) {
        this.evidence[i] = Math.min(this.maxEvidence, this.evidence[i] + (n >= 2 ? this.multiHitWeight : this.hitWeight));
        touched.add(i);
      }
      for (const [i, n] of seen) {
        if (hits.has(i) || this.evidence[i] <= 0) continue;
        this.evidence[i] = Math.max(0, this.evidence[i] - Math.min(this.maxClearPerPulse, this.clearWeight * n));
        touched.add(i);
      }
      const newly = [];
      for (const i of touched) {
        const e = this.evidence[i];
        if (!this.occupied[i] && e >= this.occupyThreshold) { this.setOccupied(i, true); newly.push(i); }
        else if (this.occupied[i] && e < this.releaseThreshold) this.setOccupied(i, false);
        if (e <= 0) this.hitSum.fill(0, i * 4, i * 4 + 4);
      }
      return newly;
    }

    /** Marks a cell as an obstacle (or not) and blocks every grid point within `clearance` of its echo point. */
    setOccupied(i, on) {
      this.occupied[i] = on ? 1 : 0;
      let p;
      if (on) {
        const n = this.hitSum[i * 4 + 3];
        p = n > 0 ? [this.hitSum[i * 4] / n, this.hitSum[i * 4 + 1] / n, this.hitSum[i * 4 + 2] / n] : this.cell(i);
        this.occupiedList.add(i);
        this.occupiedPoint.set(i, p);
      } else {
        p = this.occupiedPoint.get(i);
        this.occupiedList.delete(i);
        this.occupiedPoint.delete(i);
      }
      const r = this.clearance;
      const lo = [0, 1, 2].map(k => Math.ceil(p[k] - r[k] + 1e-6));
      const hi = [0, 1, 2].map(k => Math.floor(p[k] + r[k] - 1e-6));
      for (let x = lo[0]; x <= hi[0]; x++)
        for (let y = lo[1]; y <= hi[1]; y++)
          for (let z = lo[2]; z <= hi[2]; z++) {
            const j = this.id(x, y, z);
            if (j >= 0) this.blocked[j] += on ? 1 : -1;
          }
    }
  }

  // ---------------------------------------------------------------- planner

  const DIRS = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];

  /** 6-connected A* from start to goal using only `belief` (unknown cells count as free). */
  function planPath(belief, start, goal, maxExpansions = 400000) {
    const total = belief.evidence.length;
    const startId = belief.id(...start), goalId = belief.id(...goal);
    if (startId < 0 || goalId < 0) return null;
    const g = new Float32Array(total).fill(Infinity);
    const prev = new Int32Array(total).fill(-1);
    const closed = new Uint8Array(total);
    const h = (x, y, z) => Math.abs(x - goal[0]) + Math.abs(y - goal[1]) + Math.abs(z - goal[2]);
    const open = [];
    const push = (id, gg, f) => {
      open.push({ id, g: gg, f });
      let i = open.length - 1;
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (open[p].f < open[i].f || (open[p].f === open[i].f && open[p].g >= open[i].g)) break;
        [open[p], open[i]] = [open[i], open[p]];
        i = p;
      }
    };
    const pop = () => {
      const top = open[0], last = open.pop();
      if (open.length) {
        open[0] = last;
        let i = 0;
        for (;;) {
          const l = 2 * i + 1, r = l + 1;
          let s = i;
          const less = (a, b) => open[a].f < open[b].f || (open[a].f === open[b].f && open[a].g > open[b].g);
          if (l < open.length && less(l, s)) s = l;
          if (r < open.length && less(r, s)) s = r;
          if (s === i) break;
          [open[s], open[i]] = [open[i], open[s]];
          i = s;
        }
      }
      return top;
    };
    g[startId] = 0;
    push(startId, 0, h(...start));
    let expanded = 0;
    while (open.length && expanded < maxExpansions) {
      const cur = pop();
      if (closed[cur.id] || cur.g !== g[cur.id]) continue;
      closed[cur.id] = 1;
      expanded++;
      if (cur.id === goalId) {
        const path = [];
        for (let i = goalId; i !== -1; i = prev[i]) path.push(belief.cell(i));
        path.reverse();
        path.expanded = expanded;
        return path;
      }
      const [x, y, z] = belief.cell(cur.id);
      for (const [dx, dy, dz] of DIRS) {
        const nx = x + dx, ny = y + dy, nz = z + dz;
        const n = belief.id(nx, ny, nz);
        if (n < 0 || closed[n] || belief.blocked[n] > 0) continue;
        const ng = cur.g + 1;
        if (ng >= g[n]) continue;
        g[n] = ng;
        prev[n] = cur.id;
        push(n, ng, ng + h(nx, ny, nz));
      }
    }
    return null;
  }

  // ---------------------------------------------------------------- flight

  function boxesOverlap(a, b) {
    return a.min[0] < b.max[0] && a.max[0] > b.min[0] && a.min[1] < b.max[1] && a.max[1] > b.min[1] && a.min[2] < b.max[2] && a.max[2] > b.min[2];
  }

  class SonarFlight {
    /**
     * opts: start, goal (cells), bounds (known outer box), speed (units/s), halfExtents (drone body,
     * for the collision check only), sensor options, seed.
     */
    constructor(opts) {
      // retryFor: seconds to hover and keep scanning when no path exists (false echoes clear in about a second)
      this.opts = Object.assign({ speed: 5, halfExtents: [3.25, 0.5, 3.25], retryFor: 3 }, opts);
      this.sensor = new SonarSensor(Object.assign({ seed: opts.seed || 12345 }, opts.sensor || {}));
      this.belief = new SonarBelief(opts.bounds, opts.belief || {});
      this.reset();
    }

    reset(position) {
      this.pos = (position || this.opts.start).slice();
      this.running = false;
      this.status = 'idle';
      this.path = null;
      this.pathIndex = 0;
      this.moving = false;
      this.retreating = false;
      this.replanRequested = false;
      this.moveTarget = null;
      this.replans = 0;
      this.collisions = 0;
      this.moves = 0;
      this.waiting = 0;
      this.lastHit = null;
      this.lastPulse = null;
      this.newCells = 0;
      this.belief.clear();
    }

    /** Takes off from the current cell: clears the belief, takes warm-up readings, plans. */
    start(boxes, warmupPulses = 3) {
      this.pos = this.pos.map(Math.round);
      this.belief.clear();
      for (let i = 0; i < warmupPulses; i++) this.onPulse(this.sensor.pulse(this.pos, boxes));
      this.replans = 0;
      this.collisions = 0;
      this.moves = 0;
      this.waiting = 0;
      this.running = true;
      this.moving = false;
      this.retreating = false;
      this.replanRequested = false;
      return this.plan();
    }

    /** Plans from the current cell. Without a path the drone hovers and retries (see step) before giving up. */
    plan() {
      const from = this.pos.map(Math.round);
      const path = planPath(this.belief, from, this.opts.goal);
      this.pathIndex = 0;
      if (!path) {
        this.path = null;
        this.status = 'hovering: no path yet';
        return false;
      }
      this.path = path;
      this.waiting = 0;
      this.status = 'flying';
      return true;
    }

    onPulse(pulse) {
      this.lastPulse = pulse;
      const newly = this.belief.integrate(pulse, this.sensor.settings.maxRange);
      this.newCells += newly.length;
      if (newly.length && this.running && this.remainingPathBlocked()) this.requestReplan();
      return newly;
    }

    remainingPathBlocked() {
      if (!this.path) return true;
      for (let i = this.pathIndex + 1; i < this.path.length; i++)
        if (this.belief.isBlocked(...this.path[i])) return true;
      return false;
    }

    requestReplan() {
      this.replanRequested = true;
      if (this.moving && !this.retreating && this.belief.isBlocked(...this.moveTarget)) {
        this.moveTarget = this.path[this.pathIndex];   // turn back to the cell just left
        this.retreating = true;
      }
    }

    /** Advances time by dt: sensor pulses, flight, then the collision check. Returns fired pulses. */
    step(dt, boxes) {
      const pulses = this.sensor.tick(dt, this.pos, boxes);
      for (const p of pulses) this.onPulse(p);
      if (!this.running) return pulses;

      // No path: hover in place, keep scanning, retry each pulse; give up after retryFor seconds.
      if (!this.path) {
        this.waiting += dt;
        if (pulses.length) { this.replans++; this.plan(); }
        if (!this.path && this.waiting >= this.opts.retryFor) { this.running = false; this.status = 'no path'; }
        return pulses;
      }

      const before = this.pos.slice();
      let budget = this.opts.speed * dt;
      while (budget > 0 && this.running) {
        if (!this.moving) {
          if (this.replanRequested) {
            this.replanRequested = false;
            this.replans++;
            if (!this.plan()) break;   // hover from here on (handled at the top of step)
          }
          if (this.pathIndex >= this.path.length - 1) { this.running = false; this.status = 'landed'; break; }
          this.moveTarget = this.path[this.pathIndex + 1];
          this.moving = true;
        }
        const d = [0, 1, 2].map(k => this.moveTarget[k] - this.pos[k]);
        const rem = Math.hypot(d[0], d[1], d[2]);
        const step = Math.min(budget, rem);
        for (let k = 0; k < 3; k++) this.pos[k] += rem > 0 ? d[k] / rem * step : 0;
        budget -= step;
        if (step >= rem - 1e-9) {
          this.pos = this.moveTarget.slice();
          this.moving = false;
          this.moves++;
          if (this.retreating) this.retreating = false; else this.pathIndex++;
        }
      }
      this.checkCollision(before, boxes);
      return pulses;
    }

    /** Safety check (not perception): the body swept from `from` to the current position vs. real boxes. */
    checkCollision(from, boxes) {
      const h = this.opts.halfExtents;
      const swept = {
        min: [0, 1, 2].map(k => Math.min(from[k], this.pos[k]) - h[k]),
        max: [0, 1, 2].map(k => Math.max(from[k], this.pos[k]) + h[k]),
      };
      for (const b of boxes) {
        if (!boxesOverlap(swept, b)) continue;
        this.collisions++;
        this.lastHit = b.name || 'obstacle';
        this.pos = from;          // stop where it last was clear
        this.running = false;
        this.status = 'collision';
        return true;
      }
      return false;
    }
  }

  const api = { SonarSensor, SonarBelief, SonarFlight, planPath, rayBox, SENSOR_DEFAULTS };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.SkyrlSonarCore = api;
})(typeof window !== 'undefined' ? window : globalThis);
