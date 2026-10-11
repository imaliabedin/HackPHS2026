
(() => {
  const core = window.SkyrlSonarCore;
  const world = window.skyrlWorld;
  if (!core || !world || !window.THREE) return;

  const START = [-38, 8, 35];
  const GOAL = [34, 1, 31];
  const BOUNDS = { min: [-46, 1, -46], max: [46, 18, 46] };
  const SPEED = 5;              
  const VISUAL_SPEED = 24;      
  const ECHO_PULSES = 5;       

  const params = new URLSearchParams(location.search);
  const seed = Number(params.get('seed')) || (Date.now() % 100000);
  const flight = new core.SonarFlight({ start: START, goal: GOAL, bounds: BOUNDS, speed: SPEED, seed });
  const maxRange = flight.sensor.settings.maxRange;
  const samplesPerPulse = flight.sensor.queriesPerPulse;

  const $ = id => document.getElementById(id);
  const view = { rays: true, rings: true, belief: true, follow: params.get('follow') === '1' };
  const recent = [];           

  

  const tmpBox = new THREE.Box3();
  function boxOf(object, name) {
    tmpBox.setFromObject(object);
    return { name, min: tmpBox.min.toArray(), max: tmpBox.max.toArray() };
  }
  
  const blockBoxes = world.blocks.children.filter(o => o.isMesh).map((m, i) => boxOf(m, `block ${i + 1}`));
  function obstacleBoxes() {
    const boxes = blockBoxes.slice();
    const placed = world.scene.getObjectByName('User placed Unity props');
    if (placed) {
      placed.updateMatrixWorld(true);
      for (const prop of placed.children)
        prop.traverse(o => {
          if (o.isMesh && !o.userData.selectionRing) boxes.push(boxOf(o, prop.userData.catalogName || 'object'));
        });
    }
    return boxes;
  }

  

  const overlay = new THREE.Group();
  overlay.name = 'Sonar overlay';
  world.scene.add(overlay);

 
  const circle = new THREE.BufferGeometry().setFromPoints(
    Array.from({ length: 65 }, (_, i) => new THREE.Vector3(Math.cos(i / 64 * Math.PI * 2), 0, Math.sin(i / 64 * Math.PI * 2))));
  const rings = Array.from({ length: 8 }, () => {
    const g = new THREE.Group();
    const mat = new THREE.LineBasicMaterial({ color: 0x55d8ff, transparent: true, opacity: 0.5, depthWrite: false });
    for (const rot of [[0, 0, 0], [Math.PI / 2, 0, 0], [0, 0, Math.PI / 2]]) {
      const l = new THREE.Line(circle, mat);
      l.rotation.set(...rot);
      g.add(l);
    }
    g.visible = false;
    g.userData.mat = mat;
    overlay.add(g);
    return g;
  });
  let nextRing = 0;

 
  const rayPos = new Float32Array(samplesPerPulse * 6), rayCol = new Float32Array(samplesPerPulse * 6);
  const rayGeo = new THREE.BufferGeometry();
  rayGeo.setAttribute('position', new THREE.BufferAttribute(rayPos, 3));
  rayGeo.setAttribute('color', new THREE.BufferAttribute(rayCol, 3));
  const rays = new THREE.LineSegments(rayGeo, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.45, depthWrite: false }));
  rays.frustumCulled = false;
  overlay.add(rays);

  
  const echoCount = ECHO_PULSES * samplesPerPulse;
  const echoPos = new Float32Array(echoCount * 3), echoCol = new Float32Array(echoCount * 3);
  const echoGeo = new THREE.BufferGeometry();
  echoGeo.setAttribute('position', new THREE.BufferAttribute(echoPos, 3));
  echoGeo.setAttribute('color', new THREE.BufferAttribute(echoCol, 3));
  const echoes = new THREE.Points(echoGeo, new THREE.PointsMaterial({ size: 0.45, vertexColors: true, transparent: true, blending: THREE.AdditiveBlending, depthWrite: false }));
  echoes.frustumCulled = false;
  overlay.add(echoes);
  const backPos = new Float32Array(echoCount * 6);
  const backGeo = new THREE.BufferGeometry();
  backGeo.setAttribute('position', new THREE.BufferAttribute(backPos, 3));
  const back = new THREE.LineSegments(backGeo, new THREE.LineBasicMaterial({ color: 0xffef6a, transparent: true, opacity: 0.9, depthWrite: false }));
  back.frustumCulled = false;
  overlay.add(back);

  
  const MAX_BELIEF = 6000;
  const beliefMesh = new THREE.InstancedMesh(new THREE.BoxGeometry(0.9, 0.9, 0.9),
    new THREE.MeshBasicMaterial({ color: 0xff3b3b, wireframe: true, transparent: true, opacity: 0.85 }), MAX_BELIEF);
  beliefMesh.count = 0;
  beliefMesh.frustumCulled = false;
  overlay.add(beliefMesh);

  
  const routePos = new Float32Array(600 * 3);
  const routeGeo = new THREE.BufferGeometry();
  routeGeo.setAttribute('position', new THREE.BufferAttribute(routePos, 3));
  const route = new THREE.Line(routeGeo, new THREE.LineBasicMaterial({ color: 0xffd84a }));
  route.frustumCulled = false;
  overlay.add(route);

  const C_OBST = [1, 0.55, 0.1], C_GROUND = [0.55, 0.42, 0.3], C_NONE = [0.35, 0.4, 0.5];

  function onPulse(pulse) {
    recent.push(pulse);
    if (recent.length > ECHO_PULSES) recent.shift();
    const g = rings[nextRing++ % rings.length];
    g.position.fromArray(pulse.origin);
    g.userData.t0 = pulse.time;
    g.visible = true;

    pulse.samples.forEach((m, i) => {
      const end = m.hasReturn ? m.hit : m.origin.map((v, k) => v + m.dir[k] * maxRange);
      const c = !m.hasReturn ? C_NONE : flight.belief.isGroundEcho(m) ? C_GROUND : C_OBST;
      rayPos.set(m.origin, i * 6); rayPos.set(end, i * 6 + 3);
      rayCol.set(c, i * 6); rayCol.set(c, i * 6 + 3);
    });
    rayGeo.attributes.position.needsUpdate = rayGeo.attributes.color.needsUpdate = true;

    beliefMesh.count = 0;
    const mtx = new THREE.Matrix4();
    for (const id of flight.belief.occupiedList) {
      if (beliefMesh.count >= MAX_BELIEF) break;
      mtx.setPosition(...flight.belief.cell(id));
      beliefMesh.setMatrixAt(beliefMesh.count++, mtx);
    }
    beliefMesh.instanceMatrix.needsUpdate = true;
    updateRoute();
  }

  function updateRoute() {
    const path = flight.path || [];
    const pts = [flight.pos].concat(path.slice(flight.pathIndex + 1)).slice(0, 600);
    pts.forEach((p, i) => routePos.set(p, i * 3));
    routeGeo.setDrawRange(0, flight.running ? pts.length : 0);
    routeGeo.attributes.position.needsUpdate = true;
  }

  function drawWaves() {
    const now = flight.sensor.clock;
    for (const g of rings) {
      if (!g.visible) continue;
      const r = (now - g.userData.t0) * VISUAL_SPEED;
      if (r > maxRange) { g.visible = false; continue; }
      g.scale.setScalar(Math.max(r, 0.01));
      g.userData.mat.opacity = 0.55 * (1 - r / maxRange);
      g.visible = view.rings;
    }
    let n = 0;
    for (const p of recent) {
      const travelled = (now - p.time) * VISUAL_SPEED;
      const fade = Math.max(0, 1 - (now - p.time) / (ECHO_PULSES / flight.sensor.settings.pulseRate));
      for (const m of p.samples) {
        const i = n++;
        if (!m.hasReturn || travelled < m.range) { echoCol.fill(0, i * 3, i * 3 + 3); backPos.fill(0, i * 6, i * 6 + 6); continue; }
        echoPos.set(m.hit, i * 3);
        const c = flight.belief.isGroundEcho(m) ? C_GROUND : C_OBST;
        echoCol.set(c.map(v => v * fade), i * 3);
        const backDist = travelled - m.range;
        if (c === C_OBST && backDist < m.range) {
          const head = m.hit.map((v, k) => v - m.dir[k] * backDist);
          backPos.set(head, i * 6);
          backPos.set(head.map((v, k) => v - m.dir[k] * Math.min(1.2, m.range - backDist)), i * 6 + 3);
        } else backPos.fill(0, i * 6, i * 6 + 6);
      }
    }
    echoGeo.setDrawRange(0, n);
    backGeo.setDrawRange(0, n * 2);
    echoGeo.attributes.position.needsUpdate = echoGeo.attributes.color.needsUpdate = true;
    backGeo.attributes.position.needsUpdate = true;
  }

  

  const STATUS = {
    idle: '● IDLE · SONAR SCANNING', flying: '● FLYING · SONAR', landed: '● LANDED', collision: '● COLLISION · STOPPED',
    'no path': '● NO PATH', 'hovering: no path yet': '● HOVERING · RESCANNING',
  };

  function hud(dt) {
    const last = flight.lastPulse;
    let obst = 0, ground = 0, none = 0, nearest = Infinity;
    if (last) for (const m of last.samples) {
      if (!m.hasReturn) none++;
      else if (flight.belief.isGroundEcho(m)) ground++;
      else { obst++; nearest = Math.min(nearest, m.range); }
    }
    const set = (id, v) => { const e = $(id); if (e) e.textContent = v; };
    set('sonarEchoes', `${obst} obstacle · ${ground} ground · ${none} none`);
    set('sonarNearest', nearest < Infinity ? `${nearest.toFixed(1)} u` : '—');
    set('sonarBelief', String(flight.belief.occupiedList.size));
    set('sonarReplans', String(flight.replans));
    set('sonarCollisions', String(flight.collisions));
    set('sonarPulseMs', `${last ? last.ms.toFixed(2) : '—'} ms`);
    set('replans', String(flight.replans));
    set('collisions', String(flight.collisions));
    set('agentStatus', STATUS[flight.status] || `● ${flight.status.toUpperCase()}`);
    const banner = $('sonarBanner');
    if (banner) {
      banner.textContent = flight.status === 'landed' ? 'LANDED — reached Landing Zone A using sonar only'
        : flight.status === 'collision' ? `COLLISION with ${flight.lastHit} — drone stopped`
          : flight.status === 'no path' ? 'NO PATH — every route looks blocked to the sonar' : '';
      banner.className = flight.status;
    }
    const p = flight.pos;
    set('coords', `X ${p[0].toFixed(1)} / Y ${p[1].toFixed(1)} / Z ${p[2].toFixed(1)}`);
    const alt = $('altitude'); if (alt) alt.innerHTML = `${(p[1] - world.groundY).toFixed(1)} <i>m</i>`;
    const vel = $('velocity'); if (vel) vel.innerHTML = `${(flight.running && flight.path ? SPEED : 0).toFixed(1)} <i>m/s</i>`;
    if (flight.moving && flight.moveTarget) {
      const h = (Math.atan2(flight.moveTarget[0] - p[0], flight.moveTarget[2] - p[2]) * 180 / Math.PI + 360) % 360;
      if (Math.hypot(flight.moveTarget[0] - p[0], flight.moveTarget[2] - p[2]) > 1e-3) set('heading', `${String(h | 0).padStart(3, '0')}°`);
    }
    const run = $('run');
    if (run) run.textContent = flight.running ? 'Ⅱ  Stop sonar flight' : '▶  Run sonar flight';
  }

  

  let frame = 0;
  function update(dt) {
    frame++;
    const boxes = obstacleBoxes();
    const fired = flight.step(dt, boxes);
    for (const p of fired) onPulse(p);
    if (flight.running) updateRoute();

    const d = world.drone;
    d.position.fromArray(flight.pos);
    if (!flight.running && flight.status === 'idle') d.position.y += Math.sin(frame * 0.025) * 0.35;
    if (flight.moving && flight.moveTarget) {
      const dx = flight.moveTarget[0] - flight.pos[0], dz = flight.moveTarget[2] - flight.pos[2];
      if (Math.hypot(dx, dz) > 1e-3) {
        const turn = Math.atan2(Math.sin(Math.atan2(dx, dz) - d.rotation.y), Math.cos(Math.atan2(dx, dz) - d.rotation.y));
        d.rotation.y += turn * 0.15;   // the short way round
      }
    }
    if (view.follow && world.controls && !world.controls.drag) {
      world.controls.target.lerp(d.position, 0.08);
      world.cameraUpdate();
    }
    drawWaves();
    hud(dt);
  }

  function start() {
    flight.reset(START);
    recent.length = 0;
    const ok = flight.start(obstacleBoxes());   // includes the warm-up pulses at the start position
    if (flight.lastPulse) onPulse(flight.lastPulse);
    return ok;
  }

  function stop() {
    flight.running = false;
    flight.status = 'idle';
    routeGeo.setDrawRange(0, 0);
  }

  function reset() {
    flight.reset(START);
    recent.length = 0;
    beliefMesh.count = 0;
    routeGeo.setDrawRange(0, 0);
    world.drone.rotation.set(0, 0, 0);
  }

  function toggle(name, buttonId) {
    view[name] = !view[name];
    const b = $(buttonId);
    if (b) b.classList.toggle('active', view[name]);
    if (name === 'rays') rays.visible = view.rays;
    if (name === 'belief') beliefMesh.visible = view.belief;
    if (name === 'rings' && !view.rings) for (const g of rings) g.visible = false;
  }

  [['rays', 'sonarRays'], ['rings', 'sonarRings'], ['belief', 'sonarBeliefBtn'], ['follow', 'sonarFollow']].forEach(([n, id]) => {
    const b = $(id);
    if (!b) return;
    b.classList.toggle('active', view[n]);
    b.addEventListener('click', () => toggle(n, id));
  });
  const q = $('sonarQueries');
  if (q) q.textContent = `${flight.sensor.beams.length} beams × ${flight.sensor.samplesPerBeam} = ${samplesPerPulse} / pulse · ${flight.sensor.settings.pulseRate} Hz · ${maxRange} u`;

  window.SkyrlSonar = {
    flight, update, start, stop, reset,
    get running() { return flight.running; },
    toggleRun() { return flight.running ? (stop(), false) : start(); },
  };

  if (params.get('autorun') === '1') setTimeout(start, Number(params.get('delay')) || 1500);
})();
