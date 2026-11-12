/* Pass-By Release Simulator viewer.
 * Two modes:
 *   static -- window.SIM_STATIC holds pre-computed runs + a Monte Carlo batch
 *   live   -- served by `python -m simcore serve`; runs are computed on demand via /api/*
 */
(() => {
  "use strict";
  const STATES = ["IDLE", "ARMED", "TRACKING", "COMMITTED", "RELEASED", "LANDED", "ABORTED"];
  const STATIC = window.SIM_STATIC || null;
  const $ = (id) => document.getElementById(id);
  const KMH = 3.6;

  let run = null;          // current run JSON
  let presetId = null;
  let tCur = 0;
  let playing = false;
  let zoom = "range";
  let lastTs = null;
  let mcData = null;
  let C = {};               // theme colours

  // ------------------------------------------------------------------ utils
  function readColors() {
    const cs = getComputedStyle(document.documentElement);
    for (const k of ["bg", "panel", "ink", "muted", "rule", "grid", "road", "road-line", "vehicle", "estimate",
      "payload", "zone", "keepout", "sensor", "ok", "warn", "bad", "chip"]) {
      C[k] = cs.getPropertyValue("--" + k).trim();
    }
    C.mono = cs.getPropertyValue("--font-mono").trim() || "monospace";
    C.body = cs.getPropertyValue("--font-body").trim() || "sans-serif";
  }

  function setup(canvas) {
    const dpr = window.devicePixelRatio || 1;
    const r = canvas.getBoundingClientRect();
    const w = Math.max(10, Math.round(r.width)), h = Math.max(10, Math.round(r.height));
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    return { ctx, w, h };
  }

  function idxAt(t) {
    const ts = run.frames.t;
    let lo = 0, hi = ts.length - 1;
    if (t <= ts[0]) return 0;
    if (t >= ts[hi]) return hi;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (ts[mid] <= t) lo = mid; else hi = mid;
    }
    return lo;
  }
  const F = (key, i) => { const v = run.frames[key][i]; return v === null || v === undefined ? NaN : v; };
  const fin = (v) => typeof v === "number" && isFinite(v);
  const fmt = (v, d = 2, unit = "") => (fin(v) ? v.toFixed(d) + (unit ? " " + unit : "") : "—");
  const tEnd = () => run.frames.t[run.frames.t.length - 1];

  function niceTicks(lo, hi, n = 5) {
    const span = hi - lo || 1;
    const step0 = span / n;
    const mag = Math.pow(10, Math.floor(Math.log10(step0)));
    const err = step0 / mag;
    const step = (err >= 7.5 ? 10 : err >= 3.5 ? 5 : err >= 1.5 ? 2 : 1) * mag;
    const out = [];
    for (let v = Math.ceil(lo / step) * step; v <= hi + step * 1e-6; v += step) out.push(+v.toFixed(10));
    return out;
  }

  function eventsOf(kind) { return run.events.filter((e) => e.kind === kind); }
  function firstEvent(kind) { return run.events.find((e) => e.kind === kind); }

  // -------------------------------------------------------------- plan view
  function planView(w, h) {
    const sc = run.scene;
    const box = zoom === "range"
      ? { x0: -62, x1: 14, y0: -3, y1: sc.lane_y + 4 }
      : { x0: -7, x1: 11, y0: -1.2, y1: sc.lane_y + 2.6 };
    const s = Math.min(w / (box.x1 - box.x0), h / (box.y1 - box.y0));
    const cx = (box.x0 + box.x1) / 2, cy = (box.y0 + box.y1) / 2;
    return {
      s,
      X: (x) => w / 2 + (x - cx) * s,
      Y: (y) => h / 2 - (y - cy) * s,
      xmin: cx - w / 2 / s, xmax: cx + w / 2 / s, ymin: cy - h / 2 / s, ymax: cy + h / 2 / s,
    };
  }

  function drawPlan() {
    const { ctx, w, h } = setup($("plan"));
    const sc = run.scene, cfg = run.config;
    const v = planView(w, h);
    const i = idxAt(tCur);
    const X = v.X, Y = v.Y;
    ctx.fillStyle = C.panel; ctx.fillRect(0, 0, w, h);

    // grid
    const gstep = zoom === "range" ? 10 : 2;
    ctx.strokeStyle = C.grid; ctx.lineWidth = 1; ctx.font = `11px ${C.mono}`; ctx.fillStyle = C.muted;
    for (let gx = Math.ceil(v.xmin / gstep) * gstep; gx <= v.xmax; gx += gstep) {
      ctx.beginPath(); ctx.moveTo(X(gx), 0); ctx.lineTo(X(gx), h); ctx.stroke();
      ctx.fillText(`${gx} m`, X(gx) + 3, h - 5);
    }
    for (let gy = Math.ceil(v.ymin / gstep) * gstep; gy <= v.ymax; gy += gstep) {
      ctx.beginPath(); ctx.moveTo(0, Y(gy)); ctx.lineTo(w, Y(gy)); ctx.stroke();
    }

    // road
    const half = 1.75;
    ctx.fillStyle = C.road;
    ctx.fillRect(0, Y(sc.lane_y + half + 0.6), w, (2 * half + 1.2) * v.s);
    ctx.strokeStyle = C["road-line"]; ctx.lineWidth = 2; ctx.setLineDash([12, 10]);
    for (const yy of [sc.lane_y - half, sc.lane_y + half]) {
      ctx.beginPath(); ctx.moveTo(0, Y(yy)); ctx.lineTo(w, Y(yy)); ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.fillStyle = C.muted; ctx.font = `11px ${C.body}`;
    ctx.fillText("lane  →  direction of travel", 8, Y(sc.lane_y + half + 0.6) + 13);

    // keep-out corridor between the zone and the vehicle's near side
    const nearSide = sc.lane_y - sc.vehicle_width / 2;
    const clear = cfg.safety.lane_clearance_min;
    ctx.fillStyle = C.keepout;
    ctx.fillRect(0, Y(nearSide), w, clear * v.s);
    ctx.fillStyle = C.vehicle; ctx.globalAlpha = 0.8; ctx.font = `11px ${C.body}`;
    ctx.fillText(`keep-out ≥ ${clear} m`, w - 120, Y(nearSide - clear / 2) + 4);
    ctx.globalAlpha = 1;

    // zone
    const z = sc.zone;
    ctx.fillStyle = C.zone; ctx.strokeStyle = C.payload; ctx.lineWidth = 1.5;
    ctx.fillRect(X(z.x[0]), Y(z.y[1]), (z.x[1] - z.x[0]) * v.s, (z.y[1] - z.y[0]) * v.s);
    ctx.strokeRect(X(z.x[0]), Y(z.y[1]), (z.x[1] - z.x[0]) * v.s, (z.y[1] - z.y[0]) * v.s);
    ctx.fillStyle = C.payload; ctx.font = `600 11px ${C.body}`;
    ctx.fillText("TEST ZONE", X(z.x[0]) + 3, Y(z.y[1]) - 4);

    // gantry travel
    const g = sc.gantry;
    ctx.strokeStyle = C.payload; ctx.setLineDash([4, 3]); ctx.lineWidth = 1;
    ctx.strokeRect(X(g.x[0]), Y(g.y[1]), (g.x[1] - g.x[0]) * v.s, (g.y[1] - g.y[0]) * v.s);
    ctx.setLineDash([]);

    // radar wedge
    const rd = sc.radar;
    const a0 = (rd.boresight_deg - rd.fov_deg / 2) * Math.PI / 180, a1 = (rd.boresight_deg + rd.fov_deg / 2) * Math.PI / 180;
    ctx.beginPath(); ctx.moveTo(X(rd.pos[0]), Y(rd.pos[1]));
    for (let k = 0; k <= 24; k++) {
      const a = a0 + (a1 - a0) * k / 24;
      ctx.lineTo(X(rd.pos[0] + rd.max_range * Math.sin(a)), Y(rd.pos[1] + rd.max_range * Math.cos(a)));
    }
    ctx.closePath();
    ctx.fillStyle = C.sensor; ctx.globalAlpha = 0.07; ctx.fill(); ctx.globalAlpha = 0.5;
    ctx.strokeStyle = C.sensor; ctx.lineWidth = 1; ctx.stroke(); ctx.globalAlpha = 1;

    // vehicle truth
    const vx = F("veh_x", i), L = sc.vehicle_length, W = sc.vehicle_width;

    // lidar beams
    const ld = sc.lidar;
    ld.gates_x.forEach((gx, k) => {
      const blocked = vx - L / 2 <= gx && gx <= vx + L / 2;
      const yEnd = blocked ? nearSide : Math.min(ld.mount_y + ld.max_range, v.ymax);
      ctx.strokeStyle = C.sensor; ctx.lineWidth = blocked ? 3 : 1; ctx.globalAlpha = blocked ? 1 : 0.6;
      ctx.beginPath(); ctx.moveTo(X(gx), Y(ld.mount_y)); ctx.lineTo(X(gx), Y(yEnd)); ctx.stroke();
      ctx.globalAlpha = 1;
      ctx.fillStyle = C.sensor; ctx.fillRect(X(gx) - 4, Y(ld.mount_y) - 4, 8, 8);
      if (zoom === "range") { ctx.font = `11px ${C.mono}`; ctx.fillText(`L${k}`, X(gx) - 6, Y(ld.mount_y) + 16); }
    });

    // microphone + trigger rings
    const trig = F("trigger", i) > 0;
    ctx.fillStyle = C.sensor;
    ctx.beginPath(); ctx.arc(X(sc.mic[0]), Y(sc.mic[1]), 4, 0, 2 * Math.PI); ctx.fill();
    if (trig) {
      ctx.strokeStyle = C.sensor; ctx.lineWidth = 1.2;
      const phase = (tCur * 2) % 1;
      for (let k = 0; k < 3; k++) {
        const r = ((k + phase) / 3) * 3.2 * v.s;
        ctx.globalAlpha = 1 - (k + phase) / 3;
        ctx.beginPath(); ctx.arc(X(sc.mic[0]), Y(sc.mic[1]), r, 0, 2 * Math.PI); ctx.stroke();
      }
      ctx.globalAlpha = 1;
    }
    ctx.font = `11px ${C.body}`; ctx.fillStyle = C.muted;
    ctx.fillText("mic + radar", X(sc.mic[0]) + 6, Y(sc.mic[1]) + 14);

    // aim + 3-sigma landing ellipse
    const aim = F("aim_x", i);
    const k3 = cfg.safety.sigma_k;
    if (fin(aim)) {
      ctx.strokeStyle = C.payload; ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.ellipse(X(aim), Y(sc.y_aim), Math.max(2, k3 * sc.launch.sigma_x * v.s), Math.max(2, k3 * sc.launch.sigma_y * v.s), 0, 0, 2 * Math.PI);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(X(aim) - 6, Y(sc.y_aim)); ctx.lineTo(X(aim) + 6, Y(sc.y_aim));
      ctx.moveTo(X(aim), Y(sc.y_aim) - 6); ctx.lineTo(X(aim), Y(sc.y_aim) + 6);
      ctx.stroke();
      // line of sight from aim point to the lane: where the vehicle centre should be at landing
      ctx.setLineDash([2, 4]); ctx.globalAlpha = 0.6;
      ctx.beginPath(); ctx.moveTo(X(aim), Y(sc.y_aim)); ctx.lineTo(X(aim), Y(sc.lane_y)); ctx.stroke();
      ctx.setLineDash([]); ctx.globalAlpha = 1;
    }

    // estimate & prediction
    const ex = F("est_x", i), ey = F("est_y", i), sx = F("sig_x", i);
    if (fin(ex)) {
      ctx.strokeStyle = C.estimate; ctx.lineWidth = 1.5;
      ctx.strokeRect(X(ex - L / 2), Y(ey + W / 2), L * v.s, W * v.s);
      ctx.beginPath(); ctx.moveTo(X(ex - 2 * sx), Y(ey)); ctx.lineTo(X(ex + 2 * sx), Y(ey)); ctx.stroke();
      const px = F("pred_x", i), ps = F("pred_sig", i);
      if (fin(px)) {
        ctx.setLineDash([5, 4]);
        ctx.strokeRect(X(px - L / 2), Y(ey + W / 2), L * v.s, W * v.s);
        ctx.setLineDash([]);
        ctx.beginPath(); ctx.moveTo(X(px - 2 * ps), Y(ey + W / 2 + 0.35)); ctx.lineTo(X(px + 2 * ps), Y(ey + W / 2 + 0.35)); ctx.stroke();
        ctx.fillStyle = C.estimate; ctx.font = `11px ${C.mono}`;
        ctx.fillText(`t+${sc.horizon_s}s`, X(px - L / 2), Y(ey + W / 2 + 0.6));
      }
    }

    // vehicle (drawn over the estimate outline so the truth is always visible)
    ctx.fillStyle = C.vehicle; ctx.globalAlpha = 0.9;
    ctx.fillRect(X(vx - L / 2), Y(sc.lane_y + W / 2), L * v.s, W * v.s);
    ctx.globalAlpha = 1;

    // gantry carriage
    const gx = F("gx", i), gy = F("gy", i), tx = F("gtx", i), ty = F("gty", i);
    ctx.strokeStyle = C.payload; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(X(tx) - 4, Y(ty) - 4); ctx.lineTo(X(tx) + 4, Y(ty) + 4);
    ctx.moveTo(X(tx) + 4, Y(ty) - 4); ctx.lineTo(X(tx) - 4, Y(ty) + 4); ctx.stroke();
    ctx.fillStyle = C.payload;
    const cs = Math.max(6, 0.3 * v.s);
    ctx.fillRect(X(gx) - cs / 2, Y(gy) - cs / 2, cs, cs);

    // payload ground track and landing
    if (run.flight) {
      const fl = run.flight;
      ctx.strokeStyle = C.payload; ctx.lineWidth = 2;
      ctx.beginPath();
      let started = false, last = null;
      for (let k = 0; k < fl.t.length; k++) {
        if (fl.t[k] > tCur) break;
        const p = [X(fl.x[k]), Y(fl.y[k])];
        if (!started) { ctx.moveTo(p[0], p[1]); started = true; } else ctx.lineTo(p[0], p[1]);
        last = k;
      }
      ctx.stroke();
      if (last !== null) {
        const landed = tCur >= fl.t[fl.t.length - 1];
        const xx = X(fl.x[last]), yy = Y(fl.y[last]);
        if (landed) {
          ctx.lineWidth = 2.5;
          ctx.beginPath(); ctx.moveTo(xx - 6, yy - 6); ctx.lineTo(xx + 6, yy + 6); ctx.moveTo(xx + 6, yy - 6); ctx.lineTo(xx - 6, yy + 6); ctx.stroke();
        } else {
          const r = 3 + 3 * (fl.z[last] / 2.5);
          ctx.fillStyle = C.payload; ctx.beginPath(); ctx.arc(xx, yy, r, 0, 2 * Math.PI); ctx.fill();
        }
      }
    }

    // header text
    ctx.fillStyle = C.ink; ctx.font = `500 12px ${C.mono}`;
    ctx.fillText(`t = ${tCur.toFixed(2)} s   ${STATES[F("state", i)] || ""}`, 8, 16);
  }

  // -------------------------------------------------------------- side view
  function drawSide() {
    const { ctx, w, h } = setup($("side"));
    const sc = run.scene, i = idxAt(tCur);
    ctx.fillStyle = C.panel; ctx.fillRect(0, 0, w, h);
    const y0 = -0.8, y1 = sc.lane_y + 2, z0 = -0.3, z1 = 3.2;
    const s = Math.min((w - 20) / (y1 - y0), (h - 24) / (z1 - z0));
    const X = (y) => 10 + (y - y0) * s, Z = (z) => h - 18 - (z - z0) * s;
    // ground
    ctx.strokeStyle = C.muted; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0, Z(0)); ctx.lineTo(w, Z(0)); ctx.stroke();
    ctx.fillStyle = C.muted; ctx.font = `11px ${C.mono}`;
    for (const t of niceTicks(0, y1, 6)) { ctx.fillText(`${t}`, X(t) - 4, h - 4); }
    // zone + keep-out
    const z = sc.zone;
    ctx.fillStyle = C.zone; ctx.fillRect(X(z.y[0]), Z(0) - 6, (z.y[1] - z.y[0]) * s, 6);
    ctx.fillStyle = C.payload; ctx.fillRect(X(z.y[0]), Z(0), (z.y[1] - z.y[0]) * s, 3);
    const near = sc.lane_y - sc.vehicle_width / 2;
    ctx.fillStyle = C.keepout; ctx.fillRect(X(near - run.config.safety.lane_clearance_min), Z(z1), run.config.safety.lane_clearance_min * s, Z(0) - Z(z1));
    // vehicle profile
    ctx.fillStyle = C.vehicle; ctx.globalAlpha = 0.85;
    ctx.fillRect(X(near), Z(1.5), sc.vehicle_width * s, 1.5 * s);
    ctx.globalAlpha = 1;
    // carriage
    const gy = F("gy", i);
    ctx.fillStyle = C.payload;
    ctx.fillRect(X(gy) - 4, Z(run.config.payload.height) - 4, 8, 8);
    ctx.strokeStyle = C.payload; ctx.beginPath(); ctx.moveTo(X(gy), Z(0)); ctx.lineTo(X(gy), Z(run.config.payload.height)); ctx.stroke();
    // flight
    const note = $("flight-note");
    if (run.flight) {
      const fl = run.flight;
      ctx.setLineDash([3, 4]); ctx.strokeStyle = C.payload; ctx.globalAlpha = 0.45; ctx.lineWidth = 1;
      ctx.beginPath(); fl.y.forEach((yy, k) => (k ? ctx.lineTo(X(yy), Z(fl.z[k])) : ctx.moveTo(X(yy), Z(fl.z[k])))); ctx.stroke();
      ctx.setLineDash([]); ctx.globalAlpha = 1; ctx.lineWidth = 2;
      ctx.beginPath();
      let last = null;
      for (let k = 0; k < fl.t.length && fl.t[k] <= tCur; k++) { k ? ctx.lineTo(X(fl.y[k]), Z(fl.z[k])) : ctx.moveTo(X(fl.y[k]), Z(fl.z[k])); last = k; }
      ctx.stroke();
      if (last !== null) { ctx.fillStyle = C.payload; ctx.beginPath(); ctx.arc(X(fl.y[last]), Z(fl.z[last]), 4, 0, 2 * Math.PI); ctx.fill(); }
      const tof = fl.t[fl.t.length - 1] - fl.t[0];
      const apex = Math.max(...fl.z);
      note.textContent = `flight ${tof.toFixed(2)} s · apex ${apex.toFixed(2)} m · drag + wind`;
    } else {
      ctx.fillStyle = C.muted; ctx.font = `12px ${C.body}`;
      ctx.fillText(run.summary.status === "ABORTED" ? "No release: the system stood down." : "Waiting for release.", X(0.5), Z(2.6));
      note.textContent = "";
    }
    ctx.fillStyle = C.muted; ctx.font = `11px ${C.body}`;
    ctx.fillText("rig", X(0) - 6, Z(0) + 14 > h ? h - 4 : Z(0) - 4);
  }

  // ------------------------------------------------------------ time charts
  const charts = [];
  function chartDefs() {
    const fr = run.frames;
    const t = fr.t;
    const m = run.measurements;
    const finalPlan = eventsOf("release_command").slice(-1)[0];
    const tLandFinal = finalPlan ? finalPlan.t_land : null;
    return [
      {
        id: "c-sound", title: "Sound level", unit: "dB",
        note: "Microphone level with the adaptive noise floor and trigger threshold. Shaded: trigger active.",
        shade: fr.trigger,
        series: [
          { x: t, y: fr.spl, color: C.sensor, width: 1.2, label: "measured" },
          { x: t, y: fr.spl_floor, color: C.muted, width: 1, label: "floor" },
          { x: t, y: fr.spl_threshold, color: C.vehicle, width: 1, dash: [5, 4], label: "threshold" },
        ],
      },
      {
        id: "c-speed", title: "Speed", unit: "km/h",
        note: "Dots are raw radar radial speeds: lower than the true speed because of the viewing angle (cosine effect). The tracker removes it.",
        series: [
          { x: m.radar.map((r) => r[0]), y: m.radar.map((r) => Math.abs(r[2]) * KMH), color: C.sensor, points: true, label: "radar |v_r|" },
          { x: t, y: fr.veh_v.map((v) => v * KMH), color: C.vehicle, width: 2, label: "true" },
          { x: t, lo: fr.est_v.map((v, k) => fin(v) ? (v - 2 * fr.sig_v[k]) * KMH : null), hi: fr.est_v.map((v, k) => fin(v) ? (v + 2 * fr.sig_v[k]) * KMH : null), color: C.estimate, band: true },
          { x: t, y: fr.est_v.map((v) => fin(v) ? v * KMH : null), color: C.estimate, width: 1.5, label: "estimate ± 2σ" },
        ],
      },
      {
        id: "c-lat", title: "Lane offset (detection distance)", unit: "m",
        note: "Distance from the rig to the lane centre. Set by the light-gate ranges, refined by radar geometry and microphone bearings.",
        series: [
          { x: t, y: t.map(() => run.scene.lane_y), color: C.vehicle, width: 2, label: "true" },
          { x: t, lo: fr.est_y.map((v, k) => fin(v) ? v - 2 * fr.sig_y[k] : null), hi: fr.est_y.map((v, k) => fin(v) ? v + 2 * fr.sig_y[k] : null), color: C.estimate, band: true },
          { x: t, y: fr.est_y, color: C.estimate, width: 1.5, label: "estimate ± 2σ" },
        ],
      },
      {
        id: "c-pred", title: `Prediction error at t + ${run.scene.horizon_s} s`, unit: "m",
        note: "Predicted minus actual future position of the vehicle centre, with the filter's own ±2σ.",
        zero: true,
        series: [
          { x: t, lo: fr.pred_x.map((v, k) => fin(v) ? v - fr.true_pred_x[k] - 2 * fr.pred_sig[k] : null), hi: fr.pred_x.map((v, k) => fin(v) ? v - fr.true_pred_x[k] + 2 * fr.pred_sig[k] : null), color: C.estimate, band: true },
          { x: t, y: fr.pred_x.map((v, k) => fin(v) ? v - fr.true_pred_x[k] : null), color: C.estimate, width: 1.5, label: "error ± 2σ" },
        ],
      },
      {
        id: "c-gantry", title: "Gantry axes", unit: "m",
        note: "Carriage position (solid) following the planner's release-point target (dashed).",
        series: [
          { x: t, y: fr.gtx, color: C.payload, width: 1, dash: [5, 4] },
          { x: t, y: fr.gx, color: C.payload, width: 2, label: "X (along road)" },
          { x: t, y: fr.gty, color: C.sensor, width: 1, dash: [5, 4] },
          { x: t, y: fr.gy, color: C.sensor, width: 2, label: "Y (across)" },
        ],
      },
      {
        id: "c-timing", title: "Predicted landing time, revision", unit: "ms",
        note: "How the predicted landing time moved as data arrived, relative to the value used at the release command; band is ±2σ_t.",
        zero: true,
        series: tLandFinal === null ? [] : [
          { x: t, lo: fr.t_land.map((v, k) => fin(v) ? (v - tLandFinal - 2 * fr.sigma_t[k]) * 1000 : null), hi: fr.t_land.map((v, k) => fin(v) ? (v - tLandFinal + 2 * fr.sigma_t[k]) * 1000 : null), color: C.estimate, band: true },
          { x: t, y: fr.t_land.map((v) => fin(v) ? (v - tLandFinal) * 1000 : null), color: C.estimate, width: 1.5, label: "t_land − final" },
        ],
      },
    ];
  }

  function buildCharts() {
    const host = $("charts");
    host.innerHTML = "";
    charts.length = 0;
    for (const def of chartDefs()) {
      const sec = document.createElement("section");
      sec.className = "panel chart";
      const legend = def.series.filter((s) => s.label).map((s) =>
        `<span><i style="background:${s.color}${s.points ? ";height:6px;width:6px;border-radius:50%" : ""}"></i>${s.label}</span>`).join("");
      sec.innerHTML = `<div class="panel-head"><h3>${def.title}</h3><span class="muted mono" style="font-size:12px">${def.unit}</span></div>
        <canvas role="img" aria-label="${def.title} chart"></canvas><div class="legend">${legend}</div><p class="note">${def.note}</p>`;
      host.appendChild(sec);
      const canvas = sec.querySelector("canvas");
      charts.push({ def, canvas });
      ro.observe(canvas);
    }
  }

  function rangeOf(def, t0, t1) {
    let lo = Infinity, hi = -Infinity;
    for (const s of def.series) {
      const arrs = s.band ? [s.lo, s.hi] : [s.y];
      for (const arr of arrs) {
        arr.forEach((v, k) => { if (fin(v) && s.x[k] >= t0 && s.x[k] <= t1) { lo = Math.min(lo, v); hi = Math.max(hi, v); } });
      }
    }
    if (def.zero) { lo = Math.min(lo, 0); hi = Math.max(hi, 0); }
    if (!isFinite(lo)) { lo = 0; hi = 1; }
    if (hi - lo < 1e-6) { lo -= 0.5; hi += 0.5; }
    const pad = (hi - lo) * 0.08;
    return [lo - pad, hi + pad];
  }

  function drawChart(c) {
    const { ctx, w, h } = setup(c.canvas);
    const def = c.def;
    const t0 = run.frames.t[0], t1 = tEnd();
    const [lo, hi] = rangeOf(def, t0, t1);
    const ml = 44, mr = 8, mt = 8, mb = 20;
    const X = (t) => ml + (t - t0) / (t1 - t0) * (w - ml - mr);
    const Y = (v) => mt + (1 - (v - lo) / (hi - lo)) * (h - mt - mb);
    ctx.fillStyle = C.panel; ctx.fillRect(0, 0, w, h);
    ctx.font = `10.5px ${C.mono}`; ctx.lineWidth = 1;
    for (const v of niceTicks(lo, hi, 4)) {
      ctx.strokeStyle = C.grid; ctx.beginPath(); ctx.moveTo(ml, Y(v)); ctx.lineTo(w - mr, Y(v)); ctx.stroke();
      ctx.fillStyle = C.muted; ctx.textAlign = "right"; ctx.fillText(String(+v.toFixed(3)), ml - 4, Y(v) + 3);
    }
    ctx.textAlign = "center";
    for (const v of niceTicks(t0, t1, 6)) { ctx.fillStyle = C.muted; ctx.fillText(`${v}s`, X(v), h - 5); }
    ctx.textAlign = "left";
    if (def.zero) { ctx.strokeStyle = C.muted; ctx.beginPath(); ctx.moveTo(ml, Y(0)); ctx.lineTo(w - mr, Y(0)); ctx.stroke(); }
    if (def.shade) {
      ctx.fillStyle = C.sensor; ctx.globalAlpha = 0.08;
      const t = run.frames.t;
      for (let k = 0; k < t.length - 1; k++) if (def.shade[k]) ctx.fillRect(X(t[k]), mt, X(t[k + 1]) - X(t[k]) + 0.5, h - mt - mb);
      ctx.globalAlpha = 1;
    }
    ctx.save();
    ctx.beginPath(); ctx.rect(ml, mt, w - ml - mr, h - mt - mb); ctx.clip();
    for (const s of def.series) {
      if (s.band) {
        ctx.fillStyle = s.color; ctx.globalAlpha = 0.14;
        let open = false;
        const pts = [];
        const flush = () => {
          if (pts.length > 1) {
            ctx.beginPath();
            pts.forEach((p, k) => (k ? ctx.lineTo(X(p[0]), Y(p[2])) : ctx.moveTo(X(p[0]), Y(p[2]))));
            for (let k = pts.length - 1; k >= 0; k--) ctx.lineTo(X(pts[k][0]), Y(pts[k][1]));
            ctx.closePath(); ctx.fill();
          }
          pts.length = 0;
        };
        s.x.forEach((x, k) => { if (fin(s.lo[k]) && fin(s.hi[k])) { pts.push([x, s.lo[k], s.hi[k]]); open = true; } else if (open) { flush(); open = false; } });
        flush();
        ctx.globalAlpha = 1;
      } else if (s.points) {
        ctx.fillStyle = s.color; ctx.globalAlpha = 0.75;
        s.x.forEach((x, k) => { if (fin(s.y[k])) { ctx.beginPath(); ctx.arc(X(x), Y(s.y[k]), 1.8, 0, 2 * Math.PI); ctx.fill(); } });
        ctx.globalAlpha = 1;
      } else {
        ctx.strokeStyle = s.color; ctx.lineWidth = s.width || 1.5; ctx.setLineDash(s.dash || []);
        ctx.beginPath();
        let pen = false;
        s.x.forEach((x, k) => { const v = s.y[k]; if (fin(v)) { pen ? ctx.lineTo(X(x), Y(v)) : ctx.moveTo(X(x), Y(v)); pen = true; } else pen = false; });
        ctx.stroke(); ctx.setLineDash([]);
      }
    }
    ctx.restore();
    // event ticks
    for (const kind of ["trigger_on", "state:TRACKING", "release_command", "state:LANDED", "state:ABORTED"]) {
      const e = firstEvent(kind);
      if (!e) continue;
      ctx.strokeStyle = kind === "state:ABORTED" ? C.bad : C.muted; ctx.globalAlpha = 0.6;
      ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(X(e.t), mt); ctx.lineTo(X(e.t), h - mb); ctx.stroke(); ctx.setLineDash([]);
      ctx.globalAlpha = 1;
    }
    // cursor
    ctx.strokeStyle = C.ink; ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.moveTo(X(tCur), mt); ctx.lineTo(X(tCur), h - mb); ctx.stroke();
  }

  // --------------------------------------------------------- text panels
  function renderStates() {
    const visited = new Set(["IDLE"]);
    let now = "IDLE";
    for (const e of run.events) {
      if (e.kind.startsWith("state:") && e.t <= tCur + 1e-9) { now = e.kind.slice(6); visited.add(now); }
    }
    $("states").innerHTML = STATES.map((s) => {
      const cls = ["state"];
      if (visited.has(s)) cls.push("done");
      if (s === now) cls.push("now");
      if (s === "ABORTED") cls.push("abort");
      return `<span class="${cls.join(" ")}">${s}</span>`;
    }).join("");
  }

  function renderReadouts() {
    const i = idxAt(tCur), sc = run.scene;
    const ev = F("est_v", i), sv = F("sig_v", i), ey = F("est_y", i), sy = F("sig_y", i);
    const tl = F("t_land", i), st = F("sigma_t", i), slack = F("slack", i);
    const state = STATES[F("state", i)];
    let landing = "—";
    if (run.summary.t_land !== undefined && tCur >= run.summary.t_land) landing = "landed";
    else if (fin(tl)) landing = `in ${(tl - tCur).toFixed(2)} s`;
    const gateOk = F("safety_ok", i) > 0;
    const gatePill = state === "ABORTED" ? '<span class="pill bad">stood down</span>'
      : fin(tl) ? (gateOk ? '<span class="pill ok">clear</span>' : '<span class="pill bad">blocked</span>')
        : '<span class="pill idle">idle</span>';
    const cells = [
      ["True speed", `${fmt(F("veh_v", i) * KMH, 1)} km/h`, "ground truth"],
      ["Estimated speed", fin(ev) ? `${fmt(ev * KMH, 1)} km/h` : "—", fin(sv) ? `± ${fmt(2 * sv * KMH, 2)} km/h (2σ)` : "no track yet"],
      ["Lane offset", fin(ey) ? `${fmt(ey, 2)} m` : "—", fin(sy) ? `± ${fmt(2 * sy, 2)} m · true ${sc.lane_y.toFixed(2)}` : `true ${sc.lane_y.toFixed(2)} m`],
      ["Landing", landing, fin(st) ? `σ_t ${(st * 1000).toFixed(1)} ms` : ""],
      ["Schedule slack", fin(slack) ? `${fmt(slack, 2)} s` : "—", "time to spare for the gantry"],
      ["Safety gate", gatePill, ""],
    ];
    $("readouts").innerHTML = cells.map(([k, v, s]) => `<div class="readout"><div class="k">${k}</div><div class="v">${v}</div><div class="s">${s}</div></div>`).join("");
  }

  const EVENT_TEXT = {
    trigger_on: (e) => `Acoustic trigger: ${fmt(e.level_db, 1)} dB over a ${fmt(e.floor_db, 1)} dB floor (vehicle at x = ${fmt(e.vehicle_x, 1)} m)`,
    trigger_off: () => "Acoustic trigger released",
    first_plan: (e) => `First plan: aim x = ${fmt(e.x_aim, 2)} m, release command at ${fmt(e.t_cmd, 2)} s, slack ${fmt(e.slack, 2)} s`,
    release_command: (e) => `Release command. Predicted landing at ${fmt(e.t_land, 3)} s (σ_t ${fmt(e.sigma_t * 1000, 1)} ms)`,
    no_detection: () => "Nothing detected before the time limit",
  };
  function eventText(e) {
    if (EVENT_TEXT[e.kind]) return EVENT_TEXT[e.kind](e);
    if (e.kind.startsWith("state:")) {
      const s = e.kind.slice(6);
      if (s === "ABORTED") return `<span class="bad">Stood down: ${e.reason}${e.failed ? " (" + e.failed.join(", ") + ")" : ""}</span>`;
      if (s === "TRACKING") return `Track initialised from ${e.sensor}`;
      if (s === "COMMITTED") return `Aim frozen at x = ${fmt(e.x_aim, 2)} m`;
      if (s === "RELEASED") return `Payload released at ${fmt(e.launch_speed, 2)} m/s, ${fmt(e.elevation_deg, 1)}°`;
      if (s === "LANDED") return `Landed at (${fmt(e.landing[0], 2)}, ${fmt(e.landing[1], 2)}) m`;
      return `State: ${s}`;
    }
    return e.kind;
  }
  function renderEvents() {
    $("events").innerHTML = run.events.map((e) =>
      `<li class="${e.t > tCur + 1e-9 ? "future" : ""}"><span class="t">${e.t.toFixed(2)}</span><span>${eventText(e)}</span></li>`).join("");
  }

  function renderResult() {
    const s = run.summary;
    const rows = [
      ["Status", s.status + (s.abort_reason ? ` (${s.abort_reason})` : "")],
      ["Trigger fired with vehicle at", fin(s.x_at_trigger) ? `${fmt(s.x_at_trigger, 1)} m` : "—"],
      ["Speed error at release command", fin(s.speed_error_at_cmd) ? `${fmt(s.speed_error_at_cmd * KMH, 2)} km/h` : "—"],
      ["Lane-offset error at command", fin(s.y_error_at_cmd) ? `${fmt(s.y_error_at_cmd, 3)} m` : "—"],
      ["Predicted timing σ", fin(s.predicted_sigma_t) ? `${fmt(s.predicted_sigma_t * 1000, 1)} ms` : "—"],
      ["Actual timing error", fin(s.timing_error) ? `${fmt(s.timing_error * 1000, 1)} ms` : "—"],
      ["Along-track error vs vehicle", fin(s.along_track_error) ? `${fmt(s.along_track_error, 3)} m` : "—"],
      ["Landing miss from aim (x, y)", fin(s.miss_x) ? `${fmt(s.miss_x, 3)}, ${fmt(s.miss_y, 3)} m` : "—"],
      ["Landed inside test zone", s.in_zone === undefined ? "—" : (s.in_zone ? "yes" : "no")],
      ["Clearance to vehicle side", fin(s.lateral_clearance) ? `${fmt(s.lateral_clearance, 2)} m` : "—"],
      ["Late measurements re-processed", `${s.ekf_replays}`],
      ["Outliers rejected", `${s.ekf_rejected}`],
    ];
    $("result").innerHTML = rows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join("");
    $("result-pill").innerHTML = s.status === "LANDED"
      ? `<span class="pill ok">landed ${s.in_zone ? "in zone" : "OUTSIDE zone"}</span>`
      : `<span class="pill bad">${s.status.toLowerCase()}</span>`;
    const checks = s.release_checks || (firstEvent("state:ABORTED") || {}).checks;
    const names = { in_zone: "landing ellipse in zone", lane_clearance: "lane clearance", track_quality: "track quality", speed_plausible: "speed plausible", schedule: "schedule feasible", carriage_ready: "carriage settled", track_and_plan: "track and plan exist" };
    if (checks) {
      $("checks").innerHTML = Object.entries(checks).map(([k, c]) => `<span class="pill ${c.ok ? "ok" : "bad"}">${c.ok ? "✓" : "✗"} ${names[k] || k}</span>`).join("");
      $("checks-table").innerHTML = Object.entries(checks).filter(([, c]) => c.value !== null).map(([k, c]) =>
        `<tr><td>${names[k] || k}</td><td>${fmt(c.value, 3)}${c.limit !== null ? ` / limit ${fmt(c.limit, 2)}` : ""}</td></tr>`).join("");
    } else {
      $("checks").innerHTML = '<span class="pill idle">not evaluated: no release was attempted</span>';
      $("checks-table").innerHTML = "";
    }
  }

  // ------------------------------------------------------------ Monte Carlo
  function renderMC() {
    if (!mcData) { $("mc-stats").innerHTML = ""; return; }
    const st = mcData.stats;
    $("mc-meta").textContent = `${st.n} trials` + (mcData.label ? ` · ${mcData.label}` : "");
    const tiles = [
      ["landed in zone", `${Math.round((st.in_zone_rate || 0) * st.landed)}/${st.n}`],
      ["timing σ measured", fin(st.timing_error_std) ? `${(st.timing_error_std * 1000).toFixed(1)} ms` : "—"],
      ["timing σ predicted", fin(st.predicted_sigma_t_mean) ? `${(st.predicted_sigma_t_mean * 1000).toFixed(1)} ms` : "—"],
      ["95 % |timing error|", fin(st.timing_error_p95_abs) ? `${(st.timing_error_p95_abs * 1000).toFixed(1)} ms` : "—"],
      ["along-track σ", fin(st.along_track_std) ? `${st.along_track_std.toFixed(2)} m` : "—"],
      ["min clearance", fin(st.min_lateral_clearance) ? `${st.min_lateral_clearance.toFixed(2)} m` : "—"],
    ];
    $("mc-stats").innerHTML = tiles.map(([k, v]) => `<div class="stat"><div class="v">${v}</div><div class="k">${k}</div></div>`).join("");
    drawScatter(); drawHist();
  }

  function axes(ctx, w, h, xr, yr, xl, yl) {
    const ml = 44, mr = 10, mt = 10, mb = 32;
    const X = (v) => ml + (v - xr[0]) / (xr[1] - xr[0]) * (w - ml - mr);
    const Y = (v) => mt + (1 - (v - yr[0]) / (yr[1] - yr[0])) * (h - mt - mb);
    ctx.fillStyle = C.panel; ctx.fillRect(0, 0, w, h);
    ctx.font = `10.5px ${C.mono}`; ctx.lineWidth = 1;
    for (const v of niceTicks(yr[0], yr[1], 5)) { ctx.strokeStyle = C.grid; ctx.beginPath(); ctx.moveTo(ml, Y(v)); ctx.lineTo(w - mr, Y(v)); ctx.stroke(); ctx.fillStyle = C.muted; ctx.textAlign = "right"; ctx.fillText(String(+v.toFixed(3)), ml - 4, Y(v) + 3); }
    ctx.textAlign = "center";
    for (const v of niceTicks(xr[0], xr[1], 6)) { ctx.strokeStyle = C.grid; ctx.beginPath(); ctx.moveTo(X(v), mt); ctx.lineTo(X(v), h - mb); ctx.stroke(); ctx.fillStyle = C.muted; ctx.fillText(String(+v.toFixed(3)), X(v), h - mb + 13); }
    ctx.fillText(xl, (ml + w - mr) / 2, h - 4);
    ctx.save(); ctx.translate(11, (mt + h - mb) / 2); ctx.rotate(-Math.PI / 2); ctx.fillText(yl, 0, 0); ctx.restore();
    ctx.textAlign = "left";
    return { X, Y, ml, mr, mt, mb };
  }

  function drawScatter() {
    const { ctx, w, h } = setup($("mc-scatter"));
    const rows = mcData.rows.filter((r) => r.status === "LANDED");
    const xs = rows.map((r) => r.along_track_error), ys = rows.map((r) => r.miss_y);
    const xm = Math.max(0.6, ...xs.map(Math.abs)) * 1.1, ym = Math.max(0.35, ...ys.map(Math.abs)) * 1.1;
    const a = axes(ctx, w, h, [-xm, xm], [-ym, ym], "along-track: vehicle ahead of landing point [m]", "lateral miss [m]");
    ctx.strokeStyle = C.muted; ctx.beginPath(); ctx.moveTo(a.X(0), a.mt); ctx.lineTo(a.X(0), h - a.mb); ctx.moveTo(a.ml, a.Y(0)); ctx.lineTo(w - a.mr, a.Y(0)); ctx.stroke();
    ctx.fillStyle = C.payload; ctx.globalAlpha = 0.6;
    rows.forEach((r) => { ctx.beginPath(); ctx.arc(a.X(r.along_track_error), a.Y(r.miss_y), 2.6, 0, 2 * Math.PI); ctx.fill(); });
    ctx.globalAlpha = 1;
  }

  function drawHist() {
    const { ctx, w, h } = setup($("mc-hist"));
    const st = mcData.stats;
    const vals = mcData.rows.filter((r) => r.status === "LANDED").map((r) => r.timing_error * 1000);
    if (!vals.length) return;
    const sig = (st.predicted_sigma_t_mean || 0.02) * 1000;
    const lim = Math.max(4 * sig, ...vals.map(Math.abs)) * 1.05;
    const nb = 21, bw = 2 * lim / nb;
    const counts = new Array(nb).fill(0);
    vals.forEach((v) => { const b = Math.min(nb - 1, Math.max(0, Math.floor((v + lim) / bw))); counts[b]++; });
    const cmax = Math.max(...counts, 1);
    const a = axes(ctx, w, h, [-lim, lim], [0, cmax * 1.15], "timing error [ms] (+ = vehicle early)", "trials");
    ctx.fillStyle = C.estimate; ctx.globalAlpha = 0.55;
    counts.forEach((c, b) => { const x0 = -lim + b * bw; ctx.fillRect(a.X(x0) + 1, a.Y(c), a.X(x0 + bw) - a.X(x0) - 2, a.Y(0) - a.Y(c)); });
    ctx.globalAlpha = 1;
    // predicted normal curve
    ctx.strokeStyle = C.vehicle; ctx.lineWidth = 2; ctx.beginPath();
    for (let k = 0; k <= 120; k++) {
      const x = -lim + 2 * lim * k / 120;
      const y = vals.length * bw * Math.exp(-0.5 * (x / sig) ** 2) / (sig * Math.sqrt(2 * Math.PI));
      k ? ctx.lineTo(a.X(x), a.Y(y)) : ctx.moveTo(a.X(x), a.Y(y));
    }
    ctx.stroke();
    ctx.fillStyle = C.vehicle; ctx.font = `11px ${C.body}`;
    ctx.fillText(`predicted N(0, ${sig.toFixed(1)} ms)`, a.ml + 6, a.mt + 12);
  }

  // ---------------------------------------------------------------- render
  function renderTime() {
    if (!run) return;
    const t0 = run.frames.t[0], t1 = tEnd();
    $("scrub").value = String(Math.round((tCur - t0) / (t1 - t0) * 1000));
    $("clock").textContent = `${tCur.toFixed(2)} s`;
    drawPlan(); drawSide();
    charts.forEach(drawChart);
    renderStates(); renderReadouts(); renderEvents();
  }

  function restingTime() {
    const s = run.summary;
    if (fin(s.t_land)) return Math.min(tEnd(), s.t_land + 0.05);
    const ab = firstEvent("state:ABORTED");
    if (ab) return Math.min(tEnd(), ab.t);
    return tEnd();
  }

  function setRun(r, desc) {
    run = r;
    readColors();
    buildCharts();
    renderResult();
    tCur = restingTime();
    $("scenario-desc").textContent = desc || "";
    renderTime();
  }

  // ------------------------------------------------------------- playback
  function tick(ts) {
    if (!playing) return;
    if (lastTs !== null) {
      tCur += (ts - lastTs) / 1000 * parseFloat($("rate").value);
      if (tCur >= tEnd()) { tCur = tEnd(); stop(); }
    }
    lastTs = ts;
    renderTime();
    if (playing) requestAnimationFrame(tick);
  }
  function play() {
    if (!run) return;
    if (tCur >= tEnd() - 1e-6 || tCur >= restingTime() - 1e-6) tCur = Math.max(run.frames.t[0], (firstEvent("trigger_on") || { t: 1 }).t - 1.0);
    playing = true; lastTs = null; $("play").textContent = "Pause";
    requestAnimationFrame(tick);
  }
  function stop() { playing = false; $("play").textContent = "Play"; }

  $("play").addEventListener("click", () => (playing ? stop() : play()));
  $("scrub").addEventListener("input", (e) => {
    if (!run) return;
    stop();
    const t0 = run.frames.t[0];
    tCur = t0 + (tEnd() - t0) * (+e.target.value / 1000);
    renderTime();
  });
  document.querySelectorAll("[data-zoom]").forEach((b) => b.addEventListener("click", () => {
    zoom = b.dataset.zoom;
    document.querySelectorAll("[data-zoom]").forEach((o) => o.setAttribute("aria-pressed", String(o === b)));
    renderTime();
  }));

  const ro = new ResizeObserver(() => { if (run) { renderTime(); if (mcData) { drawScatter(); drawHist(); } } });
  [$("plan"), $("side"), $("mc-scatter"), $("mc-hist")].forEach((c) => ro.observe(c));
  const rethemed = () => { readColors(); if (run) { buildCharts(); renderTime(); renderMC(); } };
  window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", rethemed);
  new MutationObserver(rethemed).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });

  // ------------------------------------------------------------ live mode
  const FIELDS = [
    ["Scenario", [
      ["vehicle.speed_kmh", "Vehicle speed [km/h]", 5, 140, 1],
      ["vehicle.lane_y", "Lane distance from rig [m]", 4, 25, 0.1],
      ["vehicle.accel", "Vehicle acceleration [m/s²]", -3, 3, 0.1],
      ["vehicle.source_level_db", "Vehicle loudness @7.5 m [dB]", 50, 90, 1],
      ["environment.gust_sigma", "Unknown wind gust σ [m/s]", 0, 3, 0.1],
      ["sim.seed", "Random seed", 0, 99999, 1],
    ]],
    ["Sensors and timing", [
      ["radar.noise", "Radar noise σ [m/s]", 0, 2, 0.01],
      ["radar.timing.latency", "Radar latency [s]", 0, 1, 0.01],
      ["microphone.timing.latency", "Microphone latency [s]", 0, 1, 0.01],
      ["lidar.timing.latency", "Light-gate latency [s]", 0, 0.5, 0.005],
      ["estimator.compute_latency", "Processing latency [s]", 0, 0.2, 0.001],
      ["release.latency_jitter", "Release jitter σ [s]", 0, 0.05, 0.001],
    ]],
    ["Actuator and planner", [
      ["gantry.v_max", "Gantry max speed [m/s]", 0.1, 5, 0.1],
      ["gantry.a_max", "Gantry max accel [m/s²]", 0.2, 20, 0.1],
      ["payload.launch_speed", "Launch speed [m/s]", 3, 12, 0.1],
      ["planner.landing_offset_s", "Land after abeam by [s]", -1, 1, 0.05],
      ["prediction.horizon_s", "Prediction display τ [s]", 0.1, 3, 0.1],
      ["safety.lane_clearance_min", "Minimum lane clearance [m]", 0.5, 5, 0.1],
    ]],
    ["Estimator inputs", [
      ["estimator.use_radar", "Use radar", "bool"],
      ["estimator.use_mic_bearing", "Use microphone bearing", "bool"],
      ["estimator.compensate_sound_delay", "Compensate sound travel time", "bool"],
    ]],
  ];
  const getPath = (obj, path) => path.split(".").reduce((o, k) => (o ? o[k] : undefined), obj);

  function buildControls(cfg) {
    const host = $("controls-host");
    host.innerHTML = `<div class="panel-head" style="margin-top:12px"><h2>Parameters</h2>
      <div class="mc-row" style="margin:0"><button type="button" class="primary" id="run-btn">Run simulation</button>
      <button type="button" id="reset-btn">Reset to scenario</button><span class="muted" id="run-status"></span></div></div>
      <div class="controls">${FIELDS.map(([legend, fields]) => `<fieldset><legend>${legend}</legend>${fields.map(([path, label, a, b, step]) => {
        const id = "f-" + path.replace(/\./g, "-");
        const val = getPath(cfg, path);
        if (a === "bool") return `<label class="field check" for="${id}"><input type="checkbox" id="${id}" data-path="${path}" ${val ? "checked" : ""}>${label}</label>`;
        return `<label class="field" for="${id}"><span>${label}</span><input type="number" id="${id}" data-path="${path}" min="${a}" max="${b}" step="${step}" value="${val}"></label>`;
      }).join("")}</fieldset>`).join("")}</div>`;
    $("run-btn").addEventListener("click", () => liveRun(true));
    $("reset-btn").addEventListener("click", () => liveRun(false));
  }
  function collectOverrides() {
    const out = {};
    document.querySelectorAll("[data-path]").forEach((el) => {
      out[el.dataset.path] = el.type === "checkbox" ? el.checked : parseFloat(el.value);
    });
    if ("sim.seed" in out) out["sim.seed"] = Math.round(out["sim.seed"]);
    return out;
  }
  async function api(path, body) {
    const r = await fetch(path, { method: body ? "POST" : "GET", headers: { "Content-Type": "application/json" }, body: body ? JSON.stringify(body) : undefined });
    if (!r.ok) throw new Error((await r.text()) || r.statusText);
    return r.json();
  }
  let livePresets = [];
  async function liveRun(withOverrides) {
    const btn = $("run-btn"), status = $("run-status");
    if (btn) btn.disabled = true;
    if (status) status.textContent = "running…";
    try {
      const preset = livePresets.find((p) => p.id === presetId);
      const res = await api("/api/run", { preset: presetId, overrides: withOverrides ? collectOverrides() : {} });
      stop();
      setRun(res, preset ? preset.description : "");
      if (!withOverrides || !document.querySelector("[data-path]")) buildControls(res.config);
      if (status) status.textContent = `done: ${res.summary.status.toLowerCase()}`;
    } catch (err) {
      if (status) status.textContent = `Run failed: ${err.message}`;
    } finally {
      if ($("run-btn")) $("run-btn").disabled = false;
    }
  }
  async function liveMC() {
    const btn = $("mc-run"), n = Math.max(10, Math.min(1000, parseInt($("mc-n").value, 10) || 100));
    btn.disabled = true; $("mc-meta").textContent = `running ${n} trials…`;
    try {
      mcData = await api("/api/montecarlo", { preset: presetId, overrides: collectOverrides(), n });
      mcData.label = "current parameters";
      renderMC();
    } catch (err) {
      $("mc-meta").textContent = `Monte Carlo failed: ${err.message}`;
    } finally { btn.disabled = false; }
  }

  // ------------------------------------------------------------------ boot
  async function boot() {
    readColors();
    const sel = $("scenario");
    if (STATIC) {
      $("mode-badge").textContent = "static export · " + STATIC.generated;
      sel.innerHTML = STATIC.presets.map((p) => `<option value="${p.id}">${p.title}</option>`).join("");
      const show = (id) => { presetId = id; const p = STATIC.presets.find((q) => q.id === id); stop(); setRun(p.run, p.description); };
      sel.addEventListener("change", () => show(sel.value));
      $("controls-host").innerHTML = `<p class="static-note">This is a pre-computed export with ${STATIC.presets.length} scenarios. To change parameters and re-run live, start the local server with <code>python -m simcore serve</code> and open <code>http://localhost:8765</code>.</p>`;
      mcData = STATIC.montecarlo; if (mcData) mcData.label = "nominal scenario, speed ±8 km/h, lane ±0.5 m";
      $("mc-actions").innerHTML = "";
      show(STATIC.presets[0].id);
      renderMC();
      return;
    }
    $("mode-badge").textContent = "live · local server";
    try {
      livePresets = await api("/api/presets");
    } catch (err) {
      $("scenario-desc").textContent = `Could not reach the simulation server (${err.message}). Start it with: python -m simcore serve`;
      return;
    }
    sel.innerHTML = livePresets.map((p) => `<option value="${p.id}">${p.title}</option>`).join("");
    presetId = livePresets[0].id;
    sel.addEventListener("change", () => { presetId = sel.value; $("controls-host").innerHTML = ""; liveRun(false); });
    $("mc-actions").innerHTML = `<label for="mc-n" class="muted">Trials</label><input id="mc-n" type="number" min="10" max="1000" step="10" value="100" style="width:6rem;font:inherit;padding:4px 6px;border:1px solid var(--rule);border-radius:6px;background:var(--bg);color:var(--ink)">
      <button type="button" class="primary" id="mc-run">Run Monte Carlo</button><span class="muted" style="font-size:13px">Uses the parameters above, randomising speed ±8 km/h and lane ±0.5 m.</span>`;
    $("mc-run").addEventListener("click", liveMC);
    await liveRun(false);
  }
  boot();
})();
