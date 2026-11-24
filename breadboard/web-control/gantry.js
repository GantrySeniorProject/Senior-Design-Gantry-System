// gantry.js - page logic that doesn't touch the DOM or the serial port, so
// test.html can check it in a browser. index.html does the wiring.
(function (root) {
  "use strict";

  const COORD_MAX = 4096;          // matches RAIL_COORD_MAX in main/rail.h
  const AXES = ["X", "Y", "Z"];
  const Z_LEVELS = 9;              // Z bar: 0, 512 ... 4096 (0 = down, 4096 = up)
  const Z_STEP = COORD_MAX / (Z_LEVELS - 1);

  // X/Y grid: n x n cells covering the whole homed rail, so changing n only changes
  // how big a cell is (span / n steps); no re-homing. Each size splits the previous
  // one's cells in 4, so blocked cells are stored once, at the finest size.
  const GRID_SIZES = [3, 8, 16, 32];
  const FINE = 32;
  const PATH_MAX = 64;             // waypoints per path, matches MOTION_PATH_MAX in main/motion.h

  // USB-serial chips used on ESP32 boards; the port picker only lists these.
  const USB_FILTERS = [
    { usbVendorId: 0x1a86 },       // WCH CH340 / CH343 (Freenove ESP32-WROVER)
    { usbVendorId: 0x10c4 },       // Silicon Labs CP210x
    { usbVendorId: 0x0403 },       // FTDI
    { usbVendorId: 0x303a },       // Espressif native USB
  ];

  function isEspPort(info) {
    return !!info && USB_FILTERS.some((f) => f.usbVendorId === info.usbVendorId);
  }

  // "@S mode=IDLE axis=- phase=MIN pos=2048,-1,-1 steps=1450,0,0 span=2900,0,0
  //  homed=1,0,0 estop=0 min=0,0,0 max=0,0,0 keep=0 zon=1 path=2/5 fault=NONE"
  // pos/steps/span/homed/min/max are X,Y,Z. axis = axis being homed ("-" if none).
  // path = waypoints started / total of the running path (0/0 = none).
  // The marker may follow echoed command text on the same line. fault= is last
  // and may contain spaces ("HOME TIMEOUT Z").
  function parseStatus(line) {
    const at = line.indexOf("@S ");
    if (at < 0) return null;
    const body = line.slice(at + 3).trim();
    const fAt = body.indexOf("fault=");
    if (fAt < 0) return null;
    const f = {};
    for (const tok of body.slice(0, fAt).trim().split(/\s+/)) {
      const eq = tok.indexOf("=");
      if (eq > 0) f[tok.slice(0, eq)] = tok.slice(eq + 1);
    }
    const triple = (k) => {
      const parts = (f[k] || "").split(",");
      if (parts.length !== 3 || !parts.every((p) => /^-?\d+$/.test(p))) return null;
      return parts.map((p) => parseInt(p, 10));
    };
    const pos = triple("pos"), steps = triple("steps"), span = triple("span");
    const homed = triple("homed"), min = triple("min"), max = triple("max");
    if (!f.mode || !f.phase || !pos || !steps || !span || !homed || !min || !max) return null;
    const pm = /^(\d+)\/(\d+)$/.exec(f.path || "");
    return {
      mode: f.mode, phase: f.phase,
      axis: AXES.includes(f.axis) ? f.axis : null,
      pos, steps, span,
      homed: homed.map((v) => v === 1), min: min.map((v) => v === 1), max: max.map((v) => v === 1),
      estop: f.estop === "1",
      keep: f.keep === "1",        // "r k" would keep homing (E-stop with homed axes)
      zOn: f.zon !== "0",          // Z axis enabled in firmware (USE_Z_AXIS); missing = on
      path: pm && +pm[2] > 0 ? { started: +pm[1], total: +pm[2] } : null,
      fault: body.slice(fAt + 6).trim(),
    };
  }

  // ----------------------------- Z bar --------------------------------------
  const zCoord = (i) => Math.round(i * Z_STEP);
  function zIndex(coord) {
    if (!(coord >= 0)) return -1;  // not homed (-1) or NaN
    return Math.max(0, Math.min(Z_LEVELS - 1, Math.round(coord / Z_STEP)));
  }
  // Where to draw a Z coordinate, as a fraction 0..1 of the track: the levels sit
  // at the centres of the buttons, so 0 and 4096 are half a button in.
  function markerFraction(coord, count) {
    if (!(coord >= 0)) return null;
    const c = Math.min(coord, COORD_MAX);
    return (c / COORD_MAX * (count - 1) + 0.5) / count;
  }

  // ----------------------------- X/Y grid cells -----------------------------
  // Cell i of n covers coordinates [i, i+1) * 4096/n; its centre is where the
  // carriage stops. Centres are exact for n = 8, 16, 32 and rounded to a whole
  // coordinate otherwise (n = 3: 683, 2048, 3413).
  const cellCenter = (n, i) => Math.round((2 * i + 1) * COORD_MAX / (2 * n));
  function cellOf(n, coord) {
    if (!(coord >= 0)) return -1;
    return Math.max(0, Math.min(n - 1, Math.floor(coord * n / COORD_MAX)));
  }
  const gridFraction = (coord) => (coord >= 0 ? Math.min(coord, COORD_MAX) / COORD_MAX : null);
  // Physical size of a cell in motor steps, from the homed span
  const cellSteps = (n, span) => (span > 0 ? span / n : null);

  // Blocked cells (something sits there; the payload must not go through).
  // Stored at 32 x 32; a bigger cell is blocked if any 32 x 32 cell inside it is.
  // Each 32 x 32 cell belongs to the bigger cell that holds its centre, so sizes
  // that don't divide 32 (3 x 3: 11 + 10 + 11 fine cells) still split it cleanly.
  const fineRange = (n, i) => [Math.ceil(i * FINE / n - 0.5), Math.ceil((i + 1) * FINE / n - 0.5)];
  class Obstacles {
    constructor() { this.fine = new Uint8Array(FINE * FINE); }
    _block(n, i, j, fn) {
      const [x0, x1] = fineRange(n, i), [y0, y1] = fineRange(n, j);
      for (let y = y0; y < y1; y++)
        for (let x = x0; x < x1; x++) fn(y * FINE + x);
    }
    isBlocked(n, i, j) {
      if (i < 0 || j < 0 || i >= n || j >= n) return true;   // off the grid counts as a wall
      let b = false;
      this._block(n, i, j, (idx) => { if (this.fine[idx]) b = true; });
      return b;
    }
    set(n, i, j, blocked) {
      if (i < 0 || j < 0 || i >= n || j >= n) return;
      this._block(n, i, j, (idx) => { this.fine[idx] = blocked ? 1 : 0; });
    }
    clear() { this.fine.fill(0); }
    count() { return this.fine.reduce((a, v) => a + v, 0); }   // blocked 32 x 32 cells
    // n x n table of blocked flags (fast lookups for the planner)
    table(n) {
      const t = new Uint8Array(n * n);
      for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) t[j * n + i] = this.isBlocked(n, i, j) ? 1 : 0;
      return t;
    }
    // Saved form: indexes of the blocked 32 x 32 cells, e.g. "33,34,65"
    toString() {
      const out = [];
      this.fine.forEach((v, idx) => { if (v) out.push(idx); });
      return out.join(",");
    }
    static fromString(s) {
      const o = new Obstacles();
      for (const t of String(s || "").split(",")) {
        if (!/^\d+$/.test(t)) continue;
        const idx = +t;
        if (idx < FINE * FINE) o.fine[idx] = 1;
      }
      return o;
    }
  }

  // ----------------------------- A* -----------------------------------------
  // 4-way moves (up / down / left / right), each costs 1, heuristic = Manhattan
  // distance. Among the shortest paths it returns one with the fewest turns:
  // the cost is compared as (length, turns), and the search state is the cell
  // plus the direction it was entered from. Manhattan stays admissible and
  // consistent for that cost, so the first time the goal comes out of the open
  // list the path is optimal.
  //   blocked(i, j) -> true for cells that can't be entered. The start cell is
  //   always allowed (the carriage is already there).
  //   trace (optional array): every step of the search is pushed to it (see
  //   formatTrace); leave it out and the search does no extra work.
  // Returns { cells: [[i,j], ...] from start to goal, length, turns, expanded } or null.
  const DIRS = [[1, 0], [-1, 0], [0, 1], [0, -1]];   // +X, -X, +Y, -Y

  class MinHeap {                     // items: [f, turns, -g, id]; lexicographic
    constructor() { this.a = []; }
    get size() { return this.a.length; }
    static less(x, y) {
      for (let k = 0; k < 3; k++) if (x[k] !== y[k]) return x[k] < y[k];
      return false;
    }
    push(it) {
      const a = this.a; a.push(it);
      let i = a.length - 1;
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (!MinHeap.less(a[i], a[p])) break;
        [a[i], a[p]] = [a[p], a[i]]; i = p;
      }
    }
    pop() {
      const a = this.a, top = a[0], last = a.pop();
      if (a.length) {
        a[0] = last;
        let i = 0;
        for (;;) {
          const l = 2 * i + 1, r = l + 1;
          let m = i;
          if (l < a.length && MinHeap.less(a[l], a[m])) m = l;
          if (r < a.length && MinHeap.less(a[r], a[m])) m = r;
          if (m === i) break;
          [a[i], a[m]] = [a[m], a[i]]; i = m;
        }
      }
      return top;
    }
  }

  const manhattan = (i, j, gi, gj) => Math.abs(i - gi) + Math.abs(j - gj);

  function astar(n, blocked, start, goal, trace) {
    const [si, sj] = start, [gi, gj] = goal;
    const inside = (i, j) => i >= 0 && j >= 0 && i < n && j < n;
    const T = trace || null;
    if (T) T.push({ type: "start", n, start: [si, sj], goal: [gi, gj], h: manhattan(si, sj, gi, gj) });
    if (!inside(si, sj) || !inside(gi, gj)) { if (T) T.push({ type: "fail", reason: "off the grid", expanded: 0 }); return null; }
    if ((si !== gi || sj !== gj) && blocked(gi, gj)) { if (T) T.push({ type: "fail", reason: "goal blocked", expanded: 0 }); return null; }
    // state id = (j * n + i) * 5 + d, d = 0..3 direction entered, 4 = start
    const S = n * n * 5;
    const gLen = new Int32Array(S).fill(-1), gTurn = new Int32Array(S), parent = new Int32Array(S).fill(-1);
    const closed = new Uint8Array(S);
    const sid = (sj * n + si) * 5 + 4;
    gLen[sid] = 0;
    const open = new MinHeap();
    open.push([manhattan(si, sj, gi, gj), 0, 0, sid]);
    let expanded = 0;
    while (open.size) {
      const [, , , id] = open.pop();
      if (closed[id]) { if (T) T.push({ type: "stale" }); continue; }   // older, worse copy of a done state
      closed[id] = 1; expanded++;
      const cell = Math.floor(id / 5), d = id % 5, i = cell % n, j = Math.floor(cell / n);
      const h = manhattan(i, j, gi, gj);
      if (T) T.push({ type: "expand", step: expanded, cell: [i, j], dir: d, g: gLen[id], turns: gTurn[id], h,
                      f: gLen[id] + h, open: open.size });
      if (i === gi && j === gj) {                       // rebuild the path
        const cells = [];
        for (let s = id; s >= 0; s = parent[s]) { const c = Math.floor(s / 5); cells.push([c % n, Math.floor(c / n)]); }
        cells.reverse();
        if (T) T.push({ type: "goal", step: expanded, length: gLen[id], turns: gTurn[id], expanded, open: open.size });
        return { cells, length: gLen[id], turns: gTurn[id], expanded };
      }
      for (let nd = 0; nd < 4; nd++) {
        const ni = i + DIRS[nd][0], nj = j + DIRS[nd][1];
        if (!inside(ni, nj)) continue;                  // edge of the rail
        if (d !== 4 && (nd ^ 1) === d) continue;        // straight back where it came from: never shorter
        if (blocked(ni, nj)) { if (T) T.push({ type: "skip", cell: [ni, nj], dir: nd, reason: "blocked" }); continue; }
        const nid = (nj * n + ni) * 5 + nd;
        if (closed[nid]) { if (T) T.push({ type: "skip", cell: [ni, nj], dir: nd, reason: "closed" }); continue; }
        const len = gLen[id] + 1, turns = gTurn[id] + (d !== 4 && d !== nd ? 1 : 0);
        if (gLen[nid] >= 0 && (gLen[nid] < len || (gLen[nid] === len && gTurn[nid] <= turns))) {
          if (T) T.push({ type: "skip", cell: [ni, nj], dir: nd, reason: "worse", g: len, turns, haveG: gLen[nid], haveTurns: gTurn[nid] });
          continue;
        }
        const better = gLen[nid] >= 0;
        gLen[nid] = len; gTurn[nid] = turns; parent[nid] = id;
        const nh = manhattan(ni, nj, gi, gj);
        open.push([len + nh, turns, -len, nid]);
        if (T) T.push({ type: "push", cell: [ni, nj], dir: nd, g: len, turns, h: nh, f: len + nh, better });
      }
    }
    if (T) T.push({ type: "fail", reason: "open list empty", expanded });
    return null;                                        // goal can't be reached
  }

  // Keeps the start, every cell where the direction changes, and the goal:
  // each pair of neighbours in the result is one straight run along the grid.
  function corners(cells) {
    if (cells.length <= 2) return cells.slice();
    const out = [cells[0]];
    for (let k = 1; k < cells.length - 1; k++) {
      const [ai, aj] = cells[k - 1], [bi, bj] = cells[k], [ci, cj] = cells[k + 1];
      if (bi - ai !== ci - bi || bj - aj !== cj - bj) out.push(cells[k]);
    }
    out.push(cells[cells.length - 1]);
    return out;
  }

  // Whole plan from the carriage position (rail coordinates) to the centre of
  // goal cell [gi, gj] on an n x n grid. Every move is along X or along Y:
  //   1. line up with the start cell's centre on the axis the first run doesn't
  //      use (a short move inside the start cell, skipped if already lined up),
  //   2. then one move per straight run of the A* path, corner to corner.
  // Returns { ok: true, cells, waypoints: [[x,y], ...], length, turns }
  //      or { ok: false, error }.
  //   trace (optional array) collects the A* steps; the result then carries it.
  function planPath(n, obstacles, pos, goal, maxMoves = PATH_MAX, trace) {
    if (!GRID_SIZES.includes(n)) return { ok: false, error: "Unknown grid size." };
    if (!pos || !(pos[0] >= 0) || !(pos[1] >= 0)) return { ok: false, error: "Home X and Y first." };
    const start = [cellOf(n, pos[0]), cellOf(n, pos[1])];
    const t = obstacles.table(n);
    if (t[goal[1] * n + goal[0]] && (goal[0] !== start[0] || goal[1] !== start[1]))
      return { ok: false, error: "That cell is blocked." };
    const res = astar(n, (i, j) => t[j * n + i] === 1, start, goal, trace);
    if (!res) return { ok: false, error: "No path: the target is walled off by blocked cells.", trace, start, goal };
    const cs = corners(res.cells).map(([i, j]) => [cellCenter(n, i), cellCenter(n, j)]);
    const [cx, cy] = cs[0];
    let wps;
    if (cs.length === 1) {
      wps = [[pos[0], cy], [cx, cy]];                  // goal is the start cell: just centre up
    } else {
      const firstAlongX = cs[1][1] === cy;             // first run moves X (Y stays on the centre line)
      wps = [firstAlongX ? [pos[0], cy] : [cx, pos[1]], ...cs.slice(1)];
    }
    // drop moves that go nowhere
    const out = [];
    let at = [pos[0], pos[1]];
    for (const w of wps) {
      if (w[0] === at[0] && w[1] === at[1]) continue;
      out.push(w); at = w;
    }
    if (!out.length) return { ok: false, error: "Already there.", trace, start, goal };
    if (out.length > maxMoves) return { ok: false, error: "Path has too many turns (" + out.length + " moves, max " + maxMoves + ").", trace, start, goal };
    return { ok: true, cells: res.cells, waypoints: out, length: res.length, turns: res.turns, start, goal, trace,
             corners: corners(res.cells), expanded: res.expanded };
  }

  // Text for the "A* path calculation" panel: every step of the search, then the
  // result and what GO sends. maxLines keeps a huge search readable.
  const ARROWS = ["→", "←", "↑", "↓", "·"];        // direction a cell was entered from (+X, -X, +Y, -Y, start)
  function formatTrace(plan, pos, maxLines = 4000) {
    if (!plan || !plan.trace) return [];
    const L = [], R = [];           // L = search steps, R = result
    const c = (p) => "(" + p[0] + "," + p[1] + ")";
    const pad = (v, w) => String(v).padStart(w);
    let stale = 0;
    for (const e of plan.trace) {
      switch (e.type) {
        case "start":
          L.push("A* on a " + e.n + "×" + e.n + " grid, start cell " + c(e.start) + " → goal cell " + c(e.goal) + ".");
          L.push("Moves: up / down / left / right, 1 per cell. h = Manhattan distance |dx| + |dy| (start h = " + e.h + ").");
          L.push("Open list ordered by f = g + h, then fewest turns. g = cells walked so far. Arrow = direction entered.");
          L.push("");
          break;
        case "expand":
          L.push("#" + pad(e.step, 4) + "  expand " + c(e.cell).padEnd(8) + ARROWS[e.dir] + "  g=" + pad(e.g, 3) +
                 " turns=" + pad(e.turns, 2) + "  h=" + pad(e.h, 3) + "  f=" + pad(e.f, 3) + "   open list: " + e.open);
          break;
        case "push":
          L.push("        + " + c(e.cell).padEnd(8) + ARROWS[e.dir] + "  g=" + pad(e.g, 3) + " turns=" + pad(e.turns, 2) +
                 "  h=" + pad(e.h, 3) + "  f=" + pad(e.f, 3) + (e.better ? "   (better route, replaces the old one)" : ""));
          break;
        case "skip":
          L.push("        x " + c(e.cell).padEnd(8) + ARROWS[e.dir] + "  " +
                 (e.reason === "blocked" ? "blocked cell" :
                  e.reason === "closed" ? "already expanded" :
                  "not better (g=" + e.g + " turns=" + e.turns + " vs " + e.haveG + " turns=" + e.haveTurns + ")"));
          break;
        case "stale": stale++; break;
        case "goal":
          R.push("Goal reached at step #" + e.step + ": length " + e.length + " (cell-to-cell moves), " +
                 e.turns + " turn" + (e.turns === 1 ? "" : "s") + ". Expanded " + e.expanded + " states; " +
                 e.open + " still in the open list" + (stale ? "; " + stale + " outdated entries skipped." : "."));
          break;
        case "fail":
          R.push("No path: " + (e.reason === "goal blocked" ? "the goal cell is blocked." :
                                e.reason === "off the grid" ? "a cell is off the grid." :
                                "the open list ran out after " + e.expanded + " states (the goal is walled off)."));
          break;
      }
    }
    if (plan.ok) {
      R.push("Path (" + plan.cells.length + " cells): " + plan.cells.map(c).join(" "));
      R.push("Corners (straight runs between them): " + plan.corners.map(c).join(" → "));
      const from = pos ? "(" + pos[0] + ", " + pos[1] + ")" : "carriage";
      R.push("Moves GO sends (rail coordinates 0..4096), from " + from + ":");
      plan.waypoints.forEach((w, k) => R.push("  " + (k + 1) + ". X " + w[0] + ", Y " + w[1] +
                                                (k === 0 && plan.waypoints.length > 1 ? "   (line up inside the start cell)" : "")));
      R.push("Command: " + pathCommand(plan.waypoints));
    } else if (plan.error && !plan.trace.some((e) => e.type === "fail")) {
      R.push("Not sent: " + plan.error);
    }
    // Long searches: keep the first and last steps; the result is always shown in full
    let steps = L;
    if (L.length > maxLines) {
      const head = 4 + Math.floor((maxLines - 4) * 0.75), tail = maxLines - head;
      steps = [...L.slice(0, head), "", "... " + (L.length - maxLines) + " search lines not shown ...", "", ...L.slice(L.length - tail)];
    }
    return [...steps, "", ...R];
  }

  const pathCommand = (waypoints) => "w " + waypoints.map(([x, y]) => x + " " + y).join(" ");

  // ----------------------------- Buttons and hints --------------------------
  // X / Y path planning is allowed (idle, both homed)
  const canPlan = (st) => !!st && st.mode === "IDLE" && st.homed[0] && st.homed[1];

  // Command a button sends in a given state, or null when it should be disabled.
  // button: "estop" | "stop" | "home" | "home:X|Y|Z" | "reset" | "resetkeep" |
  //         "min" | "max" | "z:<i>". (Grid clicks plan a path: planPath / pathCommand.)
  function commandFor(button, st) {
    if (!st) return null;
    const idle = st.mode === "IDLE";
    switch (button) {
      case "estop": return "e";
      case "stop":  return st.mode === "MOVE" || st.mode === "HOME" ? "x" : null;
      case "home":  return idle ? "h" : null;
      case "reset": return st.mode === "FAULT" && !st.estop ? "r" : null;
      case "resetkeep": return st.mode === "FAULT" && !st.estop && st.keep ? "r k" : null;
      case "min":
      case "max":   // simulated press of the homing axis' limit (like its keypad key)
        return st.mode === "HOME" && st.axis ? "p" + st.axis.toLowerCase() + (button === "max" ? "1" : "0") : null;
    }
    let m = /^home:([XYZ])$/.exec(button);
    if (m) return idle && (m[1] !== "Z" || st.zOn !== false) ? "h " + m[1].toLowerCase() : null;
    m = /^z:(\d+)$/.exec(button);
    if (m) return idle && st.zOn !== false && st.homed[2] && +m[1] < Z_LEVELS ? "z " + zCoord(+m[1]) : null;
    return null;
  }

  // Keypad keys for each axis' limits (keypad.h)
  const KEYS = { X: ["0", "1"], Y: ["#", "3"], Z: ["D", "A"] };

  // One-line instruction for what to do next.
  function hint(st) {
    if (!st) return "Connect the board to start.";
    if (st.mode === "FAULT") {
      if (st.estop) return "Fault: " + st.fault + ". Release the keypad E-stop (*), then RESET.";
      return st.keep
        ? "Fault: " + st.fault + ". If nothing moved, RESET (keep home); otherwise RESET, then HOME."
        : "Fault: " + st.fault + ". Press RESET, then HOME.";
    }
    if (st.mode === "HOME" && st.axis) {
      const a = st.axis, k = KEYS[a], i = AXES.indexOf(a);
      return st.phase === "MIN"
        ? "Homing " + a + ": moving toward MIN. Press MIN (or keypad " + k[0] + ") at that end."
        : "Homing " + a + ": measuring, " + st.steps[i] + " steps so far. Press MAX (or keypad " + k[1] + ") at the other end.";
    }
    if (st.mode === "MOVE") {
      return st.path ? "Following the path: move " + st.path.started + " of " + st.path.total + "..." : "Moving...";
    }
    const active = st.zOn === false ? ["X", "Y"] : AXES;   // Z disabled: only X, Y need homing
    const unhomed = active.filter((a) => !st.homed[AXES.indexOf(a)]);
    if (unhomed.length === active.length)
      return "Not homed. Press HOME ALL (" + (st.zOn === false ? "X, then Y" : "Z, then X, then Y") + "); it ends at 0,0.";
    if (unhomed.length) return "Not homed yet: " + unhomed.join(", ") + ". Use HOME ALL or that axis' HOME button.";
    return st.zOn === false
      ? "Homed. Click a cell to plan a path, then GO. Use Block cells to mark where items are."
      : "Homed. Click a cell to plan a path, then GO; the Z bar raises / lowers the attachment.";
  }

  // ----------------------------- E-stop sound --------------------------------
  // The board latches an ESTOP fault however the E-stop came (keypad *, the page's
  // E-STOP button / Esc, or "e" in the monitor), so the sound follows the status.
  const isEstop = (st) => !!st && st.mode === "FAULT" && st.fault === "ESTOP";
  // true on the status line where the E-stop fault appears (not on the first line
  // after connecting, so opening the page on a stopped board stays quiet)
  const estopStarted = (prev, cur) => !!prev && !isEstop(prev) && isEstop(cur);
  // true when the E-stop fault goes away (RESET): a repeating sound stops
  const estopEnded = (prev, cur) => isEstop(prev) && !!cur && !isEstop(cur);

  // "90", "90s", "1m30s", "1h2m3s" -> seconds (YouTube t= / start=)
  function parseYoutubeTime(t) {
    t = String(t || "").trim();
    if (/^\d+$/.test(t)) return +t;
    const m = /^(?:(\d+)h)?(?:(\d+)m)?(?:(\d+)s)?$/.exec(t);
    if (!m || !t) return 0;
    return (+m[1] || 0) * 3600 + (+m[2] || 0) * 60 + (+m[3] || 0);
  }
  // YouTube link (watch, youtu.be, shorts, embed, live, music, mobile) or a bare
  // 11-character video id -> { id, start } or null
  function youtubeId(text) {
    const s = String(text || "").trim();
    if (/^[\w-]{11}$/.test(s)) return { id: s, start: 0 };
    let u;
    try { u = new URL(/^[a-z]+:\/\//i.test(s) ? s : "https://" + s); } catch (_) { return null; }
    const host = u.hostname.toLowerCase().replace(/^(www\.|m\.|music\.)/, "");
    let id = null;
    if (host === "youtu.be") id = u.pathname.split("/")[1];
    else if (host === "youtube.com" || host === "youtube-nocookie.com") {
      if (u.pathname === "/watch") id = u.searchParams.get("v");
      else { const m = /^\/(?:embed|shorts|live|v)\/([^/?#]+)/.exec(u.pathname); if (m) id = m[1]; }
    }
    if (!id || !/^[\w-]{11}$/.test(id)) return null;
    return { id, start: parseYoutubeTime(u.searchParams.get("t") || u.searchParams.get("start")) };
  }
  // Why a YouTube video won't play embedded (IFrame API onError codes)
  function youtubeError(code) {
    switch (code) {
      case 2: return "That doesn't look like a valid video id.";
      case 5: return "The browser can't play this video.";
      case 100: return "Video not found (removed or private).";
      case 101: case 150: return "The owner doesn't allow this video to play on other pages. Try another video.";
      case 153: return "YouTube needs the page to be opened from a web address: run serve.bat and use http://localhost:8000.";
    }
    return "YouTube error " + code + ".";
  }
  const isAudioFile = (f) => !!f && (/^audio\//.test(f.type || "") || /\.(mp3|wav|ogg|oga|m4a|aac|flac|webm|opus)$/i.test(f.name || ""));

  // Splits serial text into lines; keeps a partial line until its newline arrives.
  class LineBuffer {
    constructor() { this.buf = ""; }
    push(text) {
      this.buf += text;
      const parts = this.buf.split(/\r?\n/);
      this.buf = parts.pop();
      if (this.buf.length > 4096) this.buf = "";   // runaway line without newline
      return parts;
    }
  }

  root.Gantry = { COORD_MAX, AXES, Z_LEVELS, KEYS, USB_FILTERS, GRID_SIZES, FINE, PATH_MAX,
                  isEspPort, parseStatus, zCoord, zIndex, markerFraction,
                  cellCenter, cellOf, gridFraction, cellSteps, Obstacles, astar, corners, planPath,
                  pathCommand, formatTrace, canPlan, commandFor, hint, LineBuffer,
                  isEstop, estopStarted, estopEnded, parseYoutubeTime, youtubeId, youtubeError, isAudioFile };
})(typeof window !== "undefined" ? window : globalThis);
