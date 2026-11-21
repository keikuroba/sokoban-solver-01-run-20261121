// Push-optimal Sokoban solver — own work, standard techniques.
//
// Idea: the generated levels decompose exactly. The player starts in a
// one-cell-wide hallway; every room is a single connected component of
// floor off the hallway row, holds as many boxes as goals, and opens onto
// the hallway through exactly one door. A box pushed out through a door
// can never come back, so in an optimal solution every box stays in its
// room, and the level optimum P* is the sum of the rooms' optima. So:
//   1. split the level into rooms,
//   2. solve each room push-optimally with A* over pushes (macro moves),
//      heuristic = minimum assignment of boxes to goals by BFS distance,
//      pruned by dead squares (wall runs / corners with no goal),
//   3. stitch the rooms together with plain BFS walks through the
//      hallway, expanding each macro push into the player's steps.
// Deterministic: fixed iteration orders, no clock, no randomness, so two
// executions write the same bytes. One run handles the whole set.
//
// Local run: mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
import { mkdirSync, readdirSync, readFileSync, statSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Directions in fixed order: up, down, left, right.
const DIRS = [
  { dy: -1, dx: 0, ch: 'u' },
  { dy: 1, dx: 0, ch: 'd' },
  { dy: 0, dx: -1, ch: 'l' },
  { dy: 0, dx: 1, ch: 'r' },
] as const;

type Grid = {
  w: number;
  h: number;
  wall: boolean[]; // index y*w+x
  player: number;
  boxes: number[];
  goals: Set<number>;
};

function parseLevel(text: string): Grid {
  const raw = text.split('\n');
  while (raw.length > 0 && raw[raw.length - 1] === '') raw.pop();
  const h = raw.length;
  const w = Math.max(...raw.map((r) => r.length));
  const wall: boolean[] = new Array(w * h).fill(true);
  let player = -1;
  const boxes: number[] = [];
  const goals = new Set<number>();
  for (let y = 0; y < h; y++) {
    const row = raw[y];
    for (let x = 0; x < w; x++) {
      const c = x < row.length ? row[x] : '#';
      const i = y * w + x;
      if (c === '#') continue;
      wall[i] = false;
      if (c === '@' || c === '+') player = i;
      if (c === '$' || c === '*') boxes.push(i);
      if (c === '.' || c === '*' || c === '+') goals.add(i);
    }
  }
  return { w, h, wall, player, boxes, goals };
}

const inBounds = (g: Grid, y: number, x: number): boolean =>
  y >= 0 && y < g.h && x >= 0 && x < g.w;
const idx = (g: Grid, y: number, x: number): number => y * g.w + x;
const yOf = (g: Grid, i: number): number => Math.floor(i / g.w);
const xOf = (g: Grid, i: number): number => i % g.w;

// BFS distances from `from` over cells in `allowed` (Set of indexes).
// Returns a Map from cell index to distance.
function bfsDist(
  g: Grid,
  from: number,
  allowed: Set<number>,
): Map<number, number> {
  const dist = new Map<number, number>();
  const q: number[] = [from];
  dist.set(from, 0);
  for (let head = 0; head < q.length; head++) {
    const cur = q[head];
    const d = dist.get(cur)!;
    const cy = yOf(g, cur);
    const cx = xOf(g, cur);
    for (const dir of DIRS) {
      const ny = cy + dir.dy;
      const nx = cx + dir.dx;
      if (!inBounds(g, ny, nx)) continue;
      const ni = idx(g, ny, nx);
      if (!allowed.has(ni) || dist.has(ni)) continue;
      dist.set(ni, d + 1);
      q.push(ni);
    }
  }
  return dist;
}

// BFS path (as a move string) from `from` to `to` over `allowed` cells,
// treating cells in `blocked` as walls. Null when unreachable.
function bfsPath(
  g: Grid,
  from: number,
  to: number,
  allowed: Set<number>,
  blocked: Set<number>,
): string | null {
  if (from === to) return '';
  const prev = new Map<number, { p: number; ch: string }>();
  const q: number[] = [from];
  prev.set(from, { p: -1, ch: '' });
  for (let head = 0; head < q.length; head++) {
    const cur = q[head];
    const cy = yOf(g, cur);
    const cx = xOf(g, cur);
    for (const dir of DIRS) {
      const ny = cy + dir.dy;
      const nx = cx + dir.dx;
      if (!inBounds(g, ny, nx)) continue;
      const ni = idx(g, ny, nx);
      if (!allowed.has(ni) || blocked.has(ni) || prev.has(ni)) continue;
      prev.set(ni, { p: cur, ch: dir.ch });
      if (ni === to) {
        let s = '';
        let n = ni;
        while (n !== from) {
          const e = prev.get(n)!;
          s = e.ch + s;
          n = e.p;
        }
        return s;
      }
      q.push(ni);
    }
  }
  return null;
}

// Minimum-weight assignment of boxes to goals (brute-force permutations;
// at most 4 boxes) using the per-goal BFS distance tables.
function assignmentHeuristic(
  boxes: number[],
  goalList: number[],
  distFromGoal: Map<number, number>[],
): number {
  const n = boxes.length;
  if (n === 0) return 0;
  const order = goalList.map((_, i) => i);
  let best = Infinity;
  const permute = (k: number): void => {
    if (k === n) {
      let s = 0;
      for (let i = 0; i < n; i++) {
        const d = distFromGoal[order[i]].get(boxes[i]);
        if (d === undefined) return; // box cut off from this goal
        s += d;
      }
      if (s < best) best = s;
      return;
    }
    for (let i = k; i < n; i++) {
      const t = order[k];
      order[k] = order[i];
      order[i] = t;
      permute(k + 1);
      order[i] = order[k];
      order[k] = t;
    }
  };
  permute(0);
  return best;
}

type Room = {
  cells: Set<number>;
  boxes: number[];
  goals: number[];
  door: number; // hallway cell the room opens onto
  entry: number; // room cell adjacent to the door
};

// Split the level into rooms: connected components of floor cells that
// are not on the player's (hallway) row.
function findRooms(g: Grid): Room[] {
  const hy = yOf(g, g.player);
  const floor = new Set<number>();
  for (let i = 0; i < g.w * g.h; i++) if (!g.wall[i]) floor.add(i);
  const seen = new Set<number>();
  const rooms: Room[] = [];
  for (const start of floor) {
    const sy = yOf(g, start);
    if (sy === hy || seen.has(start)) continue;
    const cells = new Set<number>();
    const q: number[] = [start];
    seen.add(start);
    for (let head = 0; head < q.length; head++) {
      const cur = q[head];
      cells.add(cur);
      const cy = yOf(g, cur);
      const cx = xOf(g, cur);
      for (const dir of DIRS) {
        const ny = cy + dir.dy;
        const nx = cx + dir.dx;
        if (!inBounds(g, ny, nx)) continue;
        const ni = idx(g, ny, nx);
        if (yOf(g, ni) === hy || !floor.has(ni) || seen.has(ni)) continue;
        seen.add(ni);
        q.push(ni);
      }
    }
    // The door: the unique hallway cell adjacent to a room cell.
    let door = -1;
    let entry = -1;
    for (const c of cells) {
      const cy = yOf(g, c);
      const cx = xOf(g, c);
      for (const dir of DIRS) {
        const ny = cy + dir.dy;
        const nx = cx + dir.dx;
        if (!inBounds(g, ny, nx)) continue;
        const ni = idx(g, ny, nx);
        if (yOf(g, ni) === hy && floor.has(ni) && door === -1) {
          door = ni;
          entry = c;
        }
      }
    }
    if (door === -1) continue;
    const boxes = g.boxes.filter((b) => cells.has(b)).sort((a, b) => a - b);
    const goals = [...g.goals].filter((gl) => cells.has(gl)).sort((a, b) => a - b);
    rooms.push({ cells, boxes, goals, door, entry });
  }
  // Deterministic order: by door column, then entry cell.
  rooms.sort((a, b) => xOf(g, a.door) - xOf(g, b.door) || a.entry - b.entry);
  return rooms;
}

// Cells a box can never leave to reach a goal, by reverse ("pull")
// reachability from the goals ignoring other boxes: a box at q could
// have been pushed there from q-d only when the player could stand at
// q-2d, so predecessors of q are q-d for every direction d with q-d a
// room cell and q-2d a cell the player can stand on (room cell or door).
// A cell outside that reachable set can never hold a box that reaches
// a goal, even with no other boxes around, so it is safely dead.
function deadCells(g: Grid, room: Room): Set<number> {
  const domain = new Set<number>(room.cells);
  domain.add(room.door);
  const live = new Set<number>(room.goals);
  const q: number[] = [...room.goals];
  for (let head = 0; head < q.length; head++) {
    const cur = q[head];
    const cy = yOf(g, cur);
    const cx = xOf(g, cur);
    for (const dir of DIRS) {
      const py = cy - dir.dy;
      const px = cx - dir.dx;
      const qy = cy - 2 * dir.dy;
      const qx = cx - 2 * dir.dx;
      if (!inBounds(g, py, px) || !inBounds(g, qy, qx)) continue;
      const prev = idx(g, py, px);
      const ppos = idx(g, qy, qx);
      if (!room.cells.has(prev) || !domain.has(ppos) || live.has(prev)) continue;
      live.add(prev);
      q.push(prev);
    }
  }
  const dead = new Set<number>();
  for (const c of room.cells) if (!live.has(c)) dead.add(c);
  return dead;
}

type Push = { box: number; dir: number }; // dir index into DIRS

type RoomSolution = { steps: string; endPlayer: number } | null;

// Push-optimal A* for one room. The player starts on the door cell and
// may walk on the room cells plus the door cell; boxes stay in the room.
// When `requireExit` is true, the final state must let the player walk
// back out to the door (needed for every room but the last).
function solveRoom(
  g: Grid,
  room: Room,
  requireExit: boolean,
  nodeCap: number,
  greedy = false,
): RoomSolution {
  const domain = new Set<number>(room.cells);
  domain.add(room.door);
  const goalSet = new Set<number>(room.goals);
  const distFromGoal = room.goals.map((gl) => bfsDist(g, gl, room.cells));
  const dead = deadCells(g, room);
  const startBoxes = [...room.boxes].sort((a, b) => a - b);

  // Binary-heap priority queue, deterministic tie-break by insertion order.
  // Greedy mode ranks by h alone (f = h); otherwise f = g + h (A*).
  type Node = SearchNode & { f: number; g: number; n: number };
  const heap: Node[] = [];
  let counter = 0;
  const heapPush = (nd: Node): void => {
    heap.push(nd);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (heap[p].f < heap[i].f || (heap[p].f === heap[i].f && heap[p].n <= heap[i].n)) break;
      const t = heap[p];
      heap[p] = heap[i];
      heap[i] = t;
      i = p;
    }
  };
  const heapPop = (): Node => {
    const top = heap[0];
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1;
        const r = l + 1;
        let m = i;
        if (l < heap.length && (heap[l].f < heap[m].f || (heap[l].f === heap[m].f && heap[l].n < heap[m].n))) m = l;
        if (r < heap.length && (heap[r].f < heap[m].f || (heap[r].f === heap[m].f && heap[r].n < heap[m].n))) m = r;
        if (m === i) break;
        const t = heap[m];
        heap[m] = heap[i];
        heap[i] = t;
        i = m;
      }
    }
    return top;
  };

  const keyOf = (player: number, boxes: number[]): string =>
    `${player}|${boxes.join(',')}`;
  const best = new Map<string, number>();

  const heuristic = (boxes: number[]): number => {
    for (const b of boxes) if (dead.has(b)) return Infinity;
    return assignmentHeuristic(boxes, room.goals, distFromGoal);
  };

  const h0 = heuristic(startBoxes);
  if (h0 === Infinity) return null;
  heapPush({ f: h0, g: 0, n: counter++, player: room.door, boxes: startBoxes, parent: null, push: null });
  best.set(keyOf(room.door, startBoxes), 0);

  let expansions = 0;
  while (heap.length > 0) {
    const cur = heapPop();
    const curKey = keyOf(cur.player, cur.boxes);
    if (best.get(curKey)! < cur.g) continue;
    // Goal: every box on a goal.
    if (cur.boxes.every((b) => goalSet.has(b))) {
      if (!requireExit) return expandPushes(g, room, cur);
      // Player must be able to walk back to the door.
      const boxSet = new Set<number>(cur.boxes);
      const exit = bfsPath(g, cur.player, room.door, domain, boxSet);
      if (exit !== null) return expandPushes(g, room, cur);
      // Otherwise keep searching: this goal state cannot exit.
    }
    if (++expansions > nodeCap) return null;
    // Cells the player can reach without moving a box (BFS over the
    // domain minus the boxes).
    const boxSet = new Set<number>(cur.boxes);
    const reachSet = new Set<number>();
    {
      const q: number[] = [cur.player];
      reachSet.add(cur.player);
      for (let head = 0; head < q.length; head++) {
        const c = q[head];
        const cy = yOf(g, c);
        const cx = xOf(g, c);
        for (const dir of DIRS) {
          const ny = cy + dir.dy;
          const nx = cx + dir.dx;
          if (!inBounds(g, ny, nx)) continue;
          const ni = idx(g, ny, nx);
          if (!domain.has(ni) || boxSet.has(ni) || reachSet.has(ni)) continue;
          reachSet.add(ni);
          q.push(ni);
        }
      }
    }
    for (let bi = 0; bi < cur.boxes.length; bi++) {
      const b = cur.boxes[bi];
      const by = yOf(g, b);
      const bx = xOf(g, b);
      for (let d = 0; d < DIRS.length; d++) {
        const dir = DIRS[d];
        const fromY = by - dir.dy;
        const fromX = bx - dir.dx;
        const toY = by + dir.dy;
        const toX = bx + dir.dx;
        if (!inBounds(g, fromY, fromX) || !inBounds(g, toY, toX)) continue;
        const from = idx(g, fromY, fromX);
        const to = idx(g, toY, toX);
        if (!reachSet.has(from)) continue;
        if (!room.cells.has(to) || boxSet.has(to)) continue;
        const nb = [...cur.boxes];
        nb[bi] = to;
        nb.sort((a, b2) => a - b2);
        const h = heuristic(nb);
        if (h === Infinity) continue;
        const ng = cur.g + 1;
        const nk = keyOf(b, nb);
        if (best.has(nk) && best.get(nk)! <= ng) continue;
        best.set(nk, ng);
        heapPush({ f: greedy ? h : ng + h, g: ng, n: counter++, player: b, boxes: nb, parent: cur, push: { box: b, dir: d } });
      }
    }
  }
  return null;
}

// Expand a macro-push chain into the player's individual steps. Every
// node stores its box layout; node i>0 carries the push that led to it
// from node i-1, so each push is preceded by the player's BFS walk to
// the pushing cell (boxes of the pre-push layout are obstacles).
type SearchNode = {
  player: number;
  boxes: number[];
  parent: SearchNode | null;
  push: Push | null;
};

function expandPushes(g: Grid, room: Room, goalNode: SearchNode): RoomSolution {
  const nodes: SearchNode[] = [];
  let cur: SearchNode | null = goalNode;
  while (cur !== null) {
    nodes.push(cur);
    cur = cur.parent;
  }
  nodes.reverse();
  const domain = new Set<number>(room.cells);
  domain.add(room.door);
  let steps = '';
  let player = room.door;
  for (let i = 1; i < nodes.length; i++) {
    const node = nodes[i];
    const push = node.push!;
    const box = push.box;
    const dir = DIRS[push.dir];
    const by = yOf(g, box);
    const bx = xOf(g, box);
    const from = idx(g, by - dir.dy, bx - dir.dx);
    const boxSet = new Set<number>(nodes[i - 1].boxes);
    const path = bfsPath(g, player, from, domain, boxSet);
    if (path === null) return null;
    steps += path + dir.ch;
    player = box; // after the push the player stands where the box was
  }
  return { steps, endPlayer: player };
}

function solveLevel(text: string): string {
  const g = parseLevel(text);
  if (g.player < 0 || g.boxes.length === 0) return '';
  const rooms = findRooms(g);
  if (rooms.length === 0) return '';
  // Every box must belong to exactly one room; otherwise fall back to
  // an empty (valid, unscored) answer rather than an illegal one.
  const roomBoxes = rooms.reduce((s, r) => s + r.boxes.length, 0);
  if (roomBoxes !== g.boxes.length) return '';
  const allFloor = new Set<number>();
  for (let i = 0; i < g.w * g.h; i++) if (!g.wall[i]) allFloor.add(i);

  const boxSet = new Set<number>(g.boxes);
  let steps = '';
  let player = g.player;
  for (let ri = 0; ri < rooms.length; ri++) {
    const room = rooms[ri];
    const last = ri === rooms.length - 1;
    // Walk to the door.
    const walkIn = bfsPath(g, player, room.door, allFloor, boxSet);
    if (walkIn === null) return '';
    steps += walkIn;
    player = room.door;
    // Solve the room push-optimally (exit required unless it is last).
    let sol = solveRoom(g, room, !last, 500000);
    if (sol === null && !last) {
      // No exit-preserving optimum: retry without the exit requirement.
      sol = solveRoom(g, room, false, 500000);
    }
    if (sol === null) {
      // Last resort: greedy best-first (f = h) so the level is at least solved.
      sol = solveRoomGreedy(g, room, !last);
    }
    if (sol === null) return '';
    steps += sol.steps;
    player = sol.endPlayer;
    // Boxes of this room are now on their goals.
    for (const b of room.boxes) {
      boxSet.delete(b);
    }
    for (const gl of room.goals) boxSet.add(gl);
    if (!last) {
      const walkOut = bfsPath(g, player, room.door, allFloor, boxSet);
      if (walkOut === null) return '';
      steps += walkOut;
      player = room.door;
    }
  }
  return steps;
}

// Greedy best-first fallback (f = h only) with the same move generation;
// used only if A* exhausts its node cap, so a level still gets solved.
function solveRoomGreedy(g: Grid, room: Room, requireExit: boolean): RoomSolution {
  return solveRoom(g, room, requireExit, 5000000, true);
}

for (const id of readdirSync(I).sort()) {
  let st;
  try {
    st = statSync(join(I, id));
  } catch {
    continue;
  }
  if (!st.isDirectory()) continue;
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
