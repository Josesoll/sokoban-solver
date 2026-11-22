// sokoban-solver: a push-level A* Sokoban solver built to beat a move-level plain A*.
//
// Design, in the order the ideas pay off:
//
// 1. Search over PUSHES, not steps. A state is the sorted box squares plus the canonical
//    representative of the player's reachable region (its lowest square). Walking between
//    pushes is solved exactly by a small BFS whenever a push is generated, so the branching
//    factor is at most 4 * boxes instead of 4 per step, and the cost of a state is exactly
//    the quantity the checker scores: pushes.
//
// 2. Admissible heuristic from precomputed PUSH distances. For every goal we run one reverse
//    BFS (pull a box backwards over open floor) giving, for each square, the minimum pushes
//    to bring a lone box from there to that goal (Infinity when impossible — those squares
//    are exactly the static dead squares). The heuristic is the minimum-cost bipartite
//    matching between boxes and goals under those distances (Hungarian algorithm on a tiny
//    matrix, boxes <= 16). It dominates the naive "sum of nearest-goal distances" because
//    two boxes can never claim the same goal, and it stays admissible because other boxes
//    are ignored. A* with an admissible heuristic returns a push-optimal solution for the
//    (sub)problem it searches.
//
// 3. Deadlock pruning that never prunes a solvable state: (a) a box on a dead square
//    (infinite push distance to every goal) kills the state; (b) a frozen 2x2 block of
//    walls/boxes containing a box off its goal kills the state.
//
// 4. Room decomposition. The generator's levels are a hallway (the player's row) with
//    independent rooms opening off it; a box pushed into the hallway can never come back,
//    so each box belongs to one room forever. We detect the rooms exactly that way: cut the
//    dead squares of the player's row out of the floor, flood-fill what remains, and keep
//    the components that hold boxes. When every component holds as many boxes as goals and
//    no goal is cut off, the level splits into independent subproblems of 3-4 boxes each,
//    solved one at a time with the other rooms' boxes standing as walls (on their goals
//    once solved). Each room is solved push-optimally, so the stitched level solution is
//    push-optimal too. If a level does not decompose, we fall back to searching the whole
//    level with the same engine.
//
// 5. Determinism and the wall budget are handled by WORK, never the clock: every search has
//    a fixed expansion budget, ties in the priority queue break by (f, h, insertion order),
//    and no randomness or time source is read anywhere. Two runs write the same bytes.
//
// If a room (or the whole-level fallback) exhausts its budget we still try to answer: the
// room order is retried once with a doubled budget for the failing room, and only a level
// that still fails gets an empty solution (which is at least format-valid).
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Direction tables: index 0..3 = up, down, left, right.
const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const WALK_CH = ['u', 'd', 'l', 'r'];
const PUSH_CH = ['U', 'D', 'L', 'R'];
const INF = 1e9;

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// Push-distance table: dist[goalIndex][square] = min pushes to move a lone box from
// square to that goal, computed by reverse BFS from the goal (a pull needs the target
// square and the square behind it both open). INF marks dead squares for that goal.
function pushDistances(level: Level): Int32Array[] {
  const { w, wall, goals } = level;
  const size = wall.length;
  return goals.map((g) => {
    const dist = new Int32Array(size).fill(INF);
    dist[g] = 0;
    const queue = [g];
    for (let qi = 0; qi < queue.length; qi++) {
      const cur = queue[qi];
      for (let d = 0; d < 4; d++) {
        const from = cur - (DY[d] * w + DX[d]); // box square before the pull
        const behind = from - (DY[d] * w + DX[d]); // where the player must stand
        if (from < 0 || from >= size || behind < 0 || behind >= size) continue;
        if (wall[from] || wall[behind] || dist[from] !== INF) continue;
        dist[from] = dist[cur] + 1;
        queue.push(from);
      }
    }
    return dist;
  });
}

// Minimum-cost perfect matching between boxes and goals (Hungarian, O(n^3), n <= 16).
// cost[i][j] = push distance of box i to goal j. Returns INF if no finite matching exists.
function matchingCost(cost: number[][]): number {
  const n = cost.length;
  if (n === 0) return 0;
  // Standard O(n^3) Hungarian for a square matrix with potentials u, v.
  const u = new Array<number>(n + 1).fill(0);
  const v = new Array<number>(n + 1).fill(0);
  const p = new Array<number>(n + 1).fill(0); // p[j] = row matched to column j (1-based)
  const way = new Array<number>(n + 1).fill(0);
  for (let i = 1; i <= n; i++) {
    p[0] = i;
    let j0 = 0;
    const minv = new Array<number>(n + 1).fill(Infinity);
    const used = new Array<boolean>(n + 1).fill(false);
    do {
      used[j0] = true;
      const i0 = p[j0];
      let delta = Infinity;
      let j1 = 0;
      for (let j = 1; j <= n; j++) {
        if (used[j]) continue;
        const cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
      }
      for (let j = 0; j <= n; j++) {
        if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
        else minv[j] -= delta;
      }
      j0 = j1;
    } while (p[j0] !== 0);
    do {
      const j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 !== 0);
  }
  let total = 0;
  for (let j = 1; j <= n; j++) {
    if (p[j] === 0) return INF;
    const c = cost[p[j] - 1][j - 1];
    if (c >= INF) return INF;
    total += c;
  }
  return total;
}

// Flood fill of the player's reachable region with boxes as obstacles.
// Fills `seen` with `stamp` and returns the region's lowest square (its canonical name).
function floodRegion(
  level: Level, occupied: Uint8Array, from: number,
  seen: Int32Array, stamp: number, stack: Int32Array,
): number {
  const { w, wall } = level;
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n];
    if (p < low) low = p;
    const q0 = p - w, q1 = p + w, q2 = p - 1, q3 = p + 1;
    if (!wall[q0] && !occupied[q0] && seen[q0] !== stamp) { seen[q0] = stamp; stack[n++] = q0; }
    if (!wall[q1] && !occupied[q1] && seen[q1] !== stamp) { seen[q1] = stamp; stack[n++] = q1; }
    if (!wall[q2] && !occupied[q2] && seen[q2] !== stamp) { seen[q2] = stamp; stack[n++] = q2; }
    if (!wall[q3] && !occupied[q3] && seen[q3] !== stamp) { seen[q3] = stamp; stack[n++] = q3; }
  }
  return low;
}

// Is the box just pushed to `t` part of a frozen 2x2 of walls/boxes with a box off-goal?
function frozenBlock(level: Level, occupied: Uint8Array, goalSet: Uint8Array, t: number): boolean {
  const { w, wall } = level;
  const corners = [t, t - 1, t - w, t - w - 1];
  for (const a of corners) {
    const b = a + 1, c = a + w, d = c + 1;
    if (a < 0 || d >= wall.length) continue;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b]) ||
        !(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goalSet[a]) || (occupied[b] && !goalSet[b]) ||
        (occupied[c] && !goalSet[c]) || (occupied[d] && !goalSet[d])) return true;
  }
  return false;
}

// Shortest walk from `from` to `to` avoiding walls and boxes, as lower-case LURD.
// BFS with directions tried in fixed order, so the result is deterministic.
function walkPath(level: Level, occupied: Uint8Array, from: number, to: number): string {
  if (from === to) return '';
  const { w, wall } = level;
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Int32Array(wall.length).fill(0);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    if (p === to) break;
    for (let d = 0; d < 4; d++) {
      const q = p + DY[d] * w + DX[d];
      if (q < 0 || q >= wall.length || wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = d;
      queue.push(q);
    }
  }
  if (prev[to] === -1) return '';
  let s = '';
  for (let p = to; p !== from; p = prev[p]) s = WALK_CH[how[p]] + s;
  return s;
}

interface Solved { pushes: number; moves: string }

// Push-level A* over one (sub)level. Returns a push-optimal solution, or null when the
// expansion budget runs out or the position is provably dead.
function solveAStar(level: Level, maxExpansions: number): Solved | null {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goalSet = new Uint8Array(size);
  for (const g of goals) goalSet[g] = 1;
  const dists = pushDistances(level);
  const nB = level.boxes.length;
  if (nB !== goals.length) return null;

  // Heuristic for a box set: min-cost matching under the push-distance tables.
  const costBuf: number[][] = Array.from({ length: nB }, () => new Array<number>(goals.length).fill(0));
  const heuristic = (boxes: number[]): number => {
    for (let i = 0; i < nB; i++) {
      for (let j = 0; j < goals.length; j++) costBuf[i][j] = dists[j][boxes[i]];
    }
    return matchingCost(costBuf);
  };
  const h0 = heuristic(level.boxes);
  if (h0 >= INF) return null;
  const isSolved = (boxes: number[]): boolean => boxes.every((b) => goalSet[b] === 1);

  // State storage in parallel arrays; a state records its parent and the single push
  // (box-from square, direction) that created it, so the move string is rebuilt by
  // replaying walks along the parent chain at the end.
  const stBoxes: number[][] = [level.boxes.slice()];
  const stPlayer: number[] = [level.player]; // player square right before the push (box-from after)
  const stParent: number[] = [-1];
  const stPushFrom: number[] = [-1];
  const stPushDir: number[] = [-1];
  const stG: number[] = [0];

  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  const keyOf = (boxes: number[], low: number): string =>
    low + '|' + boxes.join(',');

  for (const b of level.boxes) occupied[b] = 1;
  const startLow = floodRegion(level, occupied, level.player, seen, ++stamp, stack);
  for (const b of level.boxes) occupied[b] = 0;

  const best = new Map<string, number>();
  best.set(keyOf(level.boxes, startLow), 0);

  // Binary heap ordered by (f asc, h asc, insertion order asc): fully deterministic.
  interface Node { f: number; h: number; ord: number; s: number }
  const heap: Node[] = [];
  let ordCounter = 0;
  const less = (a: Node, b: Node): boolean =>
    a.f < b.f || (a.f === b.f && (a.h < b.h || (a.h === b.h && a.ord < b.ord)));
  const heapPush = (nd: Node): void => {
    heap.push(nd);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      const t = heap[i]; heap[i] = heap[p]; heap[p] = t;
      i = p;
    }
  };
  const heapPop = (): Node => {
    const top = heap[0];
    const last = heap.pop() as Node;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        const t = heap[i]; heap[i] = heap[c]; heap[c] = t;
        i = c;
      }
    }
    return top;
  };
  heapPush({ f: h0, h: h0, ord: ordCounter++, s: 0 });

  if (isSolved(level.boxes)) return { pushes: 0, moves: '' };

  let found = -1;
  let expansions = 0;
  while (heap.length > 0) {
    const cur = heapPop();
    const s = cur.s;
    const boxes = stBoxes[s];
    const g = stG[s];
    // Skip stale heap entries: a cheaper route to this state was already expanded.
    for (const b of boxes) occupied[b] = 1;
    const low = floodRegion(level, occupied, stPlayer[s], seen, ++stamp, stack);
    const regionStamp = stamp;
    if (best.get(keyOf(boxes, low)) !== g) {
      for (const b of boxes) occupied[b] = 0;
      continue;
    }
    if (expansions >= maxExpansions) {
      for (const b of boxes) occupied[b] = 0;
      return null;
    }
    expansions++;

    // Generate every legal push: box b in direction d needs the player on b-d
    // (inside the current region) and the target b+d free and not a dead square.
    // Order candidates by the pushed box's best remaining distance, then by square
    // and direction, so equal-f nodes surface in a stable, goal-directed order.
    const cand: Array<{ bi: number; d: number; t: number; rank: number }> = [];
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi];
      for (let d = 0; d < 4; d++) {
        const step = DY[d] * w + DX[d];
        const t = b + step;
        if (seen[b - step] !== regionStamp) continue;
        if (t < 0 || t >= size || wall[t] || occupied[t]) continue;
        let rank = INF;
        for (let j = 0; j < goals.length; j++) rank = Math.min(rank, dists[j][t]);
        if (rank >= INF) continue; // dead square for every goal
        cand.push({ bi, d, t, rank });
      }
    }
    cand.sort((a, b2) => a.rank - b2.rank || a.t - b2.t || a.d - b2.d);

    for (const { bi, d, t } of cand) {
      const b = boxes[bi];
      const step = DY[d] * w + DX[d];
      occupied[b] = 0;
      occupied[t] = 1;
      if (frozenBlock(level, occupied, goalSet, t)) {
        occupied[t] = 0; occupied[b] = 1;
        continue;
      }
      const next = boxes.slice();
      next[bi] = t;
      next.sort((a, b2) => a - b2);
      const nextLow = floodRegion(level, occupied, b, mark, ++stamp, stack);
      occupied[t] = 0;
      occupied[b] = 1;
      const key = keyOf(next, nextLow);
      const ng = g + 1;
      const prevBest = best.get(key);
      if (prevBest !== undefined && prevBest <= ng) continue;
      const h = heuristic(next);
      if (h >= INF) continue;
      best.set(key, ng);
      const ns = stBoxes.length;
      stBoxes.push(next);
      stPlayer.push(b); // after the push the player stands where the box was
      stParent.push(s);
      stPushFrom.push(b);
      stPushDir.push(d);
      stG.push(ng);
      heapPush({ f: ng + h, h, ord: ordCounter++, s: ns });
      if (isSolved(next)) { found = ns; heap.length = 0; break; }
    }
    for (const b of boxes) occupied[b] = 0;
    if (found >= 0) break;
  }
  if (found < 0) return null;

  // Rebuild the move string: replay the push chain, walking the player to each
  // push stance with a BFS over the box layout of that moment.
  const chain: number[] = [];
  for (let s = found; s > 0; s = stParent[s]) chain.push(s);
  chain.reverse();
  let moves = '';
  let player = level.player;
  let boxes = level.boxes.slice();
  const occ = new Uint8Array(size);
  for (const s of chain) {
    const from = stPushFrom[s];
    const d = stPushDir[s];
    const step = DY[d] * w + DX[d];
    for (const b of boxes) occ[b] = 1;
    moves += walkPath(level, occ, player, from - step);
    for (const b of boxes) occ[b] = 0;
    moves += PUSH_CH[d];
    player = from;
    boxes = stBoxes[s].slice();
  }
  return { pushes: chain.length, moves };
}

// Room decomposition: cut the dead squares of the player's row (the hallway) out of
// the floor and flood-fill the rest. Returns the components holding boxes, or null
// when the level is not a clean set of rooms.
interface Room { squares: number[]; boxes: number[]; goals: number[] }
function roomsOf(level: Level): Room[] | null {
  const { w, wall, goals, boxes, player } = level;
  const dists = pushDistances(level);
  const live = new Uint8Array(wall.length);
  for (let i = 0; i < wall.length; i++) {
    for (let j = 0; j < goals.length; j++) {
      if (dists[j][i] < INF) { live[i] = 1; break; }
    }
  }
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let x = 0; x < w; x++) {
    const i = row * w + x;
    if (!wall[i] && !live[i]) cut[i] = 1;
  }
  if (boxes.some((b) => cut[b] === 1)) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms: Room[] = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      const p = squares[i];
      const qs = [p - w, p + w, p - 1, p + 1];
      for (const q of qs) {
        if (q < 0 || q >= wall.length || wall[q] || cut[q] || part[q] >= 0) continue;
        part[q] = rooms.length;
        squares.push(q);
      }
    }
    rooms.push({ squares, boxes: [], goals: [] });
  }
  for (const b of boxes) rooms[part[b]].boxes.push(b);
  for (const g of goals) {
    if (part[g] < 0) return null;
    rooms[part[g]].goals.push(g);
  }
  if (rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

function endPosition(w: number, from: number, moves: string): number {
  let p = from;
  for (const ch of moves.toLowerCase()) {
    p += ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
  }
  return p;
}

// Solve a whole level: room by room when it decomposes (each room push-optimal,
// other rooms' boxes as walls), else one whole-level A* search.
function solveLevel(text: string): string {
  const level = parseLevel(text);
  const rooms = roomsOf(level);
  if (rooms === null || rooms.length <= 1) {
    const whole = solveAStar(level, 1_500_000);
    return whole ? whole.moves : '';
  }
  let player = level.player;
  let moves = '';
  const doneGoals: number[] = [];
  const pendingBoxes: number[] = rooms.flatMap((r) => r.boxes);
  for (const room of rooms) {
    const wall = level.wall.slice();
    for (const g of doneGoals) wall[g] = 1;
    for (const b of pendingBoxes) if (!room.boxes.includes(b)) wall[b] = 1;
    const sub: Level = {
      w: level.w, h: level.h, wall,
      goals: room.goals.slice(), boxes: room.boxes.slice(), player,
    };
    const solved = solveAStar(sub, 600_000) ?? solveAStar(sub, 2_400_000);
    if (solved === null) {
      // Decomposition failed on this room; fall back to the whole level once.
      const whole = solveAStar(level, 1_500_000);
      return whole ? whole.moves : '';
    }
    moves += solved.moves;
    player = endPosition(level.w, player, solved.moves);
    for (const b of room.boxes) pendingBoxes.splice(pendingBoxes.indexOf(b), 1);
    doneGoals.push(...room.goals);
  }
  return moves;
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
