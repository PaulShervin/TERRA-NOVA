// Frontier-based autonomous exploration: pick the best reachable frontier, plan, drive, repeat until none remain.
import { buildTraversable, dijkstra, findFrontiers, extractPath, smoothPath } from './Planner.js';

const INFLATION = 0.5; // m, robot collision radius used for planning (scaled robot, half-diagonal ~0.55 m)
const REPLAN_EVERY = 1.5; // s of sim time
const STUCK_CHECK = 4; // s
const BLACKLIST_RADIUS = 1.5; // m
const MIN_GOAL_DIST = 0.6; // m

export function createExplorer({ grid, controller, truth, home, inflations = [INFLATION, 0.4, 0.3], sizeWeight = 0.12 }) {
  let t = 0, sinceReplan = REPLAN_EVERY, sinceStuck = 0;
  let lastPos = [controller.pose.x, controller.pose.y];
  let goal = null; // world [x,y]
  let finishing = false; // true once exploration stopped paying off: head home instead of chasing leftovers
  let covBest = 0, covBestAt = 0;
  let goalSince = 0; // sim time the current goal was adopted
  let state = 'exploring'; // exploring | returning | complete
  let pathWorld = [];
  let debugInfo = null; // last planning snapshot, used by the in-app algorithm guide
  const blacklist = []; // { x, y, until }
  const status = { state, coverage: 0, goal: null, frontiers: 0, travelled: 0 };
  let covTimer = 1;
  let prev = [controller.pose.x, controller.pose.y];

  function coverage() {
    let hit = 0;
    for (let k = 0; k < truth.truth.length; k++) if (truth.truth[k] && grid.stateAt(k) === 1) hit++;
    return hit / truth.count;
  }

  function nearestTraversable(trav, cell) {
    for (let r = 0; r < 8; r++) {
      for (let dj = -r; dj <= r; dj++) for (let di = -r; di <= r; di++) {
        const i = cell[0] + di, j = cell[1] + dj;
        if (grid.inside(i, j) && trav[grid.idx(i, j)]) return [i, j];
      }
    }
    return null;
  }

  /** Plan with a given inflation. Returns true when a route (to a frontier or home) was issued. */
  function tryPlan(inflation) {
    const { x, y } = controller.pose;
    const trav = buildTraversable(grid, inflation, [x, y]);
    const start = nearestTraversable(trav, grid.toCell(x, y));
    if (!start) return false;
    const { cost, parent } = dijkstra(grid, trav, start);
    const clusters = findFrontiers(grid, trav);
    status.frontiers = clusters.length;

    const live = blacklist.filter((bl) => bl.until > t);
    const candidates = [];
    for (const cells of clusters) {
      let best = -1, bestCost = Infinity;
      for (const k of cells) if (cost[k] < bestCost) { bestCost = cost[k]; best = k; }
      if (best < 0 || !Number.isFinite(bestCost)) continue;
      const [wx, wy] = grid.toWorld(best % grid.W, (best / grid.W) | 0);
      if (live.some((bl) => Math.hypot(bl.x - wx, bl.y - wy) < BLACKLIST_RADIUS)) continue;
      let score = bestCost - sizeWeight * Math.min(cells.length, 40);
      if (goal && Math.hypot(goal[0] - wx, goal[1] - wy) < 1.5) score -= 60; // commit to the current target (avoids flip-flopping)
      candidates.push({ k: best, wx, wy, bestCost, score });
    }
    const usable = candidates.filter((c) => c.bestCost > MIN_GOAL_DIST);
    const pick = finishing ? undefined : (usable.length ? usable : candidates).sort((c1, c2) => c1.score - c2.score)[0];

    debugInfo = { trav, cost, clusters, goalCell: pick ? pick.k : null, inflation, candidates: candidates.length };
    if (pick) {
      state = 'exploring';
      if (!goal || Math.hypot(goal[0] - pick.wx, goal[1] - pick.wy) > 1.5) goalSince = t;
      goal = [pick.wx, pick.wy];
      status.goal = goal;
      pathWorld = smoothPath(grid, trav, extractPath(grid, parent, pick.k)).map(([i, j]) => grid.toWorld(i, j));
      controller.setRoute(pathWorld);
      return true;
    }
    return goHome(trav, cost, parent);
  }

  /** Drive to (the nearest traversable cell to) the start pose. Returns false when home is unreachable. */
  function goHome(trav, cost, parent) {
    state = 'returning';
    // Noise can make the exact home cell look blocked, so aim for the nearest traversable cell instead.
    const target = nearestTraversable(trav, grid.toCell(home[0], home[1]));
    const hk = target ? grid.idx(target[0], target[1]) : -1;
    if (Math.hypot(controller.pose.x - home[0], controller.pose.y - home[1]) < 0.9) { finish(); return true; }
    if (hk < 0 || !Number.isFinite(cost[hk])) return false;
    pathWorld = smoothPath(grid, trav, extractPath(grid, parent, hk)).map(([i, j]) => grid.toWorld(i, j));
    controller.setRoute(pathWorld);
    return true;
  }

  function finish() {
    state = 'complete'; controller.setRoute([]); pathWorld = []; goal = null; status.goal = null;
  }

  /** Try the normal safety margin first, then progressively smaller ones (to escape tight / noisy pockets). */
  function plan() {
    // A goal we already reached whose frontier is STILL there cannot be resolved (e.g. unseen space behind rock): give up on it.
    if (goal && Math.hypot(controller.pose.x - goal[0], controller.pose.y - goal[1]) < 1.2) {
      blacklist.push({ x: goal[0], y: goal[1], until: t + 600 });
      goal = null; status.goal = null;
    }
    if (goal && t - goalSince > 60) { // taking too long: give up on this goal for a while
      blacklist.push({ x: goal[0], y: goal[1], until: t + 300 });
      goal = null; status.goal = null;
    }
    for (const inflation of inflations) if (tryPlan(inflation)) return;
    finish();
  }

  function update(dt) {
    t += dt; sinceReplan += dt; sinceStuck += dt; covTimer += dt;
    status.travelled += Math.hypot(controller.pose.x - prev[0], controller.pose.y - prev[1]);
    prev = [controller.pose.x, controller.pose.y];
    if (covTimer >= 1) {
      status.coverage = coverage(); covTimer = 0;
      if (status.coverage > covBest + 0.003) { covBest = status.coverage; covBestAt = t; }
      else if (!finishing && t - covBestAt > 150) finishing = true; // < 0.3 % new coverage in 150 s of sim time
    }
    if (state === 'complete') { status.state = state; return; }

    if (sinceStuck >= STUCK_CHECK) {
      const moved = Math.hypot(controller.pose.x - lastPos[0], controller.pose.y - lastPos[1]);
      if (moved < 0.25 && !controller.isDone() && goal) { blacklist.push({ x: goal[0], y: goal[1], until: t + 60 }); sinceReplan = REPLAN_EVERY; }
      lastPos = [controller.pose.x, controller.pose.y]; sinceStuck = 0;
    }
    if (sinceReplan >= REPLAN_EVERY || (controller.isDone() && sinceReplan > 0.3)) { plan(); sinceReplan = 0; }
    status.state = state;
  }

  return { update, status, getDebug: () => debugInfo, getPath: () => pathWorld, getGoal: () => goal };
}
