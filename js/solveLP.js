/**
 * DCU site and relay-set planning for Bluetooth Mesh smart metering (Joint / Proposed MILP),
 * solved with HiGHS (highs.js). Port of Part 1.5-1.6 and Part 2 of
 * notebooks/dcu_relay_planning.ipynb.
 *
 * Model, for each planning unit with meters N and candidate poles C (see the notebook, 2.1):
 *
 *   min  sum_j |A(j)| r_j + eps * sum_i h_i,           H = min(|N|, ttlMax), eps = 1 / (H |N| + 1)
 *   s.t. sum_c y_c = 1                                   one DCU per unit
 *        u_ic <= y_c                  for c, i in D(c)   direct links only to the chosen pole
 *        sum_{j in A(i)} x_ij + sum_{c: i in D(c)} u_ic = 1   exactly one parent per meter
 *        x_ij <= r_j                  for i, j in A(i)   a meter parent must relay
 *        h_i >= h_j + 1 - H (1 - x_ij)                   hop propagation, no cycles
 *        y, r, x, u binary;  h_i in [1, H]
 *
 * A(i): meters within radio range of meter i. D(c): meters within radio range of pole c.
 * The first objective term counts redundant rebroadcasts; it is an integer and the hop term is
 * below 1, so the hop count only breaks ties.
 *
 * Planning units: meters are grouped into connected components of the meter-meter graph (the
 * DCU receives but never rebroadcasts, so it does not connect groups). Groups sharing a
 * candidate pole are merged into one unit served by one DCU, whose candidates are the poles in
 * range of every group of the unit; a chain of groups with no common pole is split greedily
 * (largest group first, ties by smallest meter index). Groups with no pole in range are
 * uncovered and are not planned.
 */
import { getHighs } from "./highs.js";

const DEFAULT_OPTIONS = Object.freeze({
  ttlMax: 127, // Bluetooth Mesh maximum TTL; caps the hop count H
  timeLimit: 600, // seconds per planning unit (HiGHS time_limit)
  mipRelGap: 1e-6, // HiGHS mip_rel_gap
  randomSeed: 20240501, // HiGHS random_seed
});

const TERMS_PER_LINE = 8; // keep LP-format lines short

/**
 * Plan DCU sites and relays by solving the Joint MILP with HiGHS.
 *
 * Note on the distance convention: 0 means "out of radio range". Two meters at identical
 * coordinates (e.g. mounted on the same pole) are 0 m apart but in range; pass a small positive
 * value (e.g. 1e-6) for such pairs, otherwise they are treated as unable to hear each other.
 *
 * @param {{x: number, y: number}[]} meters Meters (coordinates; only the count is used here).
 * @param {{x: number, y: number}[]} candidates Candidate DCU poles (coordinates; only the count is used).
 * @param {number[][]} distances Square matrix of size meters.length + candidates.length.
 *   Indices 0..M-1 are meters, M..M+C-1 are candidates (M = meters.length, C = candidates.length).
 *   Entry = Euclidean distance when in radio range, 0 when out of range.
 *   Candidate-candidate entries are ignored.
 * @param {{ttlMax?: number, timeLimit?: number, mipRelGap?: number, randomSeed?: number}} [options]
 * @returns {Promise<{
 *   status: string,
 *   dcus: number[],
 *   totalHops: number,
 *   totalDistance: number,
 *   nodes: {meterIndex: number, parentIndex: number|null, dcuIndex: number|null}[]
 * }>}
 *   status: HiGHS model status ("Optimal", "Time limit reached", ...). With several planning
 *     units it is "Optimal" only if every unit is optimal, otherwise the first other status.
 *     "No planning unit" if no meter can reach any candidate.
 *   dcus: candidate indices (0..C-1) chosen as DCU sites, one per planning unit.
 *   totalHops: sum over meters of the hop count from the meter to its DCU along its parents.
 *   totalDistance: sum of the lengths of all parent links (meter-meter and meter-DCU).
 *   nodes: one entry per meter. parentIndex is the parent's meter index, or null if the meter
 *     sends directly to its DCU. dcuIndex is the candidate index of its DCU, or null if the meter
 *     is uncovered (no path to any candidate) or its unit returned no usable solution.
 */
export async function solveLP(meters, candidates, distances, options = {}) {
  const opts = { ...DEFAULT_OPTIONS, ...options };
  const M = meters.length;
  const C = candidates.length;
  validateInputs(meters, candidates, distances);

  const { A, D } = buildAdjacency(distances, M, C);
  const units = buildPlanningUnits(A, D, M);

  const nodes = Array.from({ length: M }, (_, i) => ({ meterIndex: i, parentIndex: null, dcuIndex: null }));
  const dcus = [];
  const statuses = [];
  let totalHops = 0;
  let totalDistance = 0;

  const highs = units.length ? await getHighs() : null;
  for (const unit of units) {
    const model = buildUnitModel(unit, A, D, opts.ttlMax);
    const res = highs.solve(model.lp, {
      time_limit: opts.timeLimit,
      mip_rel_gap: opts.mipRelGap,
      random_seed: opts.randomSeed,
    });
    statuses.push(res.Status);
    const plan = extractPlan(unit, model, res);
    if (plan === null) continue; // no usable solution (e.g. time limit before a feasible point)

    dcus.push(plan.dcu);
    unit.meters.forEach((g, i) => {
      const p = plan.parent[i];
      nodes[g] = { meterIndex: g, parentIndex: p < 0 ? null : unit.meters[p], dcuIndex: plan.dcu };
      totalHops += plan.depth[i];
      totalDistance += p < 0 ? distances[g][M + plan.dcu] : distances[g][unit.meters[p]];
    });
  }

  return { status: combineStatuses(statuses), dcus, totalHops, totalDistance, nodes };
}

/** Throw if the inputs are not a consistent square, non-negative, symmetric-in-range matrix. */
function validateInputs(meters, candidates, distances) {
  if (!Array.isArray(meters) || !Array.isArray(candidates) || !Array.isArray(distances)) {
    throw new TypeError("meters, candidates and distances must be arrays");
  }
  const n = meters.length + candidates.length;
  if (distances.length !== n || distances.some((row) => !Array.isArray(row) || row.length !== n)) {
    throw new RangeError(`distances must be a ${n} x ${n} matrix (meters + candidates)`);
  }
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      const d = distances[i][j];
      if (!Number.isFinite(d) || d < 0) {
        throw new RangeError(`distances[${i}][${j}] = ${d} must be a finite non-negative number`);
      }
      const inRange = d > 0;
      const back = distances[j][i] > 0;
      const bothCandidates = i >= meters.length && j >= meters.length;
      if (!bothCandidates && inRange !== back) {
        throw new RangeError(`distances is not symmetric in range: [${i}][${j}] vs [${j}][${i}]`);
      }
    }
  }
}

/** A[i]: meters in range of meter i; D[c]: meters in range of candidate c (sorted indices). */
function buildAdjacency(distances, M, C) {
  const A = Array.from({ length: M }, (_, i) => {
    const nb = [];
    for (let j = 0; j < M; j++) if (j !== i && distances[i][j] > 0) nb.push(j);
    return nb;
  });
  const D = Array.from({ length: C }, (_, c) => {
    const inRange = [];
    for (let i = 0; i < M; i++) if (distances[i][M + c] > 0) inRange.push(i);
    return inRange;
  });
  return { A, D };
}

/**
 * Connected groups of the meter graph, merged into planning units that share a candidate.
 * @returns {{meters: number[], candidates: number[]}[]} global meter and candidate indices.
 */
function buildPlanningUnits(A, D, M) {
  // Groups: BFS over meter-meter links, in order of the smallest member.
  const groupOf = new Int32Array(M).fill(-1);
  const groups = [];
  for (let s = 0; s < M; s++) {
    if (groupOf[s] >= 0) continue;
    const members = [s];
    groupOf[s] = groups.length;
    for (let q = 0; q < members.length; q++) {
      for (const j of A[members[q]]) {
        if (groupOf[j] < 0) {
          groupOf[j] = groups.length;
          members.push(j);
        }
      }
    }
    groups.push(members.sort((a, b) => a - b));
  }

  // Candidate set of each group: poles in range of at least one of its meters.
  const groupCands = groups.map(() => new Set());
  D.forEach((inRange, c) => inRange.forEach((i) => groupCands[groupOf[i]].add(c)));

  // Union-find over covered groups that share a pole.
  const covered = groups.map((_, g) => g).filter((g) => groupCands[g].size > 0);
  const parent = new Map(covered.map((g) => [g, g]));
  const find = (g) => {
    while (parent.get(g) !== g) {
      parent.set(g, parent.get(parent.get(g)));
      g = parent.get(g);
    }
    return g;
  };
  const firstGroupOfPole = new Map();
  for (const g of covered) {
    for (const c of groupCands[g]) {
      if (firstGroupOfPole.has(c)) parent.set(find(g), find(firstGroupOfPole.get(c)));
      else firstGroupOfPole.set(c, g);
    }
  }
  const components = new Map();
  for (const g of covered) {
    const root = find(g);
    if (!components.has(root)) components.set(root, []);
    components.get(root).push(g);
  }

  // Split each component so that its groups share at least one pole (one DCU per unit).
  const units = [];
  for (const comp of components.values()) {
    const order = [...comp].sort((a, b) => groups[b].length - groups[a].length || groups[a][0] - groups[b][0]);
    while (order.length) {
      const members = [order.shift()];
      let common = new Set(groupCands[members[0]]);
      for (let k = 0; k < order.length; ) {
        const g = order[k];
        const shared = new Set([...common].filter((c) => groupCands[g].has(c)));
        if (shared.size) {
          common = shared;
          members.push(g);
          order.splice(k, 1);
        } else {
          k++;
        }
      }
      units.push({
        meters: members.flatMap((g) => groups[g]).sort((a, b) => a - b),
        candidates: [...common].sort((a, b) => a - b),
      });
    }
  }
  return units.sort((a, b) => a.meters[0] - b.meters[0]);
}

/**
 * Write the MILP of one unit in CPLEX LP format.
 * Variables: y{k} (DCU at unit candidate k), r{i} (meter i relays), x{i}_{j} (parent of i is j),
 * u{i}_{k} (i sends directly to the DCU at k), h{i} (hops); i, j, k are local unit indices.
 */
function buildUnitModel(unit, A, D, ttlMax) {
  const N = unit.meters.length;
  const local = new Map(unit.meters.map((g, i) => [g, i]));
  const Aloc = unit.meters.map((g) => A[g].map((j) => local.get(j))); // groups are closed under A
  const Dloc = unit.candidates.map((c) => D[c].filter((g) => local.has(g)).map((g) => local.get(g)));
  const H = Math.min(N, ttlMax);
  const eps = 1 / (H * N + 1);

  const objective = [];
  const rows = [];
  const binaries = [];
  const xPairs = [];
  const uPairs = [];

  unit.candidates.forEach((_, k) => binaries.push(`y${k}`));
  for (let j = 0; j < N; j++) {
    if (Aloc[j].length === 0) continue; // a meter without neighbours can never be a parent
    objective.push([Aloc[j].length, `r${j}`]);
    binaries.push(`r${j}`);
  }
  for (let i = 0; i < N; i++) objective.push([eps, `h${i}`]);

  rows.push([unit.candidates.map((_, k) => [1, `y${k}`]), "=", 1]);
  const parents = Array.from({ length: N }, () => []);
  Dloc.forEach((inRange, k) => {
    for (const i of inRange) {
      const u = `u${i}_${k}`;
      uPairs.push({ i, k, name: u });
      binaries.push(u);
      parents[i].push([1, u]);
      rows.push([[[1, u], [-1, `y${k}`]], "<=", 0]);
    }
  });
  for (let i = 0; i < N; i++) {
    for (const j of Aloc[i]) {
      const x = `x${i}_${j}`;
      xPairs.push({ i, j, name: x });
      binaries.push(x);
      parents[i].push([1, x]);
      rows.push([[[1, x], [-1, `r${j}`]], "<=", 0]);
      rows.push([[[1, `h${i}`], [-1, `h${j}`], [-H, x]], ">=", 1 - H]);
    }
  }
  parents.forEach((terms) => rows.push([terms, "=", 1]));

  const lines = ["Minimize", ` obj: ${formatTerms(objective)}`, "Subject To"];
  rows.forEach(([terms, sense, rhs], r) => lines.push(` c${r}: ${formatTerms(terms)} ${sense} ${rhs}`));
  lines.push("Bounds");
  for (let i = 0; i < N; i++) lines.push(` 1 <= h${i} <= ${H}`);
  lines.push("Binary");
  for (let s = 0; s < binaries.length; s += TERMS_PER_LINE) lines.push(` ${binaries.slice(s, s + TERMS_PER_LINE).join(" ")}`);
  lines.push("End");
  return { lp: lines.join("\n"), xPairs, uPairs, N, C: unit.candidates.length };
}

/** "+ 3 a - 1 b ..." with a line break every TERMS_PER_LINE terms. */
function formatTerms(terms) {
  const parts = terms.map(([coef, name]) => `${coef < 0 ? "-" : "+"} ${Math.abs(coef)} ${name}`);
  const chunks = [];
  for (let s = 0; s < parts.length; s += TERMS_PER_LINE) chunks.push(parts.slice(s, s + TERMS_PER_LINE).join(" "));
  return chunks.join("\n   ");
}

/**
 * Read the DCU, parents and hop depths of one unit from a HiGHS result and check that they form
 * a valid tree rooted at the DCU with relay parents. Returns null if they do not (no solution).
 */
function extractPlan(unit, model, res) {
  const cols = res.Columns ?? {};
  const val = (name) => cols[name]?.Primal ?? 0;
  let dcuK = -1;
  for (let k = 0; k < model.C; k++) if (val(`y${k}`) > 0.5) dcuK = k;
  if (dcuK < 0) return null;

  const parent = new Array(model.N).fill(-2); // -2 = no parent, -1 = DCU
  const count = new Array(model.N).fill(0);
  for (const { i, j, name } of model.xPairs) {
    if (val(name) > 0.5) {
      if (val(`r${j}`) <= 0.5) return null; // parent must be a relay
      parent[i] = j;
      count[i]++;
    }
  }
  for (const { i, k, name } of model.uPairs) {
    if (val(name) > 0.5) {
      if (k !== dcuK) return null; // direct link to a pole without DCU
      parent[i] = -1;
      count[i]++;
    }
  }
  if (count.some((n) => n !== 1)) return null;

  // Depth along parent pointers; a cycle means the point is not a valid solution.
  const depth = new Array(model.N).fill(0);
  for (let i = 0; i < model.N; i++) {
    const path = [];
    let v = i;
    while (v >= 0 && depth[v] === 0) {
      path.push(v);
      v = parent[v];
      if (path.length > model.N) return null;
    }
    let d = v < 0 ? 0 : depth[v];
    for (let p = path.length - 1; p >= 0; p--) depth[path[p]] = ++d;
  }
  return { dcu: unit.candidates[dcuK], parent, depth };
}

/** "Optimal" if every unit is optimal, otherwise the first other status. */
function combineStatuses(statuses) {
  if (statuses.length === 0) return "No planning unit";
  return statuses.find((s) => s !== "Optimal") ?? "Optimal";
}
