"""Link length, link margin and relay load for the plans in a `<site>_plans.json` file.

Usage:
    python src/plan_link_metrics.py Site1_plans.json [--out-dir DIR] [--methods NAME ...]

For every method and planning unit it reports:
  1. The length of every parent link, split by role:
       access   = non-relay meter -> its parent (a relay or the DCU)
       backbone = relay           -> its parent (the next relay or the DCU)
  2. The link margin of every link, and for each meter the weakest link on its path to the DCU.
       margin_db          = P_rx - P_sens                     (raw margin)
       margin_over_fade   = P_rx - P_sens - fade_margin       (>= 0 exactly when d <= d_max)
     with P_rx = P_eirp - PL(d), PL(d) = PL(d0) + 10 n log10(d / d0), the model the planner uses.
  3. For each relay, the number of meters really within d_max of it and its number of children
     in the plan.

In a plan, a parent is a pole id when `parent_is_dcu` is true and a meter id otherwise; meter and
pole ids share the same numbers, so the two are looked up in separate tables.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def path_loss_db(d_m: float, p: dict) -> float:
    d = max(d_m, p["d0_m"])  # co-located devices: clamp to the reference distance
    return p["pl_d0_db"] + 10.0 * p["path_loss_exponent"] * math.log10(d / p["d0_m"])


def link_margin_db(d_m: float, p: dict) -> float:
    return p["p_eirp_dbm"] - path_loss_db(d_m, p) - p["p_sens_dbm"]


def analyse_unit(method: str, unit: dict, meter_xy: dict, pole_xy: dict, p: dict):
    d_max, fade = p["d_max_m"], p["fade_margin_db"]
    dcu = unit["dcu_pole_id"]
    plan = {m["id"]: m for m in unit["meters"]}
    relays = set(unit["relay_ids"])

    # 1 + 2: one row per link (meter -> parent)
    links = {}
    for mid, m in plan.items():
        parent_xy = pole_xy[m["parent"]] if m["parent_is_dcu"] else meter_xy[m["parent"]]
        d = math.dist(meter_xy[mid], parent_xy)
        margin = link_margin_db(d, p)
        links[mid] = {
            "method": method, "unit": unit["unit"], "child": mid,
            "parent": f"DCU pole {m['parent']}" if m["parent_is_dcu"] else f"relay {m['parent']}",
            "role": "backbone" if mid in relays else "access",
            "length_m": round(d, 3),
            "margin_db": round(margin, 2),
            "margin_over_fade_db": round(margin - fade, 2),
            "within_d_max": d <= d_max + 1e-9,
        }

    # 2: weakest link on each meter's path to the DCU
    paths = []
    for mid in plan:
        hops, node = [], mid
        while True:
            hops.append(links[node])
            if plan[node]["parent_is_dcu"]:
                break
            node = plan[node]["parent"]
            if len(hops) > p["ttl_max"]:
                raise ValueError(f"{method}: parent loop at meter {mid}")
        weakest = min(hops, key=lambda r: r["margin_db"])
        paths.append({
            "method": method, "unit": unit["unit"], "meter": mid,
            "is_relay": mid in relays, "hops": len(hops),
            "path": " -> ".join([mid] + [h["parent"] for h in hops]),
            "path_length_m": round(sum(h["length_m"] for h in hops), 3),
            "weakest_link": f"{weakest['child']} -> {weakest['parent']}",
            "weakest_role": weakest["role"],
            "weakest_length_m": weakest["length_m"],
            "weakest_margin_db": weakest["margin_db"],
            "weakest_margin_over_fade_db": weakest["margin_over_fade_db"],
        })

    # 3: meters really in range of each relay vs children assigned in the plan
    loads = []
    for rid in unit["relay_ids"]:
        in_range = [o for o in plan if o != rid and math.dist(meter_xy[rid], meter_xy[o]) <= d_max]
        children = [o for o, m in plan.items() if not m["parent_is_dcu"] and m["parent"] == rid]
        loads.append({
            "method": method, "unit": unit["unit"], "relay": rid,
            "meters_in_range": len(in_range),
            "children_in_plan": len(children),
            "relay_children": sum(c in relays for c in children),
            "children": " ".join(children),
        })
    # the DCU for reference (it is the root, not a relay)
    loads.append({
        "method": method, "unit": unit["unit"], "relay": f"DCU pole {dcu}",
        "meters_in_range": sum(math.dist(pole_xy[dcu], meter_xy[o]) <= d_max for o in plan),
        "children_in_plan": sum(m["parent_is_dcu"] for m in plan.values()),
        "relay_children": sum(m["parent_is_dcu"] and o in relays for o, m in plan.items()),
        "children": "",
    })
    return list(links.values()), paths, loads


def describe(values: list[float]) -> str:
    if not values:
        return "n=0"
    return (f"n={len(values):3d}  min={min(values):7.2f}  mean={statistics.fmean(values):7.2f}  "
            f"median={statistics.median(values):7.2f}  max={max(values):7.2f}")


def write_csv(rows: list[dict], path: Path) -> None:
    if rows:
        with path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("plans_json", type=Path)
    ap.add_argument("--out-dir", type=Path, default=None, help="write links/paths/relays CSVs here")
    ap.add_argument("--methods", nargs="*", default=None, help="default: every method in the file")
    args = ap.parse_args()

    data = json.loads(args.plans_json.read_text())
    p = data["parameters"]
    meter_xy = {m["id"]: (m["x"], m["y"]) for m in data["meters"]}
    pole_xy = {q["id"]: (q["x"], q["y"]) for q in data["poles"]}
    methods = args.methods or list(data["methods"])

    all_links, all_paths, all_loads = [], [], []
    for method in methods:
        for unit in data["methods"][method]["units"]:
            links, paths, loads = analyse_unit(method, unit, meter_xy, pole_xy, p)
            all_links += links
            all_paths += paths
            all_loads += loads

    print(f"{data['site']}: d_max = {p['d_max_m']:.2f} m, fade margin = {p['fade_margin_db']:g} dB, "
          f"n = {p['path_loss_exponent']:g}")
    for method in methods:
        L = [r for r in all_links if r["method"] == method]
        P = [r for r in all_paths if r["method"] == method]
        print(f"\n=== {method} ===")
        for role in ("access", "backbone"):
            R = [r for r in L if r["role"] == role]
            print(f"  {role:8s} length (m)   {describe([r['length_m'] for r in R])}")
            print(f"  {role:8s} margin (dB)  {describe([r['margin_db'] for r in R])}")
            bad = [r["child"] for r in R if not r["within_d_max"]]
            if bad:
                print(f"  {role:8s} links longer than d_max: {bad}")
        print(f"  weakest-link margin per meter (dB) {describe([r['weakest_margin_db'] for r in P])}")
        for r in sorted(P, key=lambda r: r["weakest_margin_db"])[:3]:
            print(f"    worst: meter {r['meter']:>3s}  {r['weakest_link']:<18s} "
                  f"{r['weakest_length_m']:6.2f} m  {r['weakest_margin_db']:6.2f} dB  "
                  f"({r['weakest_margin_over_fade_db']:+.2f} dB over fade)")
        print("  relay load:")
        for r in (r for r in all_loads if r["method"] == method):
            print(f"    {r['relay']:>12s}  in range = {r['meters_in_range']:3d}  "
                  f"children in plan = {r['children_in_plan']:3d}  (of which relays: {r['relay_children']})")

    if args.out_dir:
        args.out_dir.mkdir(parents=True, exist_ok=True)
        stem = args.plans_json.stem
        write_csv(all_links, args.out_dir / f"{stem}_links.csv")
        write_csv(all_paths, args.out_dir / f"{stem}_paths.csv")
        write_csv(all_loads, args.out_dir / f"{stem}_relays.csv")
        print(f"\nCSVs written to {args.out_dir}")


if __name__ == "__main__":
    main()
