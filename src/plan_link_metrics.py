# ---
# jupyter:
#   jupytext:
#     text_representation:
#       extension: .py
#       format_name: percent
#   kernelspec:
#     display_name: Python 3
#     name: python3
# ---

# %% [markdown]
# # Link length, link margin and relay load of the planned networks
#
# This notebook reads one `<site>_plans.json` written by `dcu_relay_planning.ipynb` and compares
# the methods' plans link by link:
#
# | Part | Content |
# |---|---|
# | 1 | **Link length**, split by role: *access* (non-relay meter → its relay or the DCU) and *backbone* (relay → the next relay or the DCU) |
# | 2 | **Link margin** of every link, and for each meter the **weakest link** on its path to the DCU |
# | 3 | **Relay load**: meters really within $d_\text{max}$ of each relay vs its children in the plan |
# | 4 | Figures and CSV export |
#
# **Link budget.** The same model as the planner:
# $P_\text{rx}(d) = P_\text{EIRP} - PL(d)$ with $PL(d) = PL(d_0) + 10\,n\log_{10}(d/d_0)$.
# Two margins are reported per link:
#
# * `margin_db` $= P_\text{rx} - P_\text{sens}$ (raw margin),
# * `margin_over_fade_db` $=$ `margin_db` $- M$ (fade margin $M$), which is $\ge 0$ exactly when $d \le d_\text{max}$.
#
# Distances below $d_0$ (co-located meters) are clamped to $d_0$.
#
# **IDs.** In a plan, a parent is a *pole* id when `parent_is_dcu` is true and a *meter* id
# otherwise. Meter and pole ids reuse the same numbers (pole 9 ≠ meter 9), so they are looked up in
# separate tables.

# %% [markdown]
# ## Configuration

# %%
import os
from pathlib import Path

PLANS_JSON = Path(os.environ.get("PLANS_JSON", "/workspace/plan_results/Site1_plans.json"))
RESULTS_DIR = Path(os.environ.get("LINK_RESULTS_DIR", "/workspace/plan_results/link_metrics"))
METHODS: list[str] | None = ["Proposed", "Min-hop + min-relay"]  # None = every method in the file
CHART_NAMES = {"Proposed": "Proposed", "Min-hop + min-relay": "Min-hop", "All-relay": "All-relay"}
MAX_RELAYS_SHOWN = 20               # relay-load chart: keep the relays with the most children
FIG_WIDTH_IN = 7.16                 # IEEE two-column width

# %%
# %pip install -q matplotlib numpy pandas

# %%
import json
import math

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D

pd.set_option("display.width", 160)
pd.set_option("display.max_columns", 20)

# %% [markdown]
# ## Load the plans

# %%
data = json.loads(PLANS_JSON.read_text())
SITE = data["site"]
P = data["parameters"]
D_MAX, FADE = P["d_max_m"], P["fade_margin_db"]
METER_XY = {m["id"]: (m["x"], m["y"]) for m in data["meters"]}
POLE_XY = {q["id"]: (q["x"], q["y"]) for q in data["poles"]}
METHODS = METHODS or list(data["methods"])
missing = [m for m in METHODS if m not in data["methods"]]
assert not missing, f"methods not in {PLANS_JSON.name}: {missing}; available: {list(data['methods'])}"

print(f"{SITE}: {len(METER_XY)} meters, {len(POLE_XY)} poles, methods {METHODS}")
print(f"d_max = {D_MAX:.2f} m, fade margin = {FADE:g} dB, n = {P['path_loss_exponent']:g}, "
      f"P_EIRP = {P['p_eirp_dbm']:g} dBm, P_sens = {P['p_sens_dbm']:g} dBm")

# %% [markdown]
# ## Part 1 + 2 — Links, link margin and weakest link per meter
#
# `links` has one row per planned link (meter → parent). `paths` has one row per meter: its route
# to the DCU and the link with the lowest margin on it.


# %%
def path_loss_db(d_m: float) -> float:
    d = max(d_m, P["d0_m"])
    return P["pl_d0_db"] + 10.0 * P["path_loss_exponent"] * math.log10(d / P["d0_m"])


def link_margin_db(d_m: float) -> float:
    return P["p_eirp_dbm"] - path_loss_db(d_m) - P["p_sens_dbm"]


def analyse_unit(method: str, unit: dict) -> tuple[list[dict], list[dict], list[dict]]:
    dcu = unit["dcu_pole_id"]
    plan = {m["id"]: m for m in unit["meters"]}
    relays = set(unit["relay_ids"])

    links = {}
    for mid, m in plan.items():
        parent_xy = POLE_XY[m["parent"]] if m["parent_is_dcu"] else METER_XY[m["parent"]]
        d = math.dist(METER_XY[mid], parent_xy)
        margin = link_margin_db(d)
        links[mid] = {
            "method": method, "unit": unit["unit"], "child": mid,
            "parent": f"DCU {m['parent']}" if m["parent_is_dcu"] else f"relay {m['parent']}",
            "parent_id": m["parent"], "parent_is_dcu": m["parent_is_dcu"],
            "role": "backbone" if mid in relays else "access",
            "length_m": d, "margin_db": margin, "margin_over_fade_db": margin - FADE,
            "within_d_max": d <= D_MAX + 1e-9,
        }

    paths = []
    for mid in plan:
        hops, node = [], mid
        while True:
            hops.append(links[node])
            if plan[node]["parent_is_dcu"]:
                break
            node = plan[node]["parent"]
            if len(hops) > P["ttl_max"]:
                raise ValueError(f"{method}: parent loop at meter {mid}")
        weakest = min(hops, key=lambda r: r["margin_db"])
        paths.append({
            "method": method, "unit": unit["unit"], "meter": mid, "is_relay": mid in relays,
            "hops": len(hops), "path": " → ".join([mid] + [h["parent"] for h in hops]),
            "path_length_m": sum(h["length_m"] for h in hops),
            "weakest_link": f"{weakest['child']} → {weakest['parent']}",
            "weakest_role": weakest["role"], "weakest_length_m": weakest["length_m"],
            "weakest_margin_db": weakest["margin_db"],
            "weakest_margin_over_fade_db": weakest["margin_over_fade_db"],
        })

    loads = []
    for rid in unit["relay_ids"]:
        children = [o for o, m in plan.items() if not m["parent_is_dcu"] and m["parent"] == rid]
        loads.append({
            "method": method, "unit": unit["unit"], "node": f"relay {rid}", "is_dcu": False,
            "meters_in_range": sum(o != rid and math.dist(METER_XY[rid], METER_XY[o]) <= D_MAX for o in plan),
            "children_in_plan": len(children),
            "relay_children": sum(c in relays for c in children),
            "children": " ".join(children),
        })
    dcu_children = [o for o, m in plan.items() if m["parent_is_dcu"]]
    loads.append({  # the DCU, for reference
        "method": method, "unit": unit["unit"], "node": f"DCU {dcu}", "is_dcu": True,
        "meters_in_range": sum(math.dist(POLE_XY[dcu], METER_XY[o]) <= D_MAX for o in plan),
        "children_in_plan": len(dcu_children),
        "relay_children": sum(c in relays for c in dcu_children),
        "children": " ".join(dcu_children),
    })
    return list(links.values()), paths, loads


rows = {"links": [], "paths": [], "loads": []}
for method in METHODS:
    for unit in data["methods"][method]["units"]:
        for key, part in zip(rows, analyse_unit(method, unit)):
            rows[key] += part
links, paths, loads = (pd.DataFrame(rows[k]) for k in ("links", "paths", "loads"))
method_order = pd.CategoricalDtype(METHODS, ordered=True)
for df in (links, paths, loads):
    df["method"] = df["method"].astype(method_order)

# %% [markdown]
# ### Link length and link margin by role

# %%
link_summary = (links.groupby(["method", "role"], observed=True)
                .agg(links=("child", "size"),
                     length_mean_m=("length_m", "mean"), length_median_m=("length_m", "median"),
                     length_max_m=("length_m", "max"),
                     margin_min_db=("margin_db", "min"), margin_median_db=("margin_db", "median"),
                     margin_mean_db=("margin_db", "mean"),
                     beyond_d_max=("within_d_max", lambda s: int((~s).sum())))
                .round(2))
link_summary

# %% [markdown]
# ### Weakest link on each meter's path to the DCU

# %%
weakest_summary = (paths.groupby("method", observed=True)
                   .agg(meters=("meter", "size"), max_hops=("hops", "max"),
                        weakest_min_db=("weakest_margin_db", "min"),
                        weakest_p10_db=("weakest_margin_db", lambda s: s.quantile(0.10)),
                        weakest_median_db=("weakest_margin_db", "median"),
                        weakest_mean_db=("weakest_margin_db", "mean"),
                        weakest_is_backbone=("weakest_role", lambda s: int((s == "backbone").sum())))
                   .round(2))
display(weakest_summary)

cols = ["meter", "hops", "path", "weakest_link", "weakest_role", "weakest_length_m",
        "weakest_margin_db", "weakest_margin_over_fade_db"]
for method in METHODS:
    print(f"\n{method}: 5 meters with the lowest weakest-link margin")
    display(paths[paths["method"] == method].nsmallest(5, "weakest_margin_db")[cols].round(2))

# %% [markdown]
# ## Part 3 — Relay load: meters in range vs children in the plan
#
# `meters_in_range` counts every meter within $d_\text{max}$ of the relay (they all hear its
# rebroadcasts); `children_in_plan` counts the meters whose planned parent is the relay. The DCU is
# listed as a reference row.

# %%
loads[["method", "node", "meters_in_range", "children_in_plan", "relay_children", "children"]]

# %% [markdown]
# ## Part 4 — Figures
#
# Colors follow the method (Proposed blue, Min-hop orange, All-relay aqua) in every figure.
# Figures are saved to `RESULTS_DIR` as PDF (fonts embedded) and PNG.

# %%
PAPER_RC = {
    "font.family": "sans-serif", "font.sans-serif": ["Arial", "DejaVu Sans"], "font.size": 8,
    "pdf.fonttype": 42, "ps.fonttype": 42,
    "axes.labelsize": 9, "xtick.labelsize": 8, "ytick.labelsize": 8,
    "legend.fontsize": 8, "axes.titlesize": 8, "legend.frameon": False,
    "axes.linewidth": 0.5, "xtick.major.width": 0.5, "ytick.major.width": 0.5,
    "xtick.major.size": 2.5, "ytick.major.size": 2.5, "axes.edgecolor": "0.3",
    "xtick.color": "0.2", "ytick.color": "0.2", "axes.labelcolor": "0.1", "text.color": "0.1",
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": "0.9", "grid.linewidth": 0.5, "axes.axisbelow": True,
}
plt.rcParams.update(PAPER_RC)
METHOD_COLOR = {"Proposed": "#2a78d6", "Min-hop + min-relay": "#eb6834", "All-relay": "#1baf7a"}
FALLBACK = ["#e87ba4", "#008300", "#4a3aa7"]  # further categorical slots, in order
for i, m in enumerate(mm for mm in METHODS if mm not in METHOD_COLOR):
    METHOD_COLOR[m] = FALLBACK[i]
REF_LINE = dict(color="0.35", lw=0.8, ls="--", zorder=1)
RESULTS_DIR.mkdir(parents=True, exist_ok=True)


def save(fig: plt.Figure, name: str) -> None:
    for ext in ("pdf", "png"):
        fig.savefig(RESULTS_DIR / f"{SITE}_{name}.{ext}", bbox_inches="tight", dpi=200)


def strip(ax: plt.Axes, df: pd.DataFrame, value: str, rng: np.random.Generator) -> None:
    """One row per (method, role); a dot per link, jittered vertically, median as a tick."""
    labels = []
    for r, (method, role) in enumerate((m, ro) for m in METHODS for ro in ("access", "backbone")):
        v = df.loc[(df["method"] == method) & (df["role"] == role), value].to_numpy()
        y = -r + rng.uniform(-0.18, 0.18, len(v))
        ax.scatter(v, y, s=12 if len(v) > 2 else 28, color=METHOD_COLOR[method], alpha=0.75,
                   edgecolors="white", linewidths=0.4, zorder=3)
        if len(v) > 2:
            ax.plot([np.median(v)] * 2, [-r - 0.3, -r + 0.3], color="0.1", lw=1.2, zorder=2)
        labels.append(f"{CHART_NAMES.get(method, method)} · {role} (n={len(v)})")
    ax.set_yticks(-np.arange(len(labels)), labels)
    ax.tick_params(axis="y", length=0)
    ax.grid(axis="y", visible=False)
    ax.set_ylim(-len(labels) + 0.5, 0.5)


# %% [markdown]
# ### Figure 1 — Link length and link margin by role
#
# One dot per link, the black tick is the median (drawn for more than two links). Left: the dashed line is $d_\text{max}$. Right:
# the dashed line is the fade margin $M$; a link left of it is longer than $d_\text{max}$.

# %%
rng = np.random.default_rng(0)
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(FIG_WIDTH_IN, 0.55 * 2 * len(METHODS) + 0.9), sharey=True)
strip(ax1, links, "length_m", rng)
ax1.axvline(D_MAX, **REF_LINE)
ax1.text(D_MAX, 1.01, f"$d_{{max}}$ = {D_MAX:.1f} m", transform=ax1.get_xaxis_transform(),
         ha="right", va="bottom", fontsize=7, color="0.35")
ax1.set_xlabel("Link length (m)")
ax1.set_xlim(0, D_MAX * 1.08)
strip(ax2, links, "margin_db", rng)
ax2.axvline(FADE, **REF_LINE)
ax2.text(FADE, 1.01, f"fade margin {FADE:g} dB", transform=ax2.get_xaxis_transform(),
         ha="left", va="bottom", fontsize=7, color="0.35")
ax2.set_xlabel("Link margin $P_{rx} - P_{sens}$ (dB)")
ax2.set_xlim(0, None)
fig.suptitle(f"{SITE}: link length and link margin by role", x=0.01, ha="left", fontsize=9)
fig.tight_layout(h_pad=0, rect=(0, 0, 1, 0.97))
save(fig, "link_length_margin")
plt.show()

# %% [markdown]
# ### Figure 2 — Weakest-link margin per meter
#
# Empirical CDF over the meters of the margin of the weakest link on the path to the DCU. A curve
# further right is better; the value where a curve starts is the worst meter of that plan.

# %%
fig, ax = plt.subplots(figsize=(FIG_WIDTH_IN / 2, 2.4))
handles = []
for method in METHODS:
    v = np.sort(paths.loc[paths["method"] == method, "weakest_margin_db"].to_numpy())
    y = np.arange(1, len(v) + 1) / len(v)
    ax.step(np.r_[v[0], v], np.r_[0, y], where="post", color=METHOD_COLOR[method], lw=2)
    handles.append(Line2D([], [], color=METHOD_COLOR[method], lw=2,
                          label=f"{CHART_NAMES.get(method, method)}\nmin {v[0]:.2f}, median {np.median(v):.1f} dB"))
ax.axvline(FADE, **REF_LINE)
ax.set_xlabel("Weakest-link margin on the path to the DCU (dB)")
ax.set_ylabel("Fraction of meters")
ax.set_ylim(0, 1.02)
ax.set_xlim(FADE - 2, None)
ax.legend(handles=handles, loc="lower right", labelspacing=0.8)
ax.set_title(f"{SITE}: weakest link per meter", loc="left")
fig.tight_layout()
save(fig, "weakest_link_cdf")
plt.show()

# %% [markdown]
# ### Figure 3 — Plans on the map, meters colored by weakest-link margin
#
# Thin grey lines are access links, thick dark lines are backbone links. Each meter is colored by
# the margin of the weakest link on its path (one blue ramp, darker = lower margin, so the weak
# meters stand out). ▲ is the DCU, ◆ a relay.

# %%
from matplotlib.colors import LinearSegmentedColormap, Normalize

MARGIN_CMAP = LinearSegmentedColormap.from_list(  # dark = low margin (the weak meters)
    "margin", ["#0d366b", "#256abf", "#5598e7", "#9ec5f4", "#e6effa"])
vmax = float(np.nanpercentile(paths["weakest_margin_db"], 95))
norm = Normalize(vmin=FADE, vmax=max(vmax, FADE + 1))

all_xy = np.array(list(METER_XY.values()))
fig, axes = plt.subplots(1, len(METHODS), figsize=(FIG_WIDTH_IN, FIG_WIDTH_IN / len(METHODS) * 0.95 + 0.4),
                         sharex=True, sharey=True, squeeze=False)
for ax, method in zip(axes[0], METHODS):
    L = links[links["method"] == method]
    for role, lw, color in (("access", 0.6, "0.72"), ("backbone", 1.8, "0.15")):
        segs = []
        for _, r in L[L["role"] == role].iterrows():
            parent_xy = POLE_XY[r["parent_id"]] if r["parent_is_dcu"] else METER_XY[r["parent_id"]]
            segs.append([METER_XY[r["child"]], parent_xy])
        ax.add_collection(LineCollection(segs, colors=color, linewidths=lw, zorder=2 if role == "access" else 3))
    Pm = paths[paths["method"] == method]
    xy = np.array([METER_XY[m] for m in Pm["meter"]])
    sc = ax.scatter(xy[:, 0], xy[:, 1], c=Pm["weakest_margin_db"], cmap=MARGIN_CMAP, norm=norm, s=16,
                    edgecolors="white", linewidths=0.5, zorder=4)
    for unit in data["methods"][method]["units"]:
        rxy = np.array([METER_XY[r] for r in unit["relay_ids"]]).reshape(-1, 2)
        ax.scatter(rxy[:, 0], rxy[:, 1], marker="D", s=34, facecolors="none",
                   edgecolors=METHOD_COLOR[method], linewidths=1.3, zorder=5)
        dx, dy = POLE_XY[unit["dcu_pole_id"]]
        ax.scatter([dx], [dy], marker="^", s=70, color=METHOD_COLOR[method],
                   edgecolors="white", linewidths=0.8, zorder=6)
    ax.set_title(f"{CHART_NAMES.get(method, method)}", loc="left")
    ax.set_aspect("equal")
    ax.grid(False)
    ax.set_xlabel("x (m)")
axes[0, 0].set_ylabel("y (m)")
pad = 8
ax.set_xlim(all_xy[:, 0].min() - pad, all_xy[:, 0].max() + pad)
ax.set_ylim(all_xy[:, 1].min() - pad, all_xy[:, 1].max() + pad)
cb = fig.colorbar(sc, ax=axes[0].tolist(), shrink=0.8, pad=0.02, extend="max")
cb.set_label("Weakest-link margin (dB)")
cb.outline.set_linewidth(0.5)
fig.legend(handles=[Line2D([], [], marker="^", ls="", color="0.3", ms=7, label="DCU"),
                    Line2D([], [], marker="D", ls="", mfc="none", mec="0.3", ms=6, label="relay"),
                    Line2D([], [], color="0.15", lw=1.8, label="backbone link"),
                    Line2D([], [], color="0.72", lw=0.8, label="access link")],
           loc="lower center", ncol=4, bbox_to_anchor=(0.45, -0.02))
fig.suptitle(f"{SITE}: planned links and weakest-link margin per meter", x=0.01, ha="left", fontsize=9)
save(fig, "plan_map_margin")
plt.show()

# %% [markdown]
# ### Figure 4 — Relay load: meters in range vs children in the plan
#
# Light bar: meters within $d_\text{max}$ of the relay (all of them hear its rebroadcasts).
# Dark bar: children assigned to it in the plan. The DCU is shown for reference. With more than
# `MAX_RELAYS_SHOWN` relays, only those with the most children are drawn.

# %%
IN_RANGE_COLOR, CHILDREN_COLOR = "#9ec5f4", "#1c5cab"  # two steps of one blue ramp
panels = []
for method in METHODS:
    Lm = loads[loads["method"] == method]
    relay_rows = Lm[~Lm["is_dcu"]].sort_values(["children_in_plan", "meters_in_range"], ascending=False)
    panels.append((method, pd.concat([Lm[Lm["is_dcu"]], relay_rows.head(MAX_RELAYS_SHOWN)]), len(relay_rows)))

heights = [max(len(df), 2) for _, df, _ in panels]
fig, axes = plt.subplots(len(panels), 1, figsize=(FIG_WIDTH_IN / 2, 0.32 * sum(heights) + 0.6 * len(panels) + 0.5),
                         sharex=True, gridspec_kw={"height_ratios": heights}, squeeze=False)
xmax = max(df[["meters_in_range", "children_in_plan"]].to_numpy().max() for _, df, _ in panels)
for ax, (method, df, n_relays) in zip(axes[:, 0], panels):
    y = np.arange(len(df))
    h = 0.38
    ax.barh(y - h / 2 - 0.02, df["meters_in_range"], h, color=IN_RANGE_COLOR, label="meters in range")
    ax.barh(y + h / 2 + 0.02, df["children_in_plan"], h, color=CHILDREN_COLOR, label="children in plan")
    for yi, (a, b) in enumerate(zip(df["meters_in_range"], df["children_in_plan"])):
        ax.text(a + xmax * 0.01, yi - h / 2 - 0.02, str(a), va="center", fontsize=6.5, color="0.35")
        ax.text(b + xmax * 0.01, yi + h / 2 + 0.02, str(b), va="center", fontsize=6.5, color="0.1")
    ax.set_yticks(y, df["node"])
    ax.invert_yaxis()
    ax.tick_params(axis="y", length=0)
    ax.grid(axis="y", visible=False)
    shown = f", top {MAX_RELAYS_SHOWN} of {n_relays} relays" if n_relays > MAX_RELAYS_SHOWN else ""
    ax.set_title(f"{CHART_NAMES.get(method, method)}{shown}", loc="left")
axes[-1, 0].set_xlabel("Number of meters")
axes[-1, 0].set_xlim(0, xmax * 1.12)
fig.suptitle(f"{SITE}: relay load", x=0.01, ha="left", fontsize=9)
fig_h = fig.get_figheight()  # keep the title and legend band a fixed height in inches
fig.tight_layout()
fig.subplots_adjust(top=1 - 0.75 / fig_h)
fig.legend(*axes[0, 0].get_legend_handles_labels(), loc="upper left",
           bbox_to_anchor=(0.0, 1 - 0.22 / fig_h), ncol=2)
save(fig, "relay_load")
plt.show()

# %% [markdown]
# ## CSV export

# %%
for name, df in (("links", links), ("paths", paths), ("relays", loads)):
    out = RESULTS_DIR / f"{SITE}_{name}.csv"
    df.round(3).to_csv(out, index=False)
    print(f"wrote {out}")
link_summary.to_csv(RESULTS_DIR / f"{SITE}_link_summary.csv")
weakest_summary.to_csv(RESULTS_DIR / f"{SITE}_weakest_summary.csv")
