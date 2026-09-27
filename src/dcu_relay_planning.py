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
# # DCU site and relay-set planning for Bluetooth Mesh smart metering networks
#
# This notebook plans, for every site, **where to install the data concentrator unit (DCU)** and
# **which smart meters act as Bluetooth Mesh relays**.
#
# **Why this matters.** Bluetooth Mesh forwards messages by *managed flooding*: every node configured
# as a relay rebroadcasts each new message once after a short random back-off, and a message cache
# suppresses repeats. When several relays hear the same transmission they rebroadcast almost
# simultaneously. Only one of these rebroadcasts is needed; the rest are redundant and collide.
# The proposed method therefore picks the DCU site and the relay set that **minimise redundant
# rebroadcasts**, using the hop count only to break ties. It is compared with the site criterion
# of routing-based networks, which picks the **minimum-hop** site.
#
# | Part | Content |
# |---|---|
# | Configuration | All parameters in one cell |
# | 0 | Setup: packages, versions, HiGHS API detection |
# | 1 | Data preparation: cleaning, UTM projection, link budget, adjacency, groups, planning units, hop distances |
# | 2 | Proposed method: a single MILP (formulation, solver, independent verifier, brute-force check) |
# | 3 | Baselines (minimum-hop, all-relay) and ablation (proposed relays at the minimum-hop site) |
# | Summary | Tables, CSV, LaTeX, and plan plots |
# | 4 | JSON export for the simulation notebook |
#
# **Input.** For each site, upload a folder to the Colab runtime:
# ```
# /content/sites/<site_name>/meters.csv       columns: DEVICE_NO, LATITUDE, LONGITUDE
# /content/sites/<site_name>/candidates.csv   columns: POLE_NO, LATITUDE, LONGITUDE
# ```
# Then choose *Runtime → Run all*. Outputs are written to `results/`.

# %% [markdown]
# ## Configuration
#
# Every tunable parameter lives in this cell. Nothing below it should need editing.

# %%
# =============================== CONFIGURATION ===============================
import os
from pathlib import Path

# --- Data ---------------------------------------------------------------------
SITES_DIR = Path(os.environ.get("DCU_SITES_DIR", "/content/sites"))  # one sub-folder per site
SITES: list[str] | None = None      # e.g. ["site_a", "site_b"]; None = every folder in SITES_DIR
RESULTS_DIR = Path("results")
DROP_DUPLICATE_IDS = True           # keep only the first row of a duplicated DEVICE_NO / POLE_NO
GENERATE_DEMO_SITE = False          # True = write a synthetic "demo_site" into SITES_DIR (testing only)

# --- Radio (link budget) --------------------------------------------------------
P_EIRP_DBM = 20.0                   # transmit power incl. antenna gain (dBm)
P_SENS_DBM = -98.5                  # receiver sensitivity (dBm)
FADE_MARGIN_DB = 10.0               # fade margin M (dB)
PATH_LOSS_EXP = 3.5                 # log-distance path-loss exponent n
D0_M = 1.0                          # reference distance d0 (m)
FREQ_HZ = 2.44e9                    # carrier frequency (Hz); PL(d0) is free-space loss at d0
SHADOWING_SIGMA_DB = 6.0            # log-normal shadowing sigma; not used here, exported for simulation
D_MAX_OVERRIDE_M: float | None = 90.0  # used instead of the computed d_max if not None

# --- MILP (HiGHS) ---------------------------------------------------------------
TIME_LIMIT_S = 600.0                # per MILP solve
MIP_REL_GAP = 1e-6                  # relative MIP gap at which HiGHS stops
TTL_MAX = 127                       # Bluetooth Mesh maximum TTL; caps the hop count H
RANDOM_SEED = 20240501              # HiGHS random_seed, numpy and random
HIGHS_LOG = False                   # True = show the HiGHS solver log
WARM_START_PRUNING = True           # also try the relay-pruning heuristic as MIP start (see 2.3)

# --- Verification -----------------------------------------------------------------
BRUTE_FORCE_MAX_METERS = 12         # size of the instances checked by exhaustive search

# %% [markdown]
# ## Part 0 — Setup
#
# `highspy` (the Python interface of the HiGHS MILP solver) is not preinstalled on Colab; `pyproj`
# usually is, but it is installed here to be safe. `numpy`, `pandas`, `scipy` and `matplotlib` come
# with Colab and are not reinstalled, so the runtime does not need a restart.

# %%
# %pip install -q highspy pyproj

# %%
import collections
import importlib.metadata
import itertools
import json
import math
import random
import sys
import tempfile
import time
from dataclasses import dataclass, field

import highspy
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pyproj
import scipy
import scipy.sparse
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from scipy.spatial.distance import cdist

random.seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)


def highs_version() -> str:
    """Return the version of the HiGHS library bundled with ``highspy``."""
    h = highspy.Highs()
    if hasattr(h, "version"):
        return str(h.version())
    parts = [getattr(highspy, f"HIGHS_VERSION_{k}", None) for k in ("MAJOR", "MINOR", "PATCH")]
    return ".".join(str(p) for p in parts) if None not in parts else "unknown"


def package_version(name: str) -> str:
    """Return the installed version of a distribution package, or 'unknown'."""
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return "unknown"


# The object-oriented modelling interface (addVariable/addConstr) exists in highspy >= 1.7.
# Older versions only have the low-level matrix interface (addVars/addRows); both are supported.
HAS_MODELING_API = all(hasattr(highspy.Highs, a) for a in ("addVariable", "addConstr"))

print(f"python     {sys.version.split()[0]}")
for pkg in ("highspy", "pyproj", "numpy", "pandas", "scipy", "matplotlib"):
    print(f"{pkg:<10} {package_version(pkg)}")
print(f"HiGHS      {highs_version()}")
print(f"HiGHS API  {'modelling interface (addVariable/addConstr)' if HAS_MODELING_API else 'low-level (addVars/addRows)'}")

# %% [markdown]
# ### Optional: synthetic demo site
#
# With `GENERATE_DEMO_SITE = True` this cell writes a small synthetic site (two street blocks
# separated by an empty strip with poles, plus one far-away meter) into `SITES_DIR/demo_site`.
# It is only for testing the notebook without real data; the same generator builds the
# small instances of the brute-force check in Part 2.

# %%
def make_synthetic_site(
    seed: int,
    blocks: int = 2,
    rows: int = 3,
    cols: int = 4,
    pole_spacing_m: float = 45.0,
    block_gap_m: float = 160.0,
    max_meters_per_pole: int = 3,
    n_far_meters: int = 1,
    origin_lonlat: tuple[float, float] = (100.50, 13.75),
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Generate a synthetic site in latitude/longitude.

    Poles lie on a jittered street grid (``rows`` x ``cols`` per block). Blocks are separated by
    ``block_gap_m`` of empty land with a column of poles in its middle. Each pole serves 0 to
    ``max_meters_per_pole`` meters, some mounted exactly at the pole (identical coordinates),
    others up to 25 m away. ``n_far_meters`` meters are placed 1 km away (uncovered).

    Returns:
        (meters, candidates) data frames with the columns of the real input files.
    """
    rng = np.random.default_rng(seed)
    poles, meters = [], []
    block_width = (cols - 1) * pole_spacing_m
    for b in range(blocks):
        x0 = b * (block_width + block_gap_m)
        for r_ in range(rows):
            for c_ in range(cols):
                p = np.array([x0 + c_ * pole_spacing_m, r_ * pole_spacing_m]) + rng.uniform(-5, 5, 2)
                poles.append(p)
                for _ in range(rng.integers(0, max_meters_per_pole + 1)):
                    if rng.random() < 0.3:
                        meters.append(p.copy())
                    else:
                        ang, rad = rng.uniform(0, 2 * np.pi), rng.uniform(5, 25)
                        meters.append(p + rad * np.array([np.cos(ang), np.sin(ang)]))
        if b < blocks - 1:  # poles without meters in the gap between blocks
            for r_ in range(rows):
                poles.append(np.array([x0 + block_width + block_gap_m / 2, r_ * pole_spacing_m]))
    for k in range(n_far_meters):
        meters.append(np.array([-1000.0 - 10 * k, -1000.0]))
    lon0, lat0 = origin_lonlat
    zone = int(math.floor((lon0 + 180) / 6)) + 1
    to_utm = pyproj.Transformer.from_crs("EPSG:4326", f"EPSG:{32600 + zone}", always_xy=True)
    e0, n0 = to_utm.transform(lon0, lat0)
    to_geo = pyproj.Transformer.from_crs(f"EPSG:{32600 + zone}", "EPSG:4326", always_xy=True)

    def to_frame(xy: list[np.ndarray], id_col: str, prefix: str) -> pd.DataFrame:
        arr = np.array(xy)
        lon, lat = to_geo.transform(e0 + arr[:, 0], n0 + arr[:, 1])
        return pd.DataFrame({id_col: [f"{prefix}{k:04d}" for k in range(len(arr))],
                             "LATITUDE": np.round(lat, 8), "LONGITUDE": np.round(lon, 8)})

    return to_frame(meters, "DEVICE_NO", "M"), to_frame(poles, "POLE_NO", "P")


if GENERATE_DEMO_SITE:
    demo_dir = SITES_DIR / "demo_site"
    demo_dir.mkdir(parents=True, exist_ok=True)
    demo_meters, demo_poles = make_synthetic_site(seed=RANDOM_SEED, rows=4, cols=6, pole_spacing_m=55.0)
    demo_meters.to_csv(demo_dir / "meters.csv", index=False)
    demo_poles.to_csv(demo_dir / "candidates.csv", index=False)
    print(f"Demo site written to {demo_dir}: {len(demo_meters)} meters, {len(demo_poles)} poles")
else:
    print("GENERATE_DEMO_SITE is False: using uploaded sites only.")

# %% [markdown]
# ## Part 1 — Data preparation
#
# ### 1.1 Discover sites, load and clean
#
# Site folders are discovered under `SITES_DIR`; `SITES` restricts the run to named folders.
# For each table, column names and values are stripped of whitespace, rows with missing or
# non-numeric coordinates are dropped, and duplicated IDs are reported (and, with
# `DROP_DUPLICATE_IDS`, only the first row is kept so IDs are unique in the export).
# Several meters sharing identical coordinates is expected (several meters mounted on one pole)
# and is only reported.

# %%
REQUIRED_COLUMNS: dict[str, tuple[str, str, str]] = {
    "meters.csv": ("DEVICE_NO", "LATITUDE", "LONGITUDE"),
    "candidates.csv": ("POLE_NO", "LATITUDE", "LONGITUDE"),
}


def discover_sites(sites_dir: Path, selected: list[str] | None) -> list[Path]:
    """Return the site folders to process, checking that each has both input files.

    Args:
        sites_dir: Folder containing one sub-folder per site.
        selected: Site names to keep, or None for all sub-folders.

    Raises:
        FileNotFoundError: If the folder, a requested site, or an input file is missing.
    """
    if not sites_dir.is_dir():
        raise FileNotFoundError(
            f"Sites folder '{sites_dir}' does not exist. Upload one folder per site to "
            f"{sites_dir}/<site_name>/ containing meters.csv and candidates.csv "
            "(or set GENERATE_DEMO_SITE = True to test with synthetic data).")
    folders = sorted(p for p in sites_dir.iterdir() if p.is_dir() and not p.name.startswith("."))
    if selected is not None:
        missing = sorted(set(selected) - {p.name for p in folders})
        if missing:
            raise FileNotFoundError(f"Sites listed in SITES not found in {sites_dir}: {missing}. "
                                    f"Available: {[p.name for p in folders]}")
        folders = [p for p in folders if p.name in set(selected)]
    if not folders:
        raise FileNotFoundError(f"No site folders found in '{sites_dir}'.")
    for folder in folders:
        for fname in REQUIRED_COLUMNS:
            if not (folder / fname).is_file():
                raise FileNotFoundError(f"Site '{folder.name}': missing file {folder / fname}")
    return folders


def load_table(path: Path, columns: tuple[str, str, str], drop_duplicate_ids: bool) -> pd.DataFrame:
    """Load one input CSV, validate its columns and clean it.

    Args:
        path: CSV file.
        columns: (id column, latitude column, longitude column).
        drop_duplicate_ids: Keep only the first row of each duplicated ID.

    Returns:
        Data frame with the ID as string and coordinates as float, index reset.

    Raises:
        ValueError: If a required column is missing or coordinates are out of range.
    """
    id_col, lat_col, lon_col = columns
    df = pd.read_csv(path, dtype=str, encoding="utf-8-sig")
    df.columns = [str(c).strip().upper() for c in df.columns]
    missing = [c for c in columns if c not in df.columns]
    if missing:
        raise ValueError(f"{path}: missing column(s) {missing}; found {list(df.columns)}")
    df = df[list(columns)].copy()
    for c in columns:
        df[c] = df[c].str.strip()
    n_raw = len(df)
    for c in (lat_col, lon_col):
        df[c] = pd.to_numeric(df[c], errors="coerce")
    bad = df[[lat_col, lon_col]].isna().any(axis=1) | df[id_col].isna() | (df[id_col] == "")
    df = df[~bad]
    out = ~df[lat_col].between(-90, 90) | ~df[lon_col].between(-180, 180)
    if out.any():
        raise ValueError(f"{path}: {int(out.sum())} rows with coordinates out of range, "
                         f"e.g. {df[out].head(3).to_dict('records')}")
    dup_mask = df[id_col].duplicated(keep=False)
    n_dup_ids = df.loc[dup_mask, id_col].nunique()
    print(f"  {path.name:<15} {n_raw} rows, {int(bad.sum())} dropped (missing ID/coordinates), "
          f"{n_dup_ids} duplicated {id_col} values")
    if n_dup_ids:
        examples = df.loc[dup_mask, id_col].unique()[:5].tolist()
        action = "keeping the first row of each" if drop_duplicate_ids else "kept as is"
        print(f"    duplicates ({action}): {examples}{' ...' if n_dup_ids > 5 else ''}")
        if drop_duplicate_ids:
            df = df.drop_duplicates(subset=id_col, keep="first")
    if df.empty:
        raise ValueError(f"{path}: no valid rows after cleaning.")
    return df.reset_index(drop=True)


def report_shared_coordinates(meters: pd.DataFrame) -> None:
    """Print how many meters share identical coordinates with another meter."""
    counts = meters.groupby(["LATITUDE", "LONGITUDE"]).size()
    shared = counts[counts > 1]
    if shared.empty:
        print("  no meters share identical coordinates")
    else:
        print(f"  {int(shared.sum())} meters share coordinates at {len(shared)} locations "
              f"(max {int(shared.max())} per location; expected for meters on one pole)")

# %% [markdown]
# ### 1.2 Projection to UTM and local coordinates
#
# Distances must be computed in metres, so coordinates are projected to UTM (WGS 84). Thailand
# spans zones 47N and 48N; each site uses **one** zone, chosen from the longitude of the centroid
# of its meters and poles:
#
# $$\text{zone} = \left\lfloor \frac{\lambda + 180}{6} \right\rfloor + 1, \qquad
# \text{EPSG} = 32600 + \text{zone} \ \ (\text{northern hemisphere}),$$
#
# i.e. EPSG 32647 or 32648. A site straddling the zone boundary is still projected into a single
# zone; the distortion a few kilometres beyond the boundary is negligible at the scale of a
# 90 m link. The projected coordinates are then shifted so that the south-west corner of the
# site is at the origin (local coordinates, in metres).

# %%
def utm_zone_epsg(lon: float, lat: float) -> tuple[int, int]:
    """Return (UTM zone, EPSG code) of the WGS 84 / UTM zone containing a point."""
    zone = int(math.floor((lon + 180.0) / 6.0)) + 1
    return zone, (32600 if lat >= 0 else 32700) + zone


def project_to_local(meters: pd.DataFrame, poles: pd.DataFrame) -> tuple[np.ndarray, np.ndarray, int, int]:
    """Project meters and poles to one UTM zone and shift them to local coordinates.

    Returns:
        (meter_xy, pole_xy, zone, epsg); coordinate arrays have shape (n, 2), in metres.
    """
    lon = np.concatenate([meters["LONGITUDE"].to_numpy(), poles["LONGITUDE"].to_numpy()])
    lat = np.concatenate([meters["LATITUDE"].to_numpy(), poles["LATITUDE"].to_numpy()])
    zone, epsg = utm_zone_epsg(float(lon.mean()), float(lat.mean()))
    transformer = pyproj.Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
    x, y = transformer.transform(lon, lat)
    xy = np.column_stack([x, y])
    origin = np.floor(xy.min(axis=0))
    xy = xy - origin
    n_m = len(meters)
    return xy[:n_m], xy[n_m:], zone, epsg

# %% [markdown]
# ### 1.3 Link budget and maximum link distance
#
# With the log-distance path-loss model, a link of length $d$ closes with fade margin $M$ when
#
# $$P_\text{EIRP} - \Big[\mathrm{PL}(d_0) + 10\,n\,\log_{10}\frac{d}{d_0}\Big] \;\ge\; P_\text{sens} + M,$$
#
# which gives the maximum link distance
#
# $$d_\text{max} = d_0 \cdot 10^{\left(P_\text{EIRP} - P_\text{sens} - M - \mathrm{PL}(d_0)\right)/(10\,n)} .$$
#
# The reference loss is the free-space path loss at $d_0$:
# $\mathrm{PL}(d_0) = 20 \log_{10}\!\left(4\pi d_0 f / c\right) \approx 40.2$ dB at 2.44 GHz and 1 m.
# With $\mathrm{PL}(d_0)$ rounded to 40 dB the formula gives 90.6 m, which the paper rounds down
# to $d_\text{max} = 90$ m (`D_MAX_OVERRIDE_M`). The unrounded value is printed for reference.
# Shadowing ($\sigma$ = `SHADOWING_SIGMA_DB`) is covered by the fade margin here and is only
# used by the simulation.

# %%
SPEED_OF_LIGHT_M_S = 299_792_458.0


def free_space_path_loss_db(d_m: float, freq_hz: float) -> float:
    """Free-space path loss 20 log10(4 pi d f / c) in dB."""
    return 20.0 * math.log10(4.0 * math.pi * d_m * freq_hz / SPEED_OF_LIGHT_M_S)


def max_link_distance_m(p_eirp_dbm: float, p_sens_dbm: float, margin_db: float,
                        pl_d0_db: float, n: float, d0_m: float) -> float:
    """Largest distance at which the log-distance link budget closes with the fade margin."""
    return d0_m * 10.0 ** ((p_eirp_dbm - p_sens_dbm - margin_db - pl_d0_db) / (10.0 * n))


PL_D0_DB = free_space_path_loss_db(D0_M, FREQ_HZ)
D_MAX_COMPUTED_M = max_link_distance_m(P_EIRP_DBM, P_SENS_DBM, FADE_MARGIN_DB, PL_D0_DB, PATH_LOSS_EXP, D0_M)
D_MAX_M = float(D_MAX_OVERRIDE_M) if D_MAX_OVERRIDE_M is not None else D_MAX_COMPUTED_M

print(f"P_EIRP              = {P_EIRP_DBM:8.2f} dBm")
print(f"P_sens              = {P_SENS_DBM:8.2f} dBm")
print(f"fade margin M       = {FADE_MARGIN_DB:8.2f} dB")
print(f"PL(d0) (free space) = {PL_D0_DB:8.2f} dB   (d0 = {D0_M} m, f = {FREQ_HZ / 1e9} GHz)")
print(f"path-loss exponent n= {PATH_LOSS_EXP:8.2f}")
print(f"allowed path loss   = {P_EIRP_DBM - P_SENS_DBM - FADE_MARGIN_DB:8.2f} dB at d_max")
print(f"d_max computed      = {D_MAX_COMPUTED_M:8.2f} m  "
      f"(with PL(d0) rounded to 40 dB: "
      f"{max_link_distance_m(P_EIRP_DBM, P_SENS_DBM, FADE_MARGIN_DB, 40.0, PATH_LOSS_EXP, D0_M):.2f} m)")
print(f"d_max used          = {D_MAX_M:8.2f} m  "
      f"({'override D_MAX_OVERRIDE_M' if D_MAX_OVERRIDE_M is not None else 'computed'})")

# %% [markdown]
# ### 1.4 Distances and adjacency
#
# Let $\mathcal{M}$ be the meters and $\mathcal{P}$ the candidate poles of a site. With the
# Euclidean distances $d_{ij}$ (meter–meter) and $d_{ic}$ (meter–pole),
#
# $$A(i) = \{\, j \in \mathcal{M} \setminus \{i\} : d_{ij} \le d_\text{max} \,\}, \qquad
#   D(c) = \{\, i \in \mathcal{M} : d_{ic} \le d_\text{max} \,\}.$$
#
# $A(i)$ are the meters that hear a transmission of meter $i$; $D(c)$ are the meters a DCU at pole
# $c$ hears directly. A meter is **isolated** when $A(i) = \emptyset$ and no pole is in range.
#
# ### 1.5 Groups
#
# The DCU only receives and never rebroadcasts, so it does not connect meters with each other.
# Groups are therefore the connected components of the meter–meter graph $(\mathcal{M}, A)$,
# found by breadth-first search (BFS). The candidate set of group $g$ is
# $C_g = \{ c : D(c) \cap g \ne \emptyset \}$. A group with $C_g = \emptyset$ cannot reach any
# DCU; its meters are **uncovered** and are not planned.
#
# ### 1.6 Planning units
#
# A planning unit is a set of groups served by **one** DCU. Each group is its own unit, unless
# groups share a candidate pole, in which case they are merged. One DCU at pole $c$ serves a
# merged unit only if $c$ is in range of every group of the unit, so the candidate set of a unit
# is the intersection $\bigcap_{g} C_g$ of its groups' candidate sets.
# If groups are linked only through a chain of different poles (G1–G2 share $p$, G2–G3 share
# $q$, no pole shared by all three), no single pole serves them all. Such a component is split
# deterministically: groups are taken in order of decreasing size (ties by smallest meter index)
# and each is added to the current unit while the intersection stays non-empty.
#
# ### 1.7 All-relay hop distances
#
# For each unit and candidate pole $c$, $\delta_{ic}$ is the hop count from meter $i$ to a DCU at
# $c$ when all meters relay: a multi-source BFS over the unit's meter graph started from the
# meters in $D(c)$ at distance 1. $\delta_{ic}$ is the shortest possible hop count, so it is
# a lower bound on the hops of any plan with the DCU at $c$.

# %%
@dataclass
class PlanningUnit:
    """One planning unit: meters served by a single DCU and the poles where that DCU may go.

    All arrays use **local** indices: meter ``i`` is ``meters[i]`` of the site and candidate ``k``
    is pole ``candidates[k]`` of the site.

    Attributes:
        site: Site name.
        uid: Unit index within the site.
        meters: Global meter indices (sorted), shape (N,).
        candidates: Global pole indices (sorted), shape (C,).
        groups: Group ids merged into this unit.
        A: A[i] = local indices of the meters within d_max of meter i.
        D: D[k] = local indices of the meters within d_max of candidate k.
        delta: (N, C) all-relay hop counts.
        dist_mm: (N, N) meter-meter distances (m).
        dist_mp: (N, C) meter-candidate distances (m).
        d_max: Maximum link distance (m).
    """
    site: str
    uid: int
    meters: np.ndarray
    candidates: np.ndarray
    groups: list[int]
    A: list[np.ndarray]
    D: list[np.ndarray]
    delta: np.ndarray
    dist_mm: np.ndarray
    dist_mp: np.ndarray
    d_max: float

    @property
    def N(self) -> int:
        return len(self.meters)

    @property
    def C(self) -> int:
        return len(self.candidates)

    @property
    def degree(self) -> np.ndarray:
        """|A(i)| for every meter: number of meters hearing a transmission of meter i."""
        return np.array([len(a) for a in self.A], dtype=int)


@dataclass
class Site:
    """A prepared site: cleaned tables, local coordinates, adjacency, groups and planning units."""
    name: str
    meters: pd.DataFrame
    poles: pd.DataFrame
    zone: int
    epsg: int
    meter_xy: np.ndarray
    pole_xy: np.ndarray
    d_max: float
    dist_mm: np.ndarray
    dist_mp: np.ndarray
    A: list[np.ndarray]
    D: list[np.ndarray]
    groups: list[np.ndarray]
    group_candidates: list[np.ndarray]
    units: list[PlanningUnit] = field(default_factory=list)
    isolated: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))
    uncovered: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))


def neighbor_lists(within: np.ndarray, exclude_self: bool) -> list[np.ndarray]:
    """Turn a boolean 'within range' matrix into per-row index arrays (sorted)."""
    out = []
    for i, row in enumerate(within):
        idx = np.flatnonzero(row)
        out.append(idx[idx != i] if exclude_self else idx)
    return out


def bfs_hops(A: list[np.ndarray], sources: np.ndarray, forwards: np.ndarray | None = None) -> np.ndarray:
    """Multi-source BFS hop counts towards a DCU.

    The meters in ``sources`` (those in range of the DCU) have hop count 1. A meter j with hop
    count k lets every neighbour i in A[j] reach the DCU in k + 1 hops, **but only if j forwards**
    (``forwards[j]`` is True). With ``forwards=None`` all meters forward (all-relay distances);
    with the relay set it gives flooding hop counts.

    Returns:
        Hop count per meter; -1 for meters that cannot reach the DCU.
    """
    n = len(A)
    hops = np.full(n, -1, dtype=int)
    queue = collections.deque()
    for s in np.asarray(sources, dtype=int):
        hops[s] = 1
        queue.append(s)
    while queue:
        j = queue.popleft()
        if forwards is not None and not forwards[j]:
            continue
        for i in A[j]:
            if hops[i] < 0:
                hops[i] = hops[j] + 1
                queue.append(i)
    return hops


def connected_groups(A: list[np.ndarray]) -> list[np.ndarray]:
    """Connected components of the meter-meter graph (BFS), ordered by smallest member."""
    label = np.full(len(A), -1, dtype=int)
    groups = []
    for s in range(len(A)):
        if label[s] >= 0:
            continue
        label[s] = len(groups)
        members, queue = [s], collections.deque([s])
        while queue:
            i = queue.popleft()
            for j in A[i]:
                if label[j] < 0:
                    label[j] = len(groups)
                    members.append(j)
                    queue.append(j)
        groups.append(np.array(sorted(members), dtype=int))
    return groups


def merge_groups_into_units(groups: list[np.ndarray], group_cands: list[np.ndarray]) -> list[tuple[list[int], np.ndarray]]:
    """Merge groups that share a candidate pole into planning units (see 1.6).

    Returns:
        List of (group ids, candidate pole indices) per unit. Groups without candidates are skipped.
    """
    covered = [g for g in range(len(groups)) if len(group_cands[g])]
    # Union-find over groups that share at least one pole.
    parent = {g: g for g in covered}

    def find(g: int) -> int:
        while parent[g] != g:
            parent[g] = parent[parent[g]]
            g = parent[g]
        return g

    first_group_of_pole: dict[int, int] = {}
    for g in covered:
        for c in group_cands[g]:
            if c in first_group_of_pole:
                parent[find(g)] = find(first_group_of_pole[c])
            else:
                first_group_of_pole[int(c)] = g
    components: dict[int, list[int]] = collections.defaultdict(list)
    for g in covered:
        components[find(g)].append(g)

    units = []
    for comp in components.values():
        order = sorted(comp, key=lambda g: (-len(groups[g]), int(groups[g][0])))
        while order:
            members = [order.pop(0)]
            common = set(group_cands[members[0]].tolist())
            for g in list(order):
                shared = common & set(group_cands[g].tolist())
                if shared:
                    common = shared
                    members.append(g)
                    order.remove(g)
            units.append((sorted(members), np.array(sorted(common), dtype=int)))
    units.sort(key=lambda u: int(min(groups[g][0] for g in u[0])))
    if sum(len(c) for c in components.values()) != sum(len(u[0]) for u in units):
        raise AssertionError("group merging lost a group")
    return units


def make_planning_unit(site: Site, uid: int, meter_idx: np.ndarray, cand_idx: np.ndarray,
                       group_ids: list[int]) -> PlanningUnit:
    """Build a PlanningUnit (local adjacency, distances and delta) from global indices.

    The meter set must be closed under adjacency (no meter outside the unit is a neighbour of a
    meter inside), which holds for unions of connected groups.
    """
    meter_idx = np.asarray(sorted(meter_idx), dtype=int)
    cand_idx = np.asarray(sorted(cand_idx), dtype=int)
    local = {int(g): i for i, g in enumerate(meter_idx)}
    A = []
    for g in meter_idx:
        nb = [local.get(int(j), -1) for j in site.A[g]]
        if min(nb, default=0) < 0:
            raise AssertionError("unit is not closed under adjacency")
        A.append(np.array(sorted(nb), dtype=int))
    dist_mm = site.dist_mm[np.ix_(meter_idx, meter_idx)]
    dist_mp = site.dist_mp[np.ix_(meter_idx, cand_idx)]
    D = [np.flatnonzero(dist_mp[:, k] <= site.d_max) for k in range(len(cand_idx))]
    delta = np.column_stack([bfs_hops(A, D[k]) for k in range(len(cand_idx))]) if len(cand_idx) \
        else np.zeros((len(meter_idx), 0), dtype=int)
    if (delta < 0).any():
        raise AssertionError(f"unit {uid}: some meter cannot reach some candidate pole")
    return PlanningUnit(site=site.name, uid=uid, meters=meter_idx, candidates=cand_idx,
                        groups=list(group_ids), A=A, D=D, delta=delta,
                        dist_mm=dist_mm, dist_mp=dist_mp, d_max=site.d_max)


def prepare_site(folder: Path, d_max: float) -> Site:
    """Run steps 1.1-1.7 for one site folder and return the prepared Site."""
    print(f"\n=== Site '{folder.name}' ===")
    meters = load_table(folder / "meters.csv", REQUIRED_COLUMNS["meters.csv"], DROP_DUPLICATE_IDS)
    poles = load_table(folder / "candidates.csv", REQUIRED_COLUMNS["candidates.csv"], DROP_DUPLICATE_IDS)
    report_shared_coordinates(meters)

    meter_xy, pole_xy, zone, epsg = project_to_local(meters, poles)
    meters[["X", "Y"]] = meter_xy
    poles[["X", "Y"]] = pole_xy
    print(f"  UTM zone {zone}N (EPSG:{epsg}); local extent "
          f"{np.ptp(np.vstack([meter_xy, pole_xy])[:, 0]):.0f} m x {np.ptp(np.vstack([meter_xy, pole_xy])[:, 1]):.0f} m")
    if zone not in (47, 48):
        print(f"  WARNING: zone {zone} is outside Thailand's zones 47N/48N - check the coordinates")

    dist_mm = cdist(meter_xy, meter_xy)
    dist_mp = cdist(meter_xy, pole_xy)
    A = neighbor_lists(dist_mm <= d_max, exclude_self=True)
    D = neighbor_lists((dist_mp <= d_max).T, exclude_self=False)
    deg = np.array([len(a) for a in A])
    poles_in_range = (dist_mp <= d_max).sum(axis=1)
    isolated = np.flatnonzero((deg == 0) & (poles_in_range == 0))
    print(f"  meter degree |A(i)|: min {deg.min()}, mean {deg.mean():.1f}, max {deg.max()}; "
          f"poles in range per meter: mean {poles_in_range.mean():.1f}; "
          f"meters per pole |D(c)|: mean {np.mean([len(d) for d in D]):.1f}")
    print(f"  isolated meters (no meter and no pole in range): {len(isolated)}"
          + (f" {meters['DEVICE_NO'].iloc[isolated].tolist()[:10]}" if len(isolated) else ""))

    groups = connected_groups(A)
    in_range = dist_mp <= d_max
    group_cands = [np.flatnonzero(in_range[g].any(axis=0)) for g in groups]
    site = Site(folder.name, meters, poles, zone, epsg, meter_xy, pole_xy, d_max,
                dist_mm, dist_mp, A, D, groups, group_cands, isolated=isolated)
    site.uncovered = np.array(sorted(i for g, cs in zip(groups, group_cands) if len(cs) == 0 for i in g), dtype=int)

    for uid, (gids, cands) in enumerate(merge_groups_into_units(groups, group_cands)):
        m_idx = np.concatenate([groups[g] for g in gids])
        site.units.append(make_planning_unit(site, uid, m_idx, cands, gids))
    n_planned = sum(u.N for u in site.units)
    if n_planned + len(site.uncovered) != len(meters):
        raise AssertionError("every meter must be either in a unit or uncovered")
    print(f"  {len(groups)} groups -> {len(site.units)} planning units; "
          f"{len(site.uncovered)} uncovered meters")
    return site

# %% [markdown]
# ### 1.8 Run data preparation, report and plot

# %%
SITE_FOLDERS = discover_sites(SITES_DIR, SITES)
print(f"Sites to process: {[p.name for p in SITE_FOLDERS]}")
sites: dict[str, Site] = {folder.name: prepare_site(folder, D_MAX_M) for folder in SITE_FOLDERS}


def site_report(site: Site) -> dict:
    """One-row summary of a prepared site."""
    sizes = np.array([len(g) for g in site.groups])
    return {
        "site": site.name, "meters": len(site.meters), "poles": len(site.poles),
        "UTM zone": f"{site.zone}N (EPSG:{site.epsg})", "d_max (m)": site.d_max,
        "groups": len(site.groups),
        "group size min/median/max": f"{sizes.min()}/{np.median(sizes):g}/{sizes.max()}",
        "singleton groups": int((sizes == 1).sum()),
        "planning units": len(site.units),
        "merged units": sum(len(u.groups) > 1 for u in site.units),
        "candidates per unit min/median/max": "/".join(
            f"{v:g}" for v in (lambda c: (c.min(), np.median(c), c.max()))(np.array([u.C for u in site.units])))
        if site.units else "-",
        "isolated meters": len(site.isolated),
        "uncovered meters": len(site.uncovered),
    }


with pd.option_context("display.max_columns", None, "display.width", 200):
    print(pd.DataFrame([site_report(s) for s in sites.values()]).set_index("site").T.to_string())

for site in sites.values():
    unit_table = pd.DataFrame([{"unit": u.uid, "groups": len(u.groups), "meters": u.N,
                                "candidates": u.C, "max degree": int(u.degree.max()),
                                "min over poles of mean delta": round(float(u.delta.mean(axis=0).min()), 2)}
                               for u in site.units])
    print(f"\nPlanning units of '{site.name}':")
    print(unit_table.to_string(index=False))


def plot_site(site: Site, ax: plt.Axes) -> None:
    """Plot meters, poles and meter-meter links, coloured by group."""
    cmap = plt.get_cmap("tab20")
    group_of = np.empty(len(site.meters), dtype=int)
    for g, members in enumerate(site.groups):
        group_of[members] = g
    i, j = np.nonzero(np.triu(site.dist_mm <= site.d_max, k=1))
    segs = np.stack([site.meter_xy[i], site.meter_xy[j]], axis=1)
    ax.add_collection(LineCollection(segs, colors=cmap(group_of[i] % 20), linewidths=0.5, alpha=0.5))
    ax.scatter(*site.pole_xy.T, marker="s", s=18, facecolors="none", edgecolors="k", linewidths=0.6)
    ax.scatter(*site.meter_xy.T, s=9, c=cmap(group_of % 20), zorder=3)
    if len(site.uncovered):
        ax.scatter(*site.meter_xy[site.uncovered].T, s=40, marker="x", c="red", zorder=4)
    ax.set_aspect("equal")
    ax.set_title(f"{site.name}: {len(site.meters)} meters, {len(site.poles)} poles, "
                 f"{len(site.groups)} groups, {len(site.units)} units")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(handles=[Line2D([], [], marker="o", ls="", c="tab:blue", label="meter (colour = group)"),
                       Line2D([], [], marker="s", ls="", mfc="none", c="k", label="candidate pole"),
                       Line2D([], [], marker="x", ls="", c="red", label="uncovered meter"),
                       Line2D([], [], c="grey", label=f"link <= {site.d_max:g} m")],
              loc="best", fontsize=8)


for site in sites.values():
    fig, ax = plt.subplots(figsize=(9, 7))
    plot_site(site, ax)
    plt.show()

# %% [markdown]
# ## Part 2 — Proposed method: a single MILP
#
# ### 2.1 Formulation
#
# For a planning unit with meters $N$, candidate poles $C$, adjacency $A(i)$ and $D(c)$, let
# $H = \min(|N|, \text{TTL}_\text{max})$ and $\varepsilon = 1 / (H\,|N| + 1)$.
#
# **Variables**
#
# | Variable | Domain | Meaning |
# |---|---|---|
# | $y_c$, $c \in C$ | $\{0,1\}$ | DCU installed at pole $c$ |
# | $r_j$, $j \in N$ | $\{0,1\}$ | meter $j$ is a relay |
# | $x_{ij}$, $i \in N,\ j \in A(i)$ | $\{0,1\}$ | the parent of meter $i$ is meter $j$ |
# | $u_{ic}$, $c \in C,\ i \in D(c)$ | $\{0,1\}$ | meter $i$ sends directly to a DCU at $c$ |
# | $h_i$, $i \in N$ | $[1, H]$ | hop count from meter $i$ to the DCU |
#
# **Objective (lexicographic through a small weight)**
#
# $$\min\ \underbrace{\sum_{j \in N} |A(j)|\, r_j}_{\text{redundant rebroadcasts } R}
#        \;+\; \varepsilon \underbrace{\sum_{i \in N} h_i}_{\text{total hops}}$$
#
# Every transmission of meter $i$ is heard by the relays in $A(i)$, and each of them rebroadcasts
# it. Summed over all meters, $\sum_i |A(i) \cap \mathcal{R}| = \sum_{j \in \mathcal{R}} |A(j)|$
# because the neighbour relation is symmetric, which is the first term. Since $1 \le h_i \le H$,
# the hop term satisfies $0 < \varepsilon \sum_i h_i \le H|N| / (H|N|+1) < 1$. The first term is
# an integer, so any decrease of it by one outweighs every possible change of the hop term:
# the MILP minimises $R$ first and uses the hops only to break ties.
#
# **Constraints**
#
# $$\begin{aligned}
# & \textstyle\sum_{c \in C} y_c = 1 && \text{one DCU per unit} \\
# & u_{ic} \le y_c && \forall c \in C,\ i \in D(c) \quad \text{direct links only to the chosen pole} \\
# & \textstyle\sum_{j \in A(i)} x_{ij} + \sum_{c:\, i \in D(c)} u_{ic} = 1 && \forall i \in N \quad \text{exactly one parent} \\
# & x_{ij} \le r_j && \forall i \in N,\ j \in A(i) \quad \text{a meter parent must relay} \\
# & h_i \ge h_j + 1 - H\,(1 - x_{ij}) && \forall i \in N,\ j \in A(i) \quad \text{hop propagation}
# \end{aligned}$$
#
# If $x_{ij} = 1$ the last constraint gives $h_i \ge h_j + 1$, so hop counts strictly increase
# away from the DCU along parent links and a cycle $i \to j \to \dots \to i$ would require
# $h_i > h_i$: the parent links form a tree rooted at the DCU. If $x_{ij} = 0$ it reads
# $h_i \ge h_j + 1 - H$, which holds for all $h \in [1,H]^N$. Along the tree, $h_i$ is at least
# the depth of $i$; minimising $\varepsilon \sum h_i$ makes it equal to the depth at the optimum.
# $h_i \le H \le \text{TTL}_\text{max}$ ensures every planned path is deliverable with a valid TTL.
#
# **Optimality of the primary term.** HiGHS reports a lower bound $LB$ on the objective. Because
# $R$ is an integer and the hop term lies in $(0,1)$, the optimal $R^*$ satisfies
# $R^* > LB - 1$, i.e. $R^* \ge \lfloor LB \rfloor$. The notebook reports this bound, so a run
# stopped by the time limit still certifies how far its $R$ can be from optimal.

# %% [markdown]
# ### 2.2 Model construction and solver
#
# The model is first built once as sparse arrays (column bounds, costs, integrality and the
# constraint rows in CSR form); `build_milp` is the only place where the formulation is encoded.
# It is then loaded into HiGHS through the **modelling interface** (`addVariable` / `addConstr`)
# when the installed `highspy` has it, and otherwise through the **low-level** interface
# (`addVars` / `addRows`). Both give the same model; the brute-force check below solves with both.
#
# HiGHS receives a **MIP start** (see 2.3). Before it is passed, the start vector is checked
# against every row, bound and integrality requirement of the model.

# %%
@dataclass
class MilpModel:
    """Sparse representation of the MILP of one planning unit (see 2.1).

    Column ``k`` has bounds [lb[k], ub[k]], cost cost[k] and is integer if is_int[k]. Row ``r`` is
    ``row_lower[r] <= sum_k a_rk x_k <= row_upper[r]``, with coefficients stored in CSR form.
    y, r, h are column indices per candidate/meter; x[(i, j)] and u[(i, k)] map pairs to columns.
    """
    lb: np.ndarray
    ub: np.ndarray
    cost: np.ndarray
    is_int: np.ndarray
    row_lower: np.ndarray
    row_upper: np.ndarray
    row_start: np.ndarray
    row_index: np.ndarray
    row_value: np.ndarray
    y: np.ndarray
    r: np.ndarray
    h: np.ndarray
    x: dict[tuple[int, int], int]
    u: dict[tuple[int, int], int]
    H: int
    eps: float

    @property
    def n_cols(self) -> int:
        return len(self.lb)

    @property
    def n_rows(self) -> int:
        return len(self.row_lower)

    def matrix(self) -> scipy.sparse.csr_matrix:
        """Constraint matrix as a scipy CSR matrix."""
        return scipy.sparse.csr_matrix((self.row_value, self.row_index, self.row_start),
                                       shape=(self.n_rows, self.n_cols))


def build_milp(unit: PlanningUnit, fixed_site: int | None = None, ttl_max: int = TTL_MAX) -> MilpModel:
    """Encode the formulation of 2.1 for one planning unit.

    Args:
        unit: Planning unit.
        fixed_site: Local candidate index; if given, y is fixed to that pole (ablation).
        ttl_max: Maximum TTL; H = min(|N|, ttl_max).
    """
    N, C = unit.N, unit.C
    H = min(N, ttl_max)
    eps = 1.0 / (H * N + 1)
    deg = unit.degree
    lb, ub, cost, is_int = [], [], [], []

    def add_cols(n: int, lo: float, hi: float, c: np.ndarray | float, integer: bool) -> np.ndarray:
        start = len(lb)
        lb.extend([lo] * n)
        ub.extend([hi] * n)
        cost.extend(np.broadcast_to(np.asarray(c, dtype=float), (n,)).tolist())
        is_int.extend([integer] * n)
        return np.arange(start, start + n)

    y = add_cols(C, 0.0, 1.0, 0.0, True)
    r = add_cols(N, 0.0, 1.0, deg, True)
    x_pairs = [(i, int(j)) for i in range(N) for j in unit.A[i]]
    x = dict(zip(x_pairs, add_cols(len(x_pairs), 0.0, 1.0, 0.0, True).tolist()))
    u_pairs = [(int(i), k) for k in range(C) for i in unit.D[k]]
    u = dict(zip(u_pairs, add_cols(len(u_pairs), 0.0, 1.0, 0.0, True).tolist()))
    h = add_cols(N, 1.0, float(H), eps, False)
    lb, ub = np.array(lb), np.array(ub)
    if fixed_site is not None:
        if not 0 <= fixed_site < C:
            raise ValueError(f"fixed_site={fixed_site} is not a candidate index of unit {unit.uid}")
        lb[y], ub[y] = 0.0, 0.0
        lb[y[fixed_site]] = ub[y[fixed_site]] = 1.0

    rows: list[tuple[list[int], list[float], float, float]] = []
    rows.append((y.tolist(), [1.0] * C, 1.0, 1.0))                              # sum_c y_c = 1
    for (i, k), col in u.items():                                               # u_ic - y_c <= 0
        rows.append(([col, int(y[k])], [1.0, -1.0], -np.inf, 0.0))
    parents_of: list[list[int]] = [[] for _ in range(N)]
    for (i, j), col in x.items():
        parents_of[i].append(col)
    for (i, k), col in u.items():
        parents_of[i].append(col)
    for i in range(N):                                                          # one parent
        rows.append((parents_of[i], [1.0] * len(parents_of[i]), 1.0, 1.0))
    for (i, j), col in x.items():                                               # x_ij - r_j <= 0
        rows.append(([col, int(r[j])], [1.0, -1.0], -np.inf, 0.0))
    for (i, j), col in x.items():                                               # h_i - h_j - H x_ij >= 1 - H
        rows.append(([int(h[i]), int(h[j]), col], [1.0, -1.0, -float(H)], 1.0 - H, np.inf))

    row_start = np.cumsum([0] + [len(rw[0]) for rw in rows]).astype(np.int32)
    return MilpModel(
        lb=lb, ub=ub, cost=np.array(cost), is_int=np.array(is_int, dtype=bool),
        row_lower=np.array([rw[2] for rw in rows]), row_upper=np.array([rw[3] for rw in rows]),
        row_start=row_start,
        row_index=np.array([c for rw in rows for c in rw[0]], dtype=np.int32),
        row_value=np.array([v for rw in rows for v in rw[1]], dtype=float),
        y=y, r=r, h=h, x=x, u=u, H=H, eps=eps)


def check_point_feasible(model: MilpModel, vec: np.ndarray, tol: float = 1e-9) -> list[str]:
    """Return the list of violated bounds, integrality and rows of ``vec`` (empty if feasible)."""
    problems = []
    if (vec < model.lb - tol).any() or (vec > model.ub + tol).any():
        problems.append("bound violated")
    if (np.abs(vec[model.is_int] - np.rint(vec[model.is_int])) > tol).any():
        problems.append("integrality violated")
    act = model.matrix() @ vec
    bad = np.flatnonzero((act < model.row_lower - tol) | (act > model.row_upper + tol))
    if len(bad):
        problems.append(f"{len(bad)} rows violated (first: {bad[:5].tolist()})")
    return problems


def _linear_sum(h: highspy.Highs, terms: list) -> object:
    """Sum of highspy linear expressions, using the fast qsum when available."""
    return h.qsum(terms) if hasattr(h, "qsum") else sum(terms[1:], terms[0])


def load_model(h: highspy.Highs, model: MilpModel, api: str) -> None:
    """Load ``model`` into a Highs instance with the modelling ('modeling') or 'low-level' API."""
    if api == "modeling":
        cols = []
        for k in range(model.n_cols):
            vtype = highspy.HighsVarType.kInteger if model.is_int[k] else highspy.HighsVarType.kContinuous
            cols.append(h.addVariable(lb=model.lb[k], ub=model.ub[k], obj=model.cost[k], type=vtype))
        if getattr(cols[-1], "index", model.n_cols - 1) != model.n_cols - 1:
            raise AssertionError("unexpected column order in the modelling interface")
        for rw in range(model.n_rows):
            s, e = model.row_start[rw], model.row_start[rw + 1]
            expr = _linear_sum(h, [float(v) * cols[int(c)] for c, v in zip(model.row_index[s:e], model.row_value[s:e])])
            lo, hi = model.row_lower[rw], model.row_upper[rw]
            if lo == hi:
                h.addConstr(expr == lo)
            elif np.isinf(lo):
                h.addConstr(expr <= hi)
            elif np.isinf(hi):
                h.addConstr(expr >= lo)
            else:
                raise ValueError("ranged rows are not used by this model")
    elif api == "low-level":
        inf = highspy.kHighsInf
        n = model.n_cols
        h.addVars(n, model.lb, model.ub)
        h.changeColsCost(n, np.arange(n, dtype=np.int32), model.cost)
        int_cols = np.flatnonzero(model.is_int).astype(np.int32)
        h.changeColsIntegrality(len(int_cols), int_cols,
                                np.array([highspy.HighsVarType.kInteger] * len(int_cols)))
        h.addRows(model.n_rows, np.where(np.isinf(model.row_lower), -inf, model.row_lower),
                  np.where(np.isinf(model.row_upper), inf, model.row_upper),
                  len(model.row_index), model.row_start[:-1], model.row_index, model.row_value)
    else:
        raise ValueError(f"unknown api '{api}'")
    if h.getNumCol() != model.n_cols or h.getNumRow() != model.n_rows:
        raise AssertionError("HiGHS model size differs from the built model")

# %% [markdown]
# ### 2.3 Plans, MIP starts, and the solver
#
# A **plan** of a unit is a DCU pole, a relay set and one parent per meter (a meter or the DCU).
# Any plan whose parent links form a tree rooted at the DCU, whose meter parents are relays and
# whose depth is at most $H$ is a feasible point of 2.1 with $h_i$ = depth of $i$.
# Two such plans are built, and the one with the smaller objective is passed to HiGHS:
#
# * **Minimum-hop plan** (the Part 3 baseline, built here because the solver needs it): a
#   shortest-path tree by BFS from the DCU; every meter with a child relays; $h_i = \delta_{ic}$.
#   With `fixed_site`, the same tree at the fixed pole.
# * **Relay pruning** (`WARM_START_PRUNING`): for each candidate site, start with all meters
#   relaying and switch relays off one at a time, largest $|A(j)|$ first (ties by index), keeping
#   a switch-off whenever every meter still reaches the DCU through relays within $H$ hops. The
#   result is a minimal relay set; its plan is the BFS tree through the relays.
#
# The MIP start only affects how fast HiGHS finds good solutions, never the model or its
# optimum. The LP relaxation of 2.1 is weak (relay selection is a connected-dominating-set
# problem), so on large units HiGHS may stop at the time limit close to its start; the reported
# lower bound $\lfloor LB \rfloor$ shows how far such a solution can be from optimal. The solve
# time reported for the MILP methods includes building the MIP starts.

# %%
@dataclass
class Plan:
    """A DCU site, relay set and parent tree for one planning unit (local indices).

    Attributes:
        method: Method name.
        uid: Planning-unit index.
        site_k: Local candidate index of the DCU pole.
        relays: Boolean relay flag per meter.
        parent: Parent per meter: a local meter index, or -1 for the DCU.
        h: Hop variables from the MILP (rounded), or None for heuristic plans.
        solve_time_s: Wall-clock time to compute the plan.
        status: Solver status ('-' for heuristics).
        objective, best_bound, gap: HiGHS objective, dual bound and relative gap (MILP only).
        primary_lb: floor(best_bound): lower bound on the optimal redundant rebroadcasts.
        fallback: True if the solver returned no solution and the MIP start was used instead.
        start: Name of the plan passed to HiGHS as MIP start (MILP only).
        start_objective: Objective value of that MIP start.
    """
    method: str
    uid: int
    site_k: int
    relays: np.ndarray
    parent: np.ndarray
    h: np.ndarray | None = None
    solve_time_s: float = 0.0
    status: str = "-"
    objective: float = float("nan")
    best_bound: float = float("nan")
    gap: float = float("nan")
    primary_lb: float = float("nan")
    fallback: bool = False
    start: str = "-"
    start_objective: float = float("nan")


def shortest_path_tree(unit: PlanningUnit, k: int) -> tuple[np.ndarray, float]:
    """BFS shortest-path tree towards a DCU at candidate ``k``.

    Meters in D(k) (depth 1) have the DCU as parent. A meter at depth d > 1 takes as parent its
    nearest neighbour at depth d - 1 (ties: smallest index).

    Returns:
        (parent array with -1 for the DCU, total link length in metres).
    """
    depth = unit.delta[:, k]
    parent = np.full(unit.N, -1, dtype=int)
    total = 0.0
    for i in range(unit.N):
        if depth[i] == 1:
            total += unit.dist_mp[i, k]
            continue
        prev = unit.A[i][depth[unit.A[i]] == depth[i] - 1]
        j = int(prev[np.argmin(unit.dist_mm[i, prev])])  # argmin returns the first (smallest index) on ties
        parent[i] = j
        total += unit.dist_mm[i, j]
    return parent, total


def tree_plan(unit: PlanningUnit, k: int, method: str) -> Plan:
    """Shortest-path-tree plan at candidate ``k``; relays are the meters with at least one child."""
    t0 = time.perf_counter()
    parent, _ = shortest_path_tree(unit, k)
    relays = np.zeros(unit.N, dtype=bool)
    relays[parent[parent >= 0]] = True
    return Plan(method, unit.uid, k, relays, parent, solve_time_s=time.perf_counter() - t0)


def min_hop_site(unit: PlanningUnit) -> tuple[int, pd.DataFrame]:
    """Minimum-hop site: smallest mean delta, ties by total tree link length, then pole index.

    Returns:
        (local candidate index, table of the criteria for every candidate).
    """
    rows = []
    for k in range(unit.C):
        _, length = shortest_path_tree(unit, k)
        rows.append({"k": k, "sum_delta": int(unit.delta[:, k].sum()),
                     "mean_delta": unit.delta[:, k].mean(), "tree_length_m": length})
    table = pd.DataFrame(rows).sort_values(["sum_delta", "tree_length_m", "k"])  # sum == mean * N, exact ints
    return int(table["k"].iloc[0]), table


def min_hop_plan(unit: PlanningUnit) -> Plan:
    """Baseline of Tanakornpintong and Pirak (2021): minimum-hop site with its shortest-path tree."""
    t0 = time.perf_counter()
    k, _ = min_hop_site(unit)
    plan = tree_plan(unit, k, "minimum-hop")
    plan.solve_time_s = time.perf_counter() - t0
    return plan


def adjacency_matrix(unit: PlanningUnit) -> scipy.sparse.csr_matrix:
    """Symmetric 0/1 meter-meter adjacency matrix of a unit (CSR)."""
    rows = np.repeat(np.arange(unit.N), unit.degree)
    cols = np.concatenate(unit.A) if unit.N else np.array([], dtype=int)
    return scipy.sparse.csr_matrix((np.ones(len(rows), dtype=np.float32), (rows, cols)), shape=(unit.N, unit.N))


def reach_hops_fast(adj: scipy.sparse.csr_matrix, sources: np.ndarray, forwards: np.ndarray) -> np.ndarray:
    """Vectorised equivalent of ``bfs_hops(A, sources, forwards)`` (one sparse product per layer)."""
    hops = np.full(adj.shape[0], -1, dtype=int)
    frontier = np.zeros(adj.shape[0], dtype=bool)
    frontier[sources] = True
    hops[frontier] = 1
    d = 1
    while True:
        fwd = frontier & forwards
        if not fwd.any():
            return hops
        frontier = (adj @ fwd.astype(np.float32) > 0) & (hops < 0)
        if not frontier.any():
            return hops
        d += 1
        hops[frontier] = d


def relay_tree(unit: PlanningUnit, k: int, relays: np.ndarray) -> np.ndarray:
    """Parent array of the BFS tree through relays: each meter's parent is its nearest relay
    neighbour one flooding hop closer to the DCU at ``k`` (or the DCU for meters in D(k))."""
    hops = bfs_hops(unit.A, unit.D[k], forwards=relays)
    parent = np.full(unit.N, -1, dtype=int)
    for i in np.flatnonzero(hops > 1):
        prev = unit.A[i][(hops[unit.A[i]] == hops[i] - 1) & relays[unit.A[i]]]
        parent[i] = int(prev[np.argmin(unit.dist_mm[i, prev])])
    return parent


def pruned_relay_plan(unit: PlanningUnit, fixed_site: int | None = None, ttl_max: int = TTL_MAX) -> Plan:
    """Relay-pruning heuristic (MIP start only).

    For each candidate site (or only ``fixed_site``), start with every meter relaying and try to
    switch relays off one by one, most expensive first (largest |A(j)|, ties by index); a switch-off
    is kept if every meter still reaches the DCU through relays within H hops. The result is a
    minimal relay set; its plan is the BFS tree through the relays. The site with the smallest
    MILP objective (R + eps * sum of hops) is returned.
    """
    t0 = time.perf_counter()
    H = min(unit.N, ttl_max)
    eps = 1.0 / (H * unit.N + 1)
    adj = adjacency_matrix(unit)
    order = sorted(range(unit.N), key=lambda j: (-unit.degree[j], j))
    best = None
    for k in (range(unit.C) if fixed_site is None else [fixed_site]):
        relays = np.ones(unit.N, dtype=bool)
        for j in order:
            relays[j] = False
            hops = reach_hops_fast(adj, unit.D[k], relays)
            if (hops < 1).any() or hops.max() > H:
                relays[j] = True
        hops = reach_hops_fast(adj, unit.D[k], relays)
        obj = unit.degree[relays].sum() + eps * hops.sum()
        if best is None or obj < best[0] - 1e-12:
            best = (obj, k, relays)
    _, k, relays = best
    return Plan("relay pruning", unit.uid, k, relays, relay_tree(unit, k, relays),
                solve_time_s=time.perf_counter() - t0)


def plan_to_vector(model: MilpModel, unit: PlanningUnit, plan: Plan) -> np.ndarray:
    """Encode a tree plan as a MILP point, with h = depth in the parent tree."""
    vec = np.zeros(model.n_cols)
    vec[model.y[plan.site_k]] = 1.0
    vec[model.r[plan.relays]] = 1.0
    for i, p in enumerate(plan.parent):
        if p >= 0:
            vec[model.x[(i, int(p))]] = 1.0
        else:
            vec[model.u[(i, plan.site_k)]] = 1.0
    vec[model.h] = tree_depths(plan.parent)
    return vec


def tree_depths(parent: np.ndarray) -> np.ndarray:
    """Depth of every node in a parent array (-1 = DCU); raises ValueError on a cycle."""
    n = len(parent)
    depth = np.zeros(n, dtype=int)
    for i in range(n):
        path, v = [], i
        while v >= 0 and depth[v] == 0:
            path.append(v)
            v = int(parent[v])
            if len(path) > n:
                raise ValueError(f"cycle in parent pointers through meter {i}")
        d = 0 if v < 0 else depth[v]
        for w in reversed(path):
            d += 1
            depth[w] = d
    return depth


def solve_milp(unit: PlanningUnit, fixed_site: int | None = None, *, api: str = "auto",
               time_limit_s: float = TIME_LIMIT_S, method: str | None = None) -> Plan:
    """Solve the MILP of 2.1 for one planning unit with HiGHS.

    Args:
        unit: Planning unit.
        fixed_site: Local candidate index to fix the DCU to (ablation), or None to optimise it.
        api: 'auto' (modelling interface if available), 'modeling' or 'low-level'.
        time_limit_s: HiGHS time limit.
        method: Name stored in the plan (default 'proposed' or 'ablation').

    Returns:
        The extracted plan, including status, objective, best bound, gap and solve time.
    """
    if api == "auto":
        api = "modeling" if HAS_MODELING_API else "low-level"
    method = method or ("proposed" if fixed_site is None else "ablation")
    model = build_milp(unit, fixed_site)

    # MIP start: the better (by objective) of the minimum-hop tree and the relay-pruning plan.
    t_start = time.perf_counter()
    starts = [min_hop_plan(unit) if fixed_site is None else tree_plan(unit, fixed_site, "minimum-hop tree")]
    if WARM_START_PRUNING:
        starts.append(pruned_relay_plan(unit, fixed_site))
    feasible_starts = []
    for cand in starts:
        vec = plan_to_vector(model, unit, cand)
        problems = check_point_feasible(model, vec)
        if problems:
            print(f"    WARNING: MIP start '{cand.method}' infeasible ({problems})")
        else:
            feasible_starts.append((float(model.cost @ vec), cand.method, vec))
    if feasible_starts:
        start_obj, start_name, start_vec = min(feasible_starts, key=lambda t: t[0])
    start_time = time.perf_counter() - t_start

    h = highspy.Highs()
    h.setOptionValue("output_flag", bool(HIGHS_LOG))
    h.setOptionValue("log_to_console", bool(HIGHS_LOG))
    h.setOptionValue("time_limit", float(time_limit_s))
    h.setOptionValue("mip_rel_gap", float(MIP_REL_GAP))
    h.setOptionValue("random_seed", int(RANDOM_SEED))
    load_model(h, model, api)
    if feasible_starts:
        sol = highspy.HighsSolution()
        sol.col_value = start_vec.tolist()
        sol.value_valid = True
        h.setSolution(sol)
    else:
        print("    WARNING: no feasible MIP start; solving without one")

    t0 = time.perf_counter()
    h.run()
    elapsed = time.perf_counter() - t0
    info = h.getInfo()
    status = h.modelStatusToString(h.getModelStatus())
    has_solution = int(info.primal_solution_status) == 2  # kSolutionStatusFeasible

    if has_solution:
        vec = np.asarray(h.getSolution().col_value, dtype=float)
        plan = extract_plan(model, unit, vec, method)
        plan.objective = float(info.objective_function_value)
    elif feasible_starts:
        print(f"    WARNING: HiGHS returned no solution ({status}); using the MIP start")
        plan = extract_plan(model, unit, start_vec, method)
        plan.objective = start_obj
        plan.fallback = True
    else:
        raise RuntimeError(f"unit {unit.uid}: no solution found ({status})")
    plan.solve_time_s = start_time + elapsed  # includes building the MIP starts
    plan.status = status
    if feasible_starts:
        plan.start, plan.start_objective = start_name, start_obj
    plan.best_bound = float(info.mip_dual_bound)
    plan.gap = float(info.mip_gap)
    plan.primary_lb = float(math.floor(plan.best_bound + 1e-6)) if math.isfinite(plan.best_bound) else float("nan")
    return plan


def extract_plan(model: MilpModel, unit: PlanningUnit, vec: np.ndarray, method: str) -> Plan:
    """Read site, relays, parents and rounded hop variables from a MILP point."""
    site_k = int(np.argmax(vec[model.y]))
    relays = vec[model.r] > 0.5
    parent = np.full(unit.N, -2, dtype=int)  # -2 = no parent found (caught by the verifier)
    for (i, j), col in model.x.items():
        if vec[col] > 0.5:
            parent[i] = j
    for (i, k), col in model.u.items():
        if vec[col] > 0.5:
            if k != site_k:
                raise AssertionError(f"meter {i} is linked to a pole without a DCU")
            parent[i] = -1
    h_int = np.rint(vec[model.h]).astype(int)
    return Plan(method, unit.uid, site_k, relays, parent, h=h_int)

# %% [markdown]
# ### 2.4 Independent verifier and scoring
#
# `verify_plan` does not use the MILP model; it only reads the plan and the unit's geometry,
# and is applied to **every** method. It checks that
#
# 1. every meter has a parent, and parent chains reach the DCU without cycles;
# 2. every meter used as a parent is a relay;
# 3. every link (meter–meter and meter–DCU) is at most $d_\text{max}$ long;
# 4. the tree depth is at most $\text{TTL}_\text{max}$;
# 5. for MILP plans, the hops recomputed from the parent pointers equal the MILP's $h_i$.
#
# It then computes the two hop measures:
#
# * **tree hops**: the depth of each meter in the planned parent tree (the TTL exported for the
#   simulation, see Part 4);
# * **flooding hops**: under managed flooding, the first copy of a message reaches the DCU along
#   the shortest path through relays, i.e. a BFS from the DCU in which only relays forward
#   (`bfs_hops` with `forwards = relays`). Flooding hops never exceed tree hops.
#
# and the redundant rebroadcasts $R = \sum_{j} |A(j)|\, r_j$ (the MILP's first objective term).

# %%
def verify_plan(unit: PlanningUnit, plan: Plan, ttl_max: int = TTL_MAX) -> dict:
    """Independently check a plan and compute its metrics.

    Raises:
        AssertionError: If the plan violates a structural requirement (1-4). A mismatch between
            MILP hop variables and tree depths (5) is reported in the returned dict.

    Returns:
        Dict with relay count, redundant rebroadcasts, tree/flooding hop arrays and statistics.
    """
    k = plan.site_k
    tag = f"[{unit.site} unit {unit.uid} {plan.method}]"
    assert 0 <= k < unit.C, f"{tag} DCU index out of range"
    assert len(plan.parent) == unit.N and len(plan.relays) == unit.N, f"{tag} wrong array sizes"
    assert (plan.parent >= -1).all(), f"{tag} meters without parent: {np.flatnonzero(plan.parent < -1)[:10]}"
    for i, p in enumerate(plan.parent):
        if p == -1:
            assert unit.dist_mp[i, k] <= unit.d_max + 1e-9, f"{tag} meter {i} too far from the DCU"
        else:
            assert p != i, f"{tag} meter {i} is its own parent"
            assert unit.dist_mm[i, p] <= unit.d_max + 1e-9, f"{tag} link {i}->{p} longer than d_max"
            assert plan.relays[p], f"{tag} parent {p} of meter {i} is not a relay"
    depth = tree_depths(plan.parent)  # raises on cycles
    assert depth.max() <= ttl_max, f"{tag} tree depth {depth.max()} exceeds TTL_MAX"
    flood = bfs_hops(unit.A, unit.D[k], forwards=plan.relays)
    assert (flood >= 1).all() and (flood <= depth).all(), f"{tag} flooding hops inconsistent"
    h_match = None if plan.h is None else bool(np.array_equal(plan.h, depth))
    if h_match is False:
        print(f"    NOTE {tag}: MILP h differs from tree depth for {int((plan.h != depth).sum())} meters "
              f"(h >= depth always; equality requires the hop term to be optimal)")
        assert (plan.h >= depth).all(), f"{tag} MILP h below tree depth"
    has_child = np.zeros(unit.N, dtype=bool)
    has_child[plan.parent[plan.parent >= 0]] = True
    return {
        "relays": int(plan.relays.sum()),
        "redundant": int(unit.degree[plan.relays].sum()),
        "relays_without_child": int((plan.relays & ~has_child).sum()),
        "tree_hops": depth, "flooding_hops": flood,
        "tree_mean": float(depth.mean()), "tree_max": int(depth.max()),
        "flood_mean": float(flood.mean()), "flood_max": int(flood.max()),
        "h_matches_tree": h_match,
    }


def plan_objective(unit: PlanningUnit, ev: dict, ttl_max: int = TTL_MAX) -> float:
    """MILP objective value of a verified plan: R + eps * sum(tree hops)."""
    H = min(unit.N, ttl_max)
    return ev["redundant"] + ev["tree_hops"].sum() / (H * unit.N + 1)

# %% [markdown]
# ### 2.5 Brute-force check on small instances
#
# For a small unit, the optimum is found by exhaustive search, independently of the MILP: for
# every candidate site $c$ and every relay subset $\mathcal{R} \subseteq N$ ($2^{|N|}$ subsets),
# the plan is feasible iff every meter reaches the DCU through relays, and the smallest total
# hop count for $(c, \mathcal{R})$ is the sum of flooding hops (a BFS tree through relays
# attains it; $h_i \le H$ holds since a shortest path visits at most $|N|$ meters). So
#
# $$\text{OPT} = \min_{c,\ \mathcal{R}\ \text{feasible}} \Big( \sum_{j \in \mathcal{R}} |A(j)|
#   + \varepsilon \sum_i \text{hops}_{c,\mathcal{R}}(i) \Big).$$
#
# The MILP (with both HiGHS interfaces, free and fixed site) must reach the same value.
# Two kinds of instances are checked: four small sparse synthetic sites, and a connected
# sub-network of up to `BRUTE_FORCE_MAX_METERS` meters cut from the largest planning unit of the
# first real site (with the poles in range of its first meter as candidates).

# %%
def brute_force_optimum(unit: PlanningUnit, fixed_site: int | None = None, ttl_max: int = TTL_MAX) -> tuple[float, int, np.ndarray]:
    """Exhaustive search over sites and relay subsets; returns (objective, site, relay mask)."""
    N = unit.N
    if N > 20:
        raise ValueError("brute force is limited to 20 meters")
    H = min(N, ttl_max)
    eps = 1.0 / (H * N + 1)
    deg = unit.degree
    best = (math.inf, -1, np.zeros(N, dtype=bool))
    sites_ = range(unit.C) if fixed_site is None else [fixed_site]
    for mask in range(2 ** N):
        relays = np.array([(mask >> j) & 1 for j in range(N)], dtype=bool)
        primary = deg[relays].sum()
        if primary > best[0]:
            continue
        for k in sites_:
            hops = bfs_hops(unit.A, unit.D[k], forwards=relays)
            if (hops < 1).any() or hops.max() > H:
                continue
            obj = primary + eps * hops.sum()
            if obj < best[0] - 1e-12:
                best = (obj, k, relays)
    return best


def sub_unit(site: Site, unit: PlanningUnit, n_max: int, uid: int = 0) -> PlanningUnit:
    """A connected sub-network of up to ``n_max`` meters of ``unit``, as a stand-alone unit.

    Meters are collected by BFS from the meter farthest (in all-relay hops) from the unit's
    minimum-hop pole. Its adjacency is restricted to the selected meters. Its candidates are the
    poles in range of the BFS start meter, so meters far from the start need relays.
    """
    k0, _ = min_hop_site(unit)
    start = int(np.argmax(unit.delta[:, k0]))
    order, seen, queue = [], {start}, collections.deque([start])
    while queue and len(order) < n_max:
        i = queue.popleft()
        order.append(i)
        for j in unit.A[i]:
            if int(j) not in seen:
                seen.add(int(j))
                queue.append(int(j))
    glob = unit.meters[np.array(order)]
    sub = restrict_adjacency(site, glob)
    cands = np.flatnonzero(site.dist_mp[glob[0]] <= site.d_max)
    return make_planning_unit(sub, uid, glob, cands, [0])


def restrict_adjacency(site: Site, keep: np.ndarray) -> Site:
    """Shallow copy of ``site`` whose meter adjacency only links meters in ``keep``."""
    keep_set = set(int(i) for i in keep)
    A = [np.array([j for j in site.A[i] if int(j) in keep_set], dtype=int) if i in keep_set else np.array([], dtype=int)
         for i in range(len(site.A))]
    return Site(site.name + "-sub", site.meters, site.poles, site.zone, site.epsg, site.meter_xy, site.pole_xy,
                site.d_max, site.dist_mm, site.dist_mp, A, site.D, site.groups, site.group_candidates)


def site_from_frames(name: str, meters: pd.DataFrame, poles: pd.DataFrame, d_max: float) -> Site:
    """Prepare a site from in-memory data frames (used for synthetic test instances)."""
    folder = Path(tempfile.mkdtemp(prefix="dcu_check_")) / name
    folder.mkdir()
    meters.to_csv(folder / "meters.csv", index=False)
    poles.to_csv(folder / "candidates.csv", index=False)
    return prepare_site(folder, d_max)


def brute_force_check(unit: PlanningUnit, label: str) -> None:
    """Compare MILP optima (both APIs, free and fixed site) with exhaustive search."""
    print(f"\nBrute-force check '{label}': {unit.N} meters, {unit.C} candidates, "
          f"{2 ** unit.N} relay subsets")
    bf_obj, bf_k, bf_relays = brute_force_optimum(unit)
    print(f"  exhaustive search: objective {bf_obj:.6f} (R = {int(unit.degree[bf_relays].sum())}), "
          f"site {bf_k}, relays {np.flatnonzero(bf_relays).tolist()}")
    apis = ["low-level"] + (["modeling"] if HAS_MODELING_API else [])
    k_fix = min_hop_site(unit)[0]
    bf_fix = brute_force_optimum(unit, fixed_site=k_fix)[0]
    for api in apis:
        for fixed, target in ((None, bf_obj), (k_fix, bf_fix)):
            plan = solve_milp(unit, fixed_site=fixed, api=api)
            ev = verify_plan(unit, plan)
            obj_plan = plan_objective(unit, ev)
            ok = (abs(plan.objective - target) < 1e-6 and abs(obj_plan - target) < 1e-6
                  and plan.status == "Optimal")
            print(f"  MILP [{api:<9} site={'free' if fixed is None else f'fixed {fixed}'}]: "
                  f"HiGHS objective {plan.objective:.6f}, verified plan objective {obj_plan:.6f}, "
                  f"target {target:.6f}, status {plan.status} -> {'OK' if ok else 'MISMATCH'}")
            assert ok, "MILP does not match the exhaustive search"


for seed_offset in range(1, 5):  # several sparse synthetic layouts
    small_meters, small_poles = make_synthetic_site(seed=RANDOM_SEED + seed_offset, blocks=1, rows=2, cols=6,
                                                    pole_spacing_m=70.0, max_meters_per_pole=2, n_far_meters=0)
    small_site = site_from_frames(f"synthetic_{seed_offset}", small_meters, small_poles, D_MAX_M)
    small_unit = max(small_site.units, key=lambda u: u.N)
    if small_unit.N > BRUTE_FORCE_MAX_METERS:
        small_unit = sub_unit(small_site, small_unit, BRUTE_FORCE_MAX_METERS)
    brute_force_check(small_unit, f"synthetic seed +{seed_offset}")

first_site = next(iter(sites.values()))
if first_site.units:
    big = max(first_site.units, key=lambda u: u.N)
    real_sub = sub_unit(first_site, big, BRUTE_FORCE_MAX_METERS)
    brute_force_check(real_sub, f"sub-network of {first_site.name} unit {big.uid}")
print("\nAll brute-force checks passed.")

# %% [markdown]
# ## Part 3 — Baselines and ablation
#
# ### 3.1 Minimum-hop placement (Tanakornpintong and Pirak, 2021)
#
# The site criterion of routing-based networks: for each candidate pole $c$ compute the mean
# all-relay hop count $\bar\delta_c = \frac{1}{|N|}\sum_i \delta_{ic}$ and choose the smallest.
# Ties are broken by the smallest total link length of the shortest-path tree, then by pole
# index. The tree is built by BFS from the DCU: each meter's parent is its nearest neighbour in
# the previous BFS layer (or the DCU itself for layer 1). Relays are the meters with at least one
# child (`min_hop_plan`, defined in 2.3).
#
# > The original work also applies throughput and delay thresholds derived from an IEEE
# > 802.15.4g queuing model. Those thresholds belong to a different MAC/PHY and a routing-based
# > network, so they are **not** used here; only the minimum-hop site criterion is adopted.
#
# ### 3.2 All-relay
#
# The same pole and tree as the minimum-hop baseline, but every meter relays (the Bluetooth Mesh
# default when the relay feature is enabled on all nodes).
#
# ### 3.3 Ablation
#
# `solve_milp(unit, fixed_site=<minimum-hop pole>)`: the proposed relay selection at the baseline's
# site. It separates the gain of relay selection from the gain of site selection.
#
# All methods are scored with the same quantities as the MILP objective (redundant rebroadcasts
# and hops) and checked by `verify_plan`.

# %%
def all_relay_plan(base: Plan) -> Plan:
    """Minimum-hop pole and tree with every meter relaying."""
    return Plan("all-relay", base.uid, base.site_k, np.ones_like(base.relays), base.parent.copy(),
                solve_time_s=base.solve_time_s)


METHODS = ("minimum-hop", "all-relay", "proposed", "ablation")
# results[site][method] = list of (plan, evaluation) per unit
results: dict[str, dict[str, list[tuple[Plan, dict]]]] = {}

for site in sites.values():
    print(f"\n=== Site '{site.name}': {len(site.units)} planning units ===")
    results[site.name] = {m: [] for m in METHODS}
    for unit in site.units:
        t_unit = time.perf_counter()
        base = min_hop_plan(unit)
        plans = {
            "minimum-hop": base,
            "all-relay": all_relay_plan(base),
            "proposed": solve_milp(unit),
            "ablation": solve_milp(unit, fixed_site=base.site_k),
        }
        for m, plan in plans.items():
            results[site.name][m].append((plan, verify_plan(unit, plan)))
        ev = {m: results[site.name][m][-1][1] for m in METHODS}
        p = plans["proposed"]
        print(f"  unit {unit.uid + 1}/{len(site.units)}: N={unit.N}, C={unit.C} | "
              f"R min-hop {ev['minimum-hop']['redundant']}, all-relay {ev['all-relay']['redundant']}, "
              f"ablation {ev['ablation']['redundant']}, proposed {ev['proposed']['redundant']} "
              f"(>= {p.primary_lb:g}) | proposed {p.status}, gap {p.gap:.2e}, {p.solve_time_s:.1f}s, "
              f"start '{p.start}' R+eps*hops={p.start_objective:.2f} | "
              f"unit total {time.perf_counter() - t_unit:.1f}s")

# %% [markdown]
# ## Summary
#
# Per site and method the metrics are aggregated over the site's planning units: DCUs (one per
# unit), relays, redundant rebroadcasts $R$, mean and max tree and flooding hops (over all planned
# meters), total solve time, MILP statuses and the largest MIP gap. `R lower bound` is
# $\sum_\text{units} \lfloor LB \rfloor$ (see 2.1): if it equals $R$, the proposed relay sets are
# proven optimal in redundant rebroadcasts.

# %%
def summarize(sites: dict[str, Site], results: dict) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Aggregate per (site, method) and per unit (site comparison)."""
    rows, unit_rows = [], []
    for name, site in sites.items():
        for m in METHODS:
            pairs = results[name][m]
            if not pairs:
                continue
            tree = np.concatenate([ev["tree_hops"] for _, ev in pairs])
            flood = np.concatenate([ev["flooding_hops"] for _, ev in pairs])
            milp = m in ("proposed", "ablation")
            statuses = collections.Counter(p.status for p, _ in pairs)
            rows.append({
                "site": name, "meters": len(site.meters), "planned meters": len(tree), "method": m,
                "DCUs": len(pairs),
                "relays": sum(ev["relays"] for _, ev in pairs),
                "redundant rebroadcasts": sum(ev["redundant"] for _, ev in pairs),
                "R lower bound": sum(p.primary_lb for p, _ in pairs) if milp else np.nan,
                "mean tree hops": tree.mean(), "max tree hops": int(tree.max()),
                "mean flooding hops": flood.mean(), "max flooding hops": int(flood.max()),
                "solve time (s)": sum(p.solve_time_s for p, _ in pairs),
                "MILP status": ", ".join(f"{s} x{c}" for s, c in statuses.items()) if milp else "-",
                "max MIP gap": max(p.gap for p, _ in pairs) if milp else np.nan,
                "fallbacks": sum(p.fallback for p, _ in pairs) if milp else 0,
            })
        for (pm, _), (pp, pr_ev) in zip(results[name]["minimum-hop"], results[name]["proposed"]):
            unit = site.units[pm.uid]
            c_mh, c_pr = unit.candidates[pm.site_k], unit.candidates[pp.site_k]
            unit_rows.append({
                "site": name, "unit": pm.uid, "meters": unit.N, "candidates": unit.C,
                "minimum-hop pole": site.poles["POLE_NO"].iloc[c_mh],
                "proposed pole": site.poles["POLE_NO"].iloc[c_pr],
                "same pole": bool(c_mh == c_pr),
                "pole distance (m)": float(np.linalg.norm(site.pole_xy[c_mh] - site.pole_xy[c_pr])),
                "mean delta at min-hop pole": unit.delta[:, pm.site_k].mean(),
                "mean delta at proposed pole": unit.delta[:, pp.site_k].mean(),
                "proposed MIP start": pp.start,
                "proposed status": pp.status,
                "proposed R": pr_ev["redundant"], "proposed R lower bound": pp.primary_lb,
            })
    return pd.DataFrame(rows), pd.DataFrame(unit_rows)


summary, unit_comparison = summarize(sites, results)
RESULTS_DIR.mkdir(parents=True, exist_ok=True)
summary.to_csv(RESULTS_DIR / "planning_summary.csv", index=False)
unit_comparison.to_csv(RESULTS_DIR / "unit_site_comparison.csv", index=False)

with pd.option_context("display.max_columns", None, "display.width", 250, "display.float_format", "{:.3f}".format):
    print(summary.to_string(index=False))
    print("\nProposed vs minimum-hop DCU pole per planning unit:")
    print(unit_comparison.to_string(index=False))
print(f"\nSaved {RESULTS_DIR / 'planning_summary.csv'} and {RESULTS_DIR / 'unit_site_comparison.csv'}")

# %%
def latex_escape(text: str) -> str:
    """Escape LaTeX special characters in plain text."""
    special = {"\\": r"\textbackslash{}", "&": r"\&", "%": r"\%", "$": r"\$", "#": r"\#",
               "_": r"\_", "{": r"\{", "}": r"\}", "~": r"\textasciitilde{}", "^": r"\textasciicircum{}"}
    return "".join(special.get(ch, ch) for ch in text)


def latex_table(summary: pd.DataFrame) -> str:
    """LaTeX tabular: Site (meters) | Method | DCUs | Relays | Redundant | Mean/max hops | Solve time."""
    lines = [r"\begin{tabular}{llrrrcr}", r"\hline",
             r"Site (meters) & Method & DCUs & Relays & Redundant rebroadcasts & Mean / max hops & Solve time (s) \\",
             r"\hline"]
    for i, (name, df) in enumerate(summary.groupby("site", sort=False)):
        if i:
            lines.append(r"\hline")
        for j, (_, row) in enumerate(df.iterrows()):
            site_cell = f"{latex_escape(name)} ({row['meters']})" if j == 0 else ""
            lines.append(f"{site_cell} & {latex_escape(row['method'])} & {row['DCUs']} & {row['relays']} & "
                         f"{row['redundant rebroadcasts']} & {row['mean tree hops']:.2f} / {row['max tree hops']} & "
                         f"{row['solve time (s)']:.2f} \\\\")
    lines += [r"\hline", r"\end{tabular}"]
    return "\n".join(lines)


latex = latex_table(summary)
(RESULTS_DIR / "planning_summary.tex").write_text(latex + "\n")
print(latex)

# %%
def plot_plan(site: Site, method: str, ax: plt.Axes) -> None:
    """Plot one method's plan for a site: meters, poles, DCUs, relays and parent links."""
    ax.scatter(*site.pole_xy.T, marker="s", s=14, facecolors="none", edgecolors="0.6", linewidths=0.5)
    segs, dcus, relays, others = [], [], [], []
    for plan, _ in results[site.name][method]:
        unit = site.units[plan.uid]
        pole = site.pole_xy[unit.candidates[plan.site_k]]
        dcus.append(pole)
        for i, p in enumerate(plan.parent):
            a = site.meter_xy[unit.meters[i]]
            segs.append([a, pole if p < 0 else site.meter_xy[unit.meters[p]]])
        relays.append(site.meter_xy[unit.meters[plan.relays]])
        others.append(site.meter_xy[unit.meters[~plan.relays]])
    ax.add_collection(LineCollection(segs, colors="tab:blue", linewidths=0.7, alpha=0.7))
    ax.scatter(*np.vstack(others).T, s=8, c="0.35", zorder=3)
    if sum(len(r) for r in relays):
        ax.scatter(*np.vstack(relays).T, s=26, c="tab:red", zorder=4)
    ax.scatter(*np.array(dcus).T, marker="*", s=260, c="gold", edgecolors="k", zorder=5)
    row = summary[(summary.site == site.name) & (summary.method == method)].iloc[0]
    ax.set_title(f"{method}: {row['relays']} relays, R = {row['redundant rebroadcasts']}, "
                 f"hops {row['mean tree hops']:.2f} / {row['max tree hops']}", fontsize=10)
    # Zoom to the planned meters (uncovered meters far away would squash the plot).
    planned = np.vstack([site.meter_xy[np.concatenate([u.meters for u in site.units])], np.array(dcus)])
    lo, hi = planned.min(axis=0) - 30, planned.max(axis=0) + 30
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")


legend = [Line2D([], [], marker="*", ls="", ms=14, mfc="gold", mec="k", label="DCU"),
          Line2D([], [], marker="o", ls="", c="tab:red", label="relay"),
          Line2D([], [], marker="o", ls="", ms=4, c="0.35", label="non-relay meter"),
          Line2D([], [], marker="s", ls="", mfc="none", c="0.6", label="pole"),
          Line2D([], [], c="tab:blue", label="parent link")]
for site in sites.values():
    if not site.units:
        continue
    fig, axes = plt.subplots(1, 2, figsize=(15, 7), sharex=True, sharey=True)
    for ax, m in zip(axes, ("minimum-hop", "proposed")):
        plot_plan(site, m, ax)
    axes[0].set_ylabel("y (m)")
    fig.legend(handles=legend, loc="lower center", ncol=5)
    fig.suptitle(f"Site {site.name}" + (f" ({len(site.uncovered)} uncovered meters not shown)" if len(site.uncovered) else ""))
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(RESULTS_DIR / f"{site.name}_plans.png", dpi=200)
    plt.show()

# %% [markdown]
# ## Part 4 — Export for simulation
#
# For each site, `results/<site>_plans.json` contains the planning parameters, the meters and
# poles in **local** coordinates (no latitude/longitude and no UTM origin), the planning units,
# and for every method the DCU pole, relay set, parent and TTL of each meter.
#
# **TTL.** A relay forwards a message only if its received TTL is at least 2, and decrements it.
# A message sent with TTL $t$ along a path of $h$ hops arrives at hop $k$ with TTL $t - k + 1$;
# the last relay (hop $h-1$) must receive TTL $\ge 2$, i.e. $t \ge h$. With $t = h$ the DCU
# receives TTL 1. So the TTL of each meter is set to its **tree hop count** $h_i$, the smallest TTL
# that delivers along its planned path.

# %%
def export_site(site: Site, results: dict, path: Path) -> None:
    """Write the planning result of one site as JSON for the simulation notebook."""
    meter_ids = site.meters["DEVICE_NO"].tolist()
    pole_ids = site.poles["POLE_NO"].tolist()
    doc = {
        "site": site.name,
        "parameters": {
            "d_max_m": site.d_max, "d_max_computed_m": D_MAX_COMPUTED_M,
            "p_eirp_dbm": P_EIRP_DBM, "p_sens_dbm": P_SENS_DBM, "fade_margin_db": FADE_MARGIN_DB,
            "path_loss_exponent": PATH_LOSS_EXP, "d0_m": D0_M, "pl_d0_db": PL_D0_DB, "freq_hz": FREQ_HZ,
            "shadowing_sigma_db": SHADOWING_SIGMA_DB, "ttl_max": TTL_MAX, "utm_zone": site.zone,
        },
        "meters": [{"id": mid, "x": round(float(x), 3), "y": round(float(y), 3)}
                   for mid, (x, y) in zip(meter_ids, site.meter_xy)],
        "poles": [{"id": pid, "x": round(float(x), 3), "y": round(float(y), 3)}
                  for pid, (x, y) in zip(pole_ids, site.pole_xy)],
        "uncovered_meter_ids": [meter_ids[i] for i in site.uncovered],
        "planning_units": [{"unit": u.uid, "meter_ids": [meter_ids[i] for i in u.meters],
                            "candidate_pole_ids": [pole_ids[c] for c in u.candidates]} for u in site.units],
        "methods": {},
    }
    for m in METHODS:
        units = []
        for plan, ev in results[site.name][m]:
            unit = site.units[plan.uid]
            dcu = pole_ids[unit.candidates[plan.site_k]]
            meters_out = []
            for i in range(unit.N):
                p = int(plan.parent[i])
                meters_out.append({
                    "id": meter_ids[unit.meters[i]],
                    "parent": dcu if p < 0 else meter_ids[unit.meters[p]],
                    "parent_is_dcu": p < 0,
                    "relay": bool(plan.relays[i]),
                    "ttl": int(ev["tree_hops"][i]),
                    "flooding_hops": int(ev["flooding_hops"][i]),
                })
            units.append({"unit": unit.uid, "dcu_pole_id": dcu,
                          "relay_ids": [meter_ids[unit.meters[i]] for i in np.flatnonzero(plan.relays)],
                          "redundant_rebroadcasts": ev["redundant"],
                          "solver_status": plan.status, "meters": meters_out})
        doc["methods"][m] = {"units": units}
    path.write_text(json.dumps(doc, indent=1, ensure_ascii=False))


for site in sites.values():
    out = RESULTS_DIR / f"{site.name}_plans.json"
    export_site(site, results, out)
    # Round-trip check: TTL equals the tree depth recomputed from the exported parents.
    doc = json.loads(out.read_text())
    for m, content in doc["methods"].items():
        for u in content["units"]:
            par = {r["id"]: (None if r["parent_is_dcu"] else r["parent"]) for r in u["meters"]}
            for r in u["meters"]:
                d, v = 0, r["id"]
                while v is not None:
                    d, v = d + 1, par[v]
                assert d == r["ttl"], f"{out}: TTL mismatch for {r['id']} ({m})"
    print(f"Wrote {out} ({out.stat().st_size / 1024:.1f} kB)")

print("\nDone.")
