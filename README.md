# DCU site and relay-set planning for Bluetooth Mesh smart metering

`notebooks/dcu_relay_planning.ipynb` is a Google Colab notebook that picks, for each site, the
data concentrator unit (DCU) pole and the set of relay meters for a Bluetooth Mesh smart
metering network. The proposed method is a single MILP (solved with HiGHS) that minimises
redundant rebroadcasts under managed flooding and uses hop count only to break ties. It is
compared with two baselines at the minimum-hop site: **Min-hop + min-relay** (the same MILP with
the site fixed and every relay weighted 1) and **All-relay** (every meter relays).

## Usage

1. Open the notebook in Colab.
2. Upload one folder per site:
   ```
   /content/sites/<site_name>/meters.csv       columns: DEVICE_NO, LATITUDE, LONGITUDE
   /content/sites/<site_name>/candidates.csv   columns: POLE_NO, LATITUDE, LONGITUDE
   ```
3. Edit the configuration cell if needed (for example `SITES`, `TIME_LIMIT_S`), then choose
   *Runtime → Run all*.

To try the notebook without data, set `GENERATE_DEMO_SITE = True`.

## Outputs (`results/`)

| File | Content |
|---|---|
| `planning_summary.csv` | Per site and method: DCUs, relays, redundant rebroadcasts, tree and flooding hops, solve time, MILP status and gap |
| `unit_site_comparison.csv` | Per planning unit: Proposed pole vs minimum-hop pole and their distance, and whether Proposed and Min-hop + min-relay select the same relays |
| `planning_summary.tex` | LaTeX `tabular` of the summary |
| `<site>_plans.png` | Min-hop + min-relay plan next to the Proposed plan |
| `<site>_plans.json` | Parameters, local coordinates, planning units and each method's plan (DCU, relays, parents, TTL per meter), read by the simulation notebook |

## Source

`src/dcu_relay_planning.py` is the same notebook in jupytext percent format, which is easier to
diff. Regenerate the notebook after editing it:

```
pip install jupytext
jupytext --to ipynb -o notebooks/dcu_relay_planning.ipynb src/dcu_relay_planning.py
```

# Simulation of the plans

`src/mesh_simulation.py` is a discrete-event simulator (plain Python, `heapq`) of Bluetooth Mesh
over the advertising bearer, run from the terminal. It evaluates the plans in `<site>_plans.json`.
Each meter sends a 200-byte reading every interval as 18 SAR segments. The segments are flooded
through the planned relays and acknowledged by the DCU, following the Mesh Protocol 1.1 SAR rules
(SAR Transmitter and SAR Receiver states, including the retransmission counters with and without
progress). All transmissions of a site share the three advertising channels, so they can collide.
The model and every assumption are explained in the comments of the script. Before the main runs it
runs seven deterministic validation tests, a reproducibility check and a check that the worker
processes give the same results as a single process. After the runs it checks whether the number of
Monte Carlo runs is enough to tell the methods apart.

## Usage

One site and one reporting interval (in minutes) per call:

```
python src/mesh_simulation.py /path/to/<site>_plans.json 15
python src/mesh_simulation.py /path/to/<site>_plans.json 5 --runs 100 --workers 16 --out /some/folder
```

| Option | Default |
|---|---|
| `--runs` | 100 Monte Carlo runs |
| `--workers` | every CPU; the simulations (runs x methods) are spread over these processes |
| `--out` | `/workspace/sim_results/<site>_<interval>/` |

Every other parameter (traffic, radio, bearer, SAR) is in the configuration section at the top of
the script. Results do not depend on the number of workers: every simulation rebuilds its random
draws from its own seeds.

## Outputs

| File | Content |
|---|---|
| `sim_runs.csv` | One row per method and run: PDR, segment delivery ratio, latency (mean, median, p95), transmissions per reading by type, collisions, SAR rounds |
| `sim_summary.csv` | Mean and 95 % confidence interval over runs per method |
| `sim_run_adequacy.csv` | Per pair of methods and metric: mean per-run difference, its 95 % CI, and whether the number of runs is enough |
| `sim_table.tex` | LaTeX `tabular` (Site, Method, PDR, Collisions, Latency) |
| `sim_per_meter_runs.csv`, `sim_per_meter.csv` | Per-meter delivered readings, per run and pooled |
| `sim_metadata.json` | Arguments, configuration and package versions |
| `figures/` | PDR by method, per-meter PDR map (local coordinates) |

`notebooks/mesh_simulation.ipynb` is the earlier notebook version of the simulator. It is no longer
updated; use the script.
