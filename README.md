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

`notebooks/mesh_simulation.ipynb` is a discrete-event simulator (plain Python, `heapq`) of Bluetooth
Mesh over the advertising bearer. It evaluates the plans in `<site>_plans.json`. Each meter sends a
200-byte reading every interval as 18 SAR segments. The segments are flooded through the planned
relays and acknowledged by the DCU, following the Mesh Protocol 1.1 SAR rules (SAR Transmitter and
SAR Receiver states, including the retransmission counters with and without progress). All transmissions of a site share the three advertising
channels, so they can collide. The notebook explains the model and every assumption, runs six
deterministic validation tests and a reproducibility check before the main runs, and uses common
random numbers across methods.

## Usage

1. Upload the plan files to `/content/results/<site>_plans.json`.
2. Edit the configuration cell if needed (for example `SITES`, `INTERVALS_MIN = [15, 5, 1]`, `N_RUNS`,
   `OFFSET_MODE`, `COLLISION_MODEL`, `N_WORKERS`), then choose *Runtime → Run all*.

The simulations run in `N_WORKERS` processes (default 2), shortest reporting interval first; each
process takes the next simulation from the queue when it finishes one. Results are identical to a
single-process run, which the notebook checks before the main runs.

## Outputs (`results/`)

| File | Content |
|---|---|
| `sim_runs.csv` | One row per site, method, interval and run: PDR, segment delivery ratio, latency (mean, median, p95), transmissions per reading by type, collisions, SAR rounds |
| `sim_summary.csv` | Mean and 95 % confidence interval over runs per site, method and interval |
| `sim_table.tex` | LaTeX `tabular` (Site, Method, PDR, Collisions, Latency) per interval |
| `sim_per_meter_runs.csv`, `sim_per_meter.csv` | Per-meter delivered readings, per run and pooled |
| `sim_metadata.json` | Configuration and package versions |
| `figures/` | PDR by method, PDR and latency against the interval, per-meter PDR maps (local coordinates) |

`src/mesh_simulation.py` is the jupytext source of the notebook. Regenerate the notebook with
`jupytext --to ipynb -o notebooks/mesh_simulation.ipynb src/mesh_simulation.py`.
