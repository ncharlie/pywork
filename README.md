# DCU site and relay-set planning for Bluetooth Mesh smart metering

`notebooks/dcu_relay_planning.ipynb` is a Google Colab notebook that picks, for each site, the
data concentrator unit (DCU) pole and the set of relay meters for a Bluetooth Mesh smart
metering network. The proposed method is a single MILP (solved with HiGHS) that minimises
redundant rebroadcasts under managed flooding and uses hop count only to break ties. It is
compared with the minimum-hop site baseline, an all-relay baseline and an ablation.

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
| `unit_site_comparison.csv` | Per planning unit: proposed pole vs minimum-hop pole and the distance between them |
| `planning_summary.tex` | LaTeX `tabular` of the summary |
| `<site>_plans.png` | Minimum-hop plan next to the proposed plan |
| `<site>_plans.json` | Parameters, local coordinates, planning units and each method's plan (DCU, relays, parents, TTL per meter), read by the simulation notebook |

## Source

`src/dcu_relay_planning.py` is the same notebook in jupytext percent format, which is easier to
diff. Regenerate the notebook after editing it:

```
pip install jupytext
jupytext --to ipynb -o notebooks/dcu_relay_planning.ipynb src/dcu_relay_planning.py
```
