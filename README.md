# DCU site and relay-set planning for Bluetooth Mesh smart metering

`notebooks/dcu_relay_planning.ipynb` is a Google Colab notebook that picks, for each site, the
data concentrator unit (DCU) pole and the set of relay meters for a Bluetooth Mesh smart
metering network. The proposed method is a single MILP (solved with Gurobi) that minimises
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

The MILPs are solved with Gurobi under an academic Web License Service (WLS) license. Add the
`WLSACCESSID`, `WLSSECRET` and `LICENSEID` values from your `gurobi.lic` as Colab secrets (key
icon in the left sidebar, notebook access on) before running. Outside Colab, set them as
environment variables.

To try the notebook without data, set `GENERATE_DEMO_SITE = True`.

## Outputs (`results/`)

| File | Content |
|---|---|
| `planning_summary.csv` | Per site and method: DCUs, relays, redundant rebroadcasts, tree and flooding hops, solve time, MILP status and gap |
| `unit_site_comparison.csv` | Per planning unit: Proposed pole vs minimum-hop pole and their distance, and whether Proposed and Min-hop + min-relay select the same relays |
| `planning_summary.tex` | LaTeX `tabular` of the summary |
| `planning_plans.pdf` | IEEE two-column figure (7.16 in wide, Arial embedded): per site, Min-hop + min-relay next to Proposed, with parent links |
| `planning_coverage.pdf` | Same layout without links; transparent d_max circles around the DCU (yellow) and relays (pink) |
| `<site>_plans.json` | Parameters, local coordinates, planning units and each method's plan (DCU, relays, parents, TTL per meter), read by the simulation notebook |

## Source

`src/dcu_relay_planning.py` is the same notebook in jupytext percent format, which is easier to
diff. Regenerate the notebook after editing it:

```
pip install jupytext
jupytext --to ipynb -o notebooks/dcu_relay_planning.ipynb src/dcu_relay_planning.py
```
