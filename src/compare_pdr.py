"""Count how often one method beats another on PDR in the same Monte Carlo iteration.

An iteration is one (site, interval_min, run) group. Within each iteration the `pdr` of
method A is compared with the `pdr` of method B; the result is a win, loss or tie for A.

Usage:
    python src/compare_pdr.py sim_runs.csv
    python src/compare_pdr.py sim_runs.csv --a Proposed --b "Min-hop + min-relay"
"""

import argparse

import pandas as pd

ITERATION_KEYS = ["site", "interval_min", "run"]


def compare_pdr(df, method_a, method_b, keys=ITERATION_KEYS, tol=1e-12):
    """Return one row per iteration with both PDRs, their difference and the outcome for A."""
    keys = [k for k in keys if k in df.columns]
    sub = df[df["method"].isin([method_a, method_b])]
    wide = sub.pivot_table(index=keys, columns="method", values="pdr", aggfunc="first")
    # Keep only iterations where both methods have a result.
    wide = wide.dropna(subset=[method_a, method_b]).reset_index()
    wide["diff"] = wide[method_a] - wide[method_b]
    wide["outcome"] = "tie"
    wide.loc[wide["diff"] > tol, "outcome"] = "win"
    wide.loc[wide["diff"] < -tol, "outcome"] = "loss"
    return wide


def summarize(wide, by=None):
    """Count wins, losses and ties, overall or per group (e.g. by interval_min)."""
    if by:
        counts = wide.groupby(by)["outcome"].value_counts().unstack(fill_value=0)
    else:
        counts = wide["outcome"].value_counts().to_frame().T
    counts = counts.reindex(columns=["win", "loss", "tie"], fill_value=0)
    counts["total"] = counts.sum(axis=1)
    counts["win_rate"] = counts["win"] / counts["total"]
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("csv", help="Monte Carlo results CSV")
    parser.add_argument("--a", default="Proposed", help="method whose wins are counted")
    parser.add_argument("--b", default="Min-hop + min-relay", help="method compared against")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)
    wide = compare_pdr(df, args.a, args.b)
    if wide.empty:
        raise SystemExit(f"No iterations contain both {args.a!r} and {args.b!r}.")

    pd.set_option("display.width", 200)
    print(f"PDR: {args.a!r} vs {args.b!r}\n")
    print("Per iteration:")
    print(wide.to_string(index=False))
    print("\nPer interval_min:")
    print(summarize(wide, by=["site", "interval_min"]).to_string())
    print("\nOverall:")
    total = summarize(wide).iloc[0]
    print(f"  {args.a} wins  : {int(total['win'])}")
    print(f"  {args.a} loses : {int(total['loss'])}")
    print(f"  ties           : {int(total['tie'])}")
    print(f"  iterations     : {int(total['total'])}")
    print(f"  win rate       : {total['win_rate']:.1%}")


if __name__ == "__main__":
    main()
