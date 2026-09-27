"""Discrete-event simulation of a Bluetooth Mesh smart metering network.

Evaluates the DCU placements and relay sets produced by the planning notebook
(``dcu_relay_planning.ipynb``) with a packet-level discrete-event simulator of Bluetooth Mesh over the
advertising bearer. Every smart meter periodically sends a 200-byte reading to the data concentrator
unit (DCU) of its planning unit. The reading is split into segments (SAR), each segment is flooded
through the relays chosen by the plan, and the DCU acknowledges the reading. All transmissions of a
whole site share the three advertising channels, so they can collide.

Three plans are compared per site: Proposed, Min-hop + min-relay and All-relay. The main metric is
the packet delivery ratio (PDR): the fraction of readings the DCU reassembles completely.

Usage::

    python src/mesh_simulation.py
    python src/mesh_simulation.py --intervals 15 5 --runs 50 --workers 16

Without arguments it simulates every ``<site>_plans.json`` in ``/workspace/sim_results`` for the
reporting intervals 1, 5, 10 and 15 min (one job per site and interval), 100 Monte Carlo runs each.
The outputs of a job go to ``/workspace/sim_results/<site>_<interval>/``, and ``summary_all.csv``
in the same folder collects the summaries of all jobs. The simulations (runs x methods) are spread
over ``--workers`` processes (default: every CPU). Every simulation rebuilds its random draws from
its own seeds, so the results do not depend on the number of workers.

Before the main runs of each site the script runs seven deterministic validation tests, and before
each job a reproducibility check (same seed twice) and a check that the worker pool gives the same
results as a single process; it stops if any fails.

Sections of this file: configuration; 0 setup and derived constants; 1 model and assumptions;
2 plan loader; 3 propagation model and random streams; 4 simulator; 5 validation tests; 6 running
the simulations; 7 results (CSV, summary with 95 % confidence intervals, check of the number of
runs, LaTeX table, plots); command-line entry point.

Coordinates in the plan files are local (metres) and confidential, so no plot uses a basemap.
"""

# =============================== CONFIGURATION ===============================
# Every tunable parameter lives here. The plan file, the reporting interval, the number of runs, the
# number of worker processes can be changed on the command line.
# Radio parameters set to None are taken from the "parameters" block of the plan file.
from pathlib import Path

# --- Input / output -------------------------------------------------------------
PLANS_DIR = Path("/workspace/sim_results")    # default: every <site>_plans.json in this folder
INTERVALS_MIN = [1, 5, 10, 15]                # default reporting intervals (min), one job each
OUTPUT_ROOT = Path("/workspace/sim_results")  # outputs: OUTPUT_ROOT/<site>_<interval>/
METHODS: list[str] = ["Proposed", "Min-hop + min-relay", "All-relay"]  # order used in tables/plots

# --- Traffic ----------------------------------------------------------------------
OFFSET_MODE = "random"               # "random" (uniform first offset) or "synchronized" (worst case)
JITTER_S = 1.0                       # per-report jitter, uniform in [0, JITTER_S]
PAYLOAD_BYTES = 200                  # meter data per reading
N_PERIODS = 20                       # measured reporting periods per run
WARMUP_PERIODS = 1                   # simulated before the measured periods, excluded from statistics
COOLDOWN_PERIODS = 1                 # simulated after them (keeps the load on), excluded from statistics
N_RUNS = 100                         # default Monte Carlo runs (--runs); methods share random draws
BASE_SEED = 20240501

# --- Radio and propagation (None = value from the plan file) ------------------------
EIRP_DBM: float | None = None
SENSITIVITY_DBM: float | None = None
PATH_LOSS_EXP: float | None = None
PL_D0_DB: float | None = None
D0_M: float | None = None
SHADOWING_SIGMA_DB: float | None = None
SHADOWING_ENABLED = True             # False = deterministic path loss (sigma = 0)
RX_MARGIN_DB = 2.0                   # RX threshold = sensitivity + margin
INTERFERENCE_THRESHOLD_DBM: float | None = None  # None = sensitivity
COLLISION_MODEL = "any_overlap"      # "any_overlap" or "sinr"
CAPTURE_DB = 8.0                     # "sinr" only: required signal-to-interference ratio

# --- Advertising bearer --------------------------------------------------------------
ADV_PDU_OCTETS = 39                  # advertising PDU on air (all network PDUs, incl. acknowledgments)
PHY_OVERHEAD_OCTETS = 1 + 4 + 3      # preamble + access address + CRC (LE 1M PHY)
US_PER_OCTET = 8                     # LE 1M PHY: 1 Mbit/s
T_INTER_PDU_MS = (1.0, 2.0)          # start-to-start spacing of the ch37/38/39 PDUs, uniform
NETWORK_TRANSMIT_COUNT = 1           # advertising events per originated network PDU
RELAY_RETRANSMIT_COUNT = 1           # advertising events per relayed network PDU
REPETITION_MS = 30.0                 # start-to-start spacing of repeated advertising events
RELAY_BACKOFF_MS = (0.0, 20.0)       # relay waits uniformly in this range before rebroadcasting
ACK_DELAY_MS = (0.0, 0.0)            # DCU delay before an acknowledgment triggered by a segment, uniform
SCAN_INTERVAL_MS = 20.0              # continuous scanning, channel switches 37 -> 38 -> 39 every interval
HALF_DUPLEX_SCOPE = "pdu"            # "pdu": deaf while sending a PDU; "adv_event": deaf for the whole event

# --- Segmentation and reassembly (SAR): Mesh Protocol 1.1 SAR Transmitter / Receiver states --
# Values are given in physical units; each must be representable by its state field (checked below).
OPCODE_BYTES = 1
TRANSMIC_BYTES = 4
SEGMENT_DATA_BYTES = 12              # upper transport bytes per segment (segmented access message)
# SAR Transmitter
SAR_SEGMENT_INTERVAL_MS = 60.0       # SAR Segment Interval Step: (step + 1) x 10 ms
SAR_UNICAST_RETRANSMISSIONS_COUNT = 2                   # retransmissions (3 attempts in total)
SAR_UNICAST_RETRANSMISSIONS_WITHOUT_PROGRESS_COUNT = 2  # retransmissions without a progressing ack
SAR_UNICAST_RETRANS_INTERVAL_STEP_MS = 200.0       # (step + 1) x 25 ms; default field 0x07
SAR_UNICAST_RETRANS_INTERVAL_INCREMENT_MS = 50.0   # (increment + 1) x 25 ms
# SAR Receiver
SAR_SEGMENTS_THRESHOLD = 3           # acks are retransmitted only for messages with SegN above this
SAR_ACK_DELAY_INCREMENT = 2.5        # SAR Acknowledgment Delay Increment: field + 1.5 (segment intervals)
SAR_RECEIVER_SEGMENT_INTERVAL_MS = 60.0  # SAR Receiver Segment Interval Step: (step + 1) x 10 ms
SAR_ACK_RETRANSMISSIONS_COUNT = 0    # extra copies of each acknowledgment (if SegN > threshold)
SAR_DISCARD_TIMEOUT_S = 10.0         # SAR Discard Timeout: (value + 1) x 5 s

# --- Output ---------------------------------------------------------------------------
LATEX_COLLISION_METRIC = "collisions_relay_dcu_per_reading"  # column shown as "Collisions" in LaTeX
FIG_DPI = 200

# --- Check of the number of runs -------------------------------------------------------------
# For every pair of methods, the per-run difference of each metric below is averaged over the runs
# with a 95 % t confidence interval. The number of runs is enough for a metric if the interval
# excludes zero and its half-width is at most ADEQUACY_MAX_RELATIVE_HALF_WIDTH x |mean difference|.
ADEQUACY_METRICS = ["pdr", "latency_mean_ms", "collisions_relay_dcu_per_reading"]
ADEQUACY_MAX_RELATIVE_HALF_WIDTH = 0.5

# ## Part 0 — Setup and derived constants
#
# Only the Python standard library, `numpy`, `pandas` and `matplotlib` are used. The event queue is a
# `heapq` binary heap; the simulations run in worker processes (`concurrent.futures`).
#
# **Time unit.** The simulator keeps time as an **integer number of microseconds**. The airtime of
# one PDU (376 µs) is exact, overlap tests have no floating-point ties, and results are bit-for-bit
# reproducible.

import argparse
import dataclasses
import heapq
import importlib.metadata
import json
import math
import os
import platform
import random
import sys
import time
import zlib
from collections import deque
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from typing import Callable

import matplotlib

matplotlib.use("Agg")                   # no display: figures are only saved
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np
import pandas as pd
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D


def package_version(name: str) -> str:
    """Return the installed version of a distribution package, or 'unknown'."""
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return "unknown"


VERSIONS = {"python": sys.version.split()[0], "numpy": np.__version__, "pandas": pd.__version__,
            "matplotlib": matplotlib.__version__, "platform": platform.platform()}
assert OFFSET_MODE in ("random", "synchronized"), OFFSET_MODE
assert COLLISION_MODEL in ("any_overlap", "sinr"), COLLISION_MODEL
assert HALF_DUPLEX_SCOPE in ("pdu", "adv_event"), HALF_DUPLEX_SCOPE
assert NETWORK_TRANSMIT_COUNT >= 1 and RELAY_RETRANSMIT_COUNT >= 1


def ms(x: float) -> int:
    """Convert milliseconds to integer microseconds (the simulator's time unit)."""
    return int(round(x * 1000))


AIRTIME_US = (PHY_OVERHEAD_OCTETS + ADV_PDU_OCTETS) * US_PER_OCTET
UPPER_TRANSPORT_BYTES = PAYLOAD_BYTES + OPCODE_BYTES + TRANSMIC_BYTES
N_SEGMENTS = math.ceil(UPPER_TRANSPORT_BYTES / SEGMENT_DATA_BYTES)
if N_SEGMENTS > 32:
    raise ValueError(f"{N_SEGMENTS} segments needed, but SegN has 5 bits (at most 32 segments)")
if int(ms(T_INTER_PDU_MS[0])) < AIRTIME_US:
    raise ValueError("T_INTER_PDU_MS is start-to-start and must exceed the PDU airtime")


def check_sar_field(name: str, value: float, unit: float, offset: float, bits: int) -> int:
    """Return the state-field value encoding ``value`` = (field + offset) x unit, or raise."""
    field_value = value / unit - offset
    if abs(field_value - round(field_value)) > 1e-9 or not 0 <= round(field_value) < 2 ** bits:
        raise ValueError(f"{name} = {value:g} cannot be encoded in the {bits}-bit Mesh 1.1 SAR state field")
    return int(round(field_value))


SAR_FIELDS = {
    "SAR Segment Interval Step": check_sar_field("SAR_SEGMENT_INTERVAL_MS", SAR_SEGMENT_INTERVAL_MS, 10, 1, 4),
    "SAR Unicast Retransmissions Count": check_sar_field("SAR_UNICAST_RETRANSMISSIONS_COUNT", SAR_UNICAST_RETRANSMISSIONS_COUNT, 1, 0, 4),
    "SAR Unicast Retransmissions Without Progress Count": check_sar_field(
        "SAR_UNICAST_RETRANSMISSIONS_WITHOUT_PROGRESS_COUNT", SAR_UNICAST_RETRANSMISSIONS_WITHOUT_PROGRESS_COUNT, 1, 0, 4),
    "SAR Unicast Retransmissions Interval Step": check_sar_field(
        "SAR_UNICAST_RETRANS_INTERVAL_STEP_MS", SAR_UNICAST_RETRANS_INTERVAL_STEP_MS, 25, 1, 4),
    "SAR Unicast Retransmissions Interval Increment": check_sar_field(
        "SAR_UNICAST_RETRANS_INTERVAL_INCREMENT_MS", SAR_UNICAST_RETRANS_INTERVAL_INCREMENT_MS, 25, 1, 4),
    "SAR Segments Threshold": check_sar_field("SAR_SEGMENTS_THRESHOLD", SAR_SEGMENTS_THRESHOLD, 1, 0, 5),
    "SAR Acknowledgment Delay Increment": check_sar_field("SAR_ACK_DELAY_INCREMENT", SAR_ACK_DELAY_INCREMENT, 1, 1.5, 3),
    "SAR Receiver Segment Interval Step": check_sar_field(
        "SAR_RECEIVER_SEGMENT_INTERVAL_MS", SAR_RECEIVER_SEGMENT_INTERVAL_MS, 10, 1, 4),
    "SAR Acknowledgment Retransmissions Count": check_sar_field(
        "SAR_ACK_RETRANSMISSIONS_COUNT", SAR_ACK_RETRANSMISSIONS_COUNT, 1, 0, 2),
    "SAR Discard Timeout": check_sar_field("SAR_DISCARD_TIMEOUT_S", SAR_DISCARD_TIMEOUT_S, 5, 1, 4),
}


def unicast_retrans_interval_ms(ttl: int) -> float:
    """Mesh 1.1 SAR unicast retransmissions timer: step + increment x (TTL - 1), or step if TTL = 0."""
    return SAR_UNICAST_RETRANS_INTERVAL_STEP_MS + (SAR_UNICAST_RETRANS_INTERVAL_INCREMENT_MS * (ttl - 1) if ttl > 0 else 0)


def ack_timer_ms(seg_n: int) -> float:
    """Mesh 1.1 SAR acknowledgment timer: min(SegN + 0.5, delay increment) x receiver segment interval."""
    return min(seg_n + 0.5, SAR_ACK_DELAY_INCREMENT) * SAR_RECEIVER_SEGMENT_INTERVAL_MS


def print_setup(intervals: list[int], n_runs: int, workers: int) -> None:
    """Print the versions, derived constants and SAR timers of this job."""
    print(VERSIONS)
    print(f"PDU airtime                 : ({PHY_OVERHEAD_OCTETS} + {ADV_PDU_OCTETS}) octets x {US_PER_OCTET} us = {AIRTIME_US} us")
    print(f"Upper transport PDU         : {PAYLOAD_BYTES} + {OPCODE_BYTES} + {TRANSMIC_BYTES} = {UPPER_TRANSPORT_BYTES} bytes")
    print(f"Segments per reading        : ceil({UPPER_TRANSPORT_BYTES} / {SEGMENT_DATA_BYTES}) = {N_SEGMENTS}")
    print(f"Nominal round duration      : {N_SEGMENTS} segments x {SAR_SEGMENT_INTERVAL_MS:g} ms = {N_SEGMENTS * SAR_SEGMENT_INTERVAL_MS:g} ms")
    print("Mesh Protocol 1.1 SAR states (field value -> physical value):")
    for name, value in SAR_FIELDS.items():
        print(f"  {name:<52} {value:>3}")
    print(f"  segment interval {SAR_SEGMENT_INTERVAL_MS:g} ms; up to {SAR_UNICAST_RETRANSMISSIONS_COUNT} retransmissions "
          f"({SAR_UNICAST_RETRANSMISSIONS_COUNT + 1} attempts), at most {SAR_UNICAST_RETRANSMISSIONS_WITHOUT_PROGRESS_COUNT} "
          f"without progress; discard timeout {SAR_DISCARD_TIMEOUT_S:g} s")
    print(f"  acknowledgment timer for a {N_SEGMENTS}-segment message: min({N_SEGMENTS - 1} + 0.5, {SAR_ACK_DELAY_INCREMENT:g}) x "
          f"{SAR_RECEIVER_SEGMENT_INTERVAL_MS:g} ms = {ack_timer_ms(N_SEGMENTS - 1):g} ms; transmissions per timer acknowledgment: "
          f"{1 + (SAR_ACK_RETRANSMISSIONS_COUNT if N_SEGMENTS - 1 > SAR_SEGMENTS_THRESHOLD else 0)}")
    for ttl in (1, 2, 3):
        print(f"  TTL {ttl}: unicast retransmissions timer {unicast_retrans_interval_ms(ttl):g} ms")
    print(f"Intervals {intervals} min, offset mode '{OFFSET_MODE}', {WARMUP_PERIODS} + {N_PERIODS} + "
          f"{COOLDOWN_PERIODS} periods, {n_runs} runs, {workers} worker process(es)")


# ## Part 1 — Model and assumptions
#
# ### 1.1 Propagation and reception
#
# The received power from node $i$ at node $j$ follows the log-distance model with log-normal
# shadowing,
#
# $$P_{rx}(i,j) = P_{EIRP} - PL(d_0) - 10\,n\,\log_{10}\!\frac{\max(d_{ij}, d_0)}{d_0} + X_{ij},
# \qquad X_{ij} = X_{ji} \sim \mathcal N(0, \sigma^2).$$
#
# * $X_{ij}$ is drawn **once per link and run** (meters and DCUs do not move) and is symmetric.
#   Draws are independent between links, including links between co-located nodes.
# * The distance is clamped to $d_0$: several meters in the data share coordinates (e.g. two meters
#   in one cabinet), and the model is not defined below $d_0$.
# * The receive antenna gain is 0 dBi (EIRP already contains the transmit antenna gain).
# * **All node pairs** are considered, not only links within $d_{max}$: shadowing can create links
#   the planner did not assume, or break planned ones. $P_{rx}$ is precomputed for all pairs.
# * A PDU can be received only if $P_{rx} \ge$ `RX_THRESHOLD_DBM` = sensitivity + `RX_MARGIN_DB`
#   (at the sensitivity itself about 30.8 % of packets are lost, so a 2 dB margin is used as the
#   "clean reception" point).
# * **Collision model `any_overlap`** (default): the reception fails if any other transmission on
#   the same channel overlaps it in time (even partially) and arrives with
#   $P_{rx} \ge$ `INTERFERENCE_THRESHOLD_DBM` (default: the sensitivity).
#   **`sinr`**: the reception succeeds if the signal exceeds the *sum* of all overlapping
#   same-channel signals (in mW, any power) by at least `CAPTURE_DB`. Thermal noise is covered by
#   the RX threshold. Partial overlap counts as full overlap in both models.
# * The three advertising channels are orthogonal; there is no adjacent-channel interference and
#   no external 2.4 GHz interference (Wi-Fi, other BLE devices).
# * Concurrent transmissions of the same PDU by two relays are **not** constructively combined:
#   they collide like any other pair (BLE advertising has no synchronous-flooding mechanism).
#
# ### 1.2 Advertising bearer (Reno et al., 2020)
#
# * Every network PDU occupies 39 octets on air; the airtime is
#   $(1 + 4 + 39 + 3) \times 8\,\mu s = 376\,\mu s$. Segment acknowledgments use the same size,
#   which is conservative (they are shorter in practice).
# * A transmission is an **advertising event**: the PDU is sent on channels 37, 38 and 39 in
#   sequence. The start-to-start spacing of consecutive PDUs is drawn uniformly from 1–2 ms
#   (the Bluetooth Core specification bounds this start-to-start time).
# * `NETWORK_TRANSMIT_COUNT` / `RELAY_RETRANSMIT_COUNT` = number of advertising events per
#   originated / relayed PDU. Repeated events start `REPETITION_MS` after the previous one.
# * **One radio per node.** A node transmits one advertising event at a time. Transmissions that
#   become due while the radio is busy wait in a FIFO queue (no size limit) and start as soon as
#   the current event ends. Own segments, acknowledgments and relayed PDUs share this queue.
# * **Scanning** is continuous (all nodes are mains powered). The scanner listens on one channel
#   and switches 37 → 38 → 39 every `SCAN_INTERVAL_MS`. Each node has its own random scan phase,
#   drawn once per run. Channel switching takes no time. A PDU is received only if the node listens
#   on the PDU's channel for its **entire** airtime.
# * **Half-duplex.** A node that transmits during any part of a PDU does not receive it
#   (`HALF_DUPLEX_SCOPE = "pdu"`). With `"adv_event"` the node is also deaf in the 1–2 ms gaps
#   between the three PDUs of its own advertising event (some controllers behave like this).
# * Processing delays (decryption, queueing in the host) are zero.
#
# ### 1.3 Network layer and managed flooding
#
# * Each node has a **network message cache** of `(SRC, SEQ)`; a PDU already in the cache is
#   discarded and never relayed again. The cache has no size limit. A source puts its own PDUs in
#   its cache, so it ignores echoes of them.
# * **Relaying.** Only meters in the method's relay set relay. A relay rebroadcasts a new PDU only if
#   the received TTL is at least 2 and the PDU is not addressed to the relay itself. It waits
#   uniformly 0–20 ms, then transmits the PDU with TTL − 1. DCUs and non-relay meters receive but
#   never forward. There is no suppression of a pending rebroadcast when the relay hears a
#   neighbour send the same PDU.
# * **TTL.** Each meter's data segments use the TTL from the plan (its hop count in the planned
#   tree), so a TTL-1 message reaches only the meter's direct neighbours. (The specification
#   reserves TTL 1 for "may have been relayed, will not be relayed" and would use TTL 0 for a
#   single hop; in this model both behave the same.) Acknowledgments to a meter use the same TTL.
# * Messages to a node's unicast address are processed only by that node. A DCU ignores data
#   segments addressed to another DCU (they still occupy its radio and cause collisions).
#
# ### 1.4 Segmentation and reassembly (SAR)
#
# * Upper transport PDU = payload + opcode (1 byte) + TransMIC (4 bytes) = 205 bytes, carried as
#   a segmented access message of 12 bytes per segment: **18 segments**. Each segment is its own
#   network PDU with its own sequence number; it is relayed and cached independently.
#
# The SAR behaviour follows **Mesh Protocol 1.1** (§3.5.3.3 and §3.5.3.4; SAR Transmitter and SAR
# Receiver states). Where the text leaves details open, the model follows the Zephyr RTOS Bluetooth
# Mesh stack (`subsys/bluetooth/mesh/transport.c`), whose comments cite these sections. The
# configuration section gives each state in physical units and checks that it can be encoded in its
# state field; the defaults are the specification's default field values (printed above).
#
# | State | Field | Value |
# |---|---|---|
# | SAR Segment Interval Step | 0x05 | (5 + 1) × 10 = 60 ms between segments |
# | SAR Unicast Retransmissions Count | 0x02 | 2 retransmissions (3 attempts) |
# | SAR Unicast Retransmissions Without Progress Count | 0x02 | 2 retransmissions (3 attempts) without progress |
# | SAR Unicast Retransmissions Interval Step | 0x07 | (7 + 1) × 25 = 200 ms |
# | SAR Unicast Retransmissions Interval Increment | 0x01 | (1 + 1) × 25 = 50 ms |
# | SAR Segments Threshold | 0x03 | acknowledgments are repeated only if SegN > 3 |
# | SAR Acknowledgment Delay Increment | 0x01 | 1 + 1.5 = 2.5 segment intervals |
# | SAR Receiver Segment Interval Step | 0x05 | (5 + 1) × 10 = 60 ms |
# | SAR Acknowledgment Retransmissions Count | 0x00 | no repeated acknowledgments |
# | SAR Discard Timeout | 0x01 | (1 + 1) × 5 = 10 s |
#
# * **Sender (meter).** Segments are sent in order. Segment $k+1$ starts one segment interval after
#   the start of segment $k$'s advertising event, and not before segment $k$ has been transmitted
#   (the radio queue can delay it further). One pass over the unacknowledged segments is a
#   **round**; the first round sends all 18.
#   * When a round ends (one segment interval after the start of its last segment), the **SAR
#     unicast retransmissions timer** runs until
#     *start of last segment* + step + increment · (TTL − 1) (step alone if TTL = 0), with the
#     TTL of the message: 200 ms for TTL 1, 250 ms for TTL 2.
#   * Two counters limit the attempts: the total count and the without-progress count, each allowing
#     count + 1 rounds including the first one. The end of every round uses one total attempt and,
#     unless an acknowledgment arrived after the round's last segment, one without-progress attempt.
#     An acknowledgment that acknowledges at least one new segment (**progress**) resets the
#     without-progress counter. When a new round is due and either counter is used up, the sender
#     cancels the message. With the defaults 2 / 2, each segment is sent at most 3 times.
#   * An acknowledgment that acknowledges every segment ends the message (success).
#   * An acknowledgment received while the timer runs (with or without progress) stops the timer
#     and starts the next round at once (but not earlier than one segment interval after the last
#     segment). An acknowledgment received during a round marks the acknowledged segments, which the
#     round then skips; if it arrived after the round's last segment, the next round starts one
#     segment interval later instead of waiting for the timer.
#   * A round retransmits only the unacknowledged segments. They keep SeqZero but get new sequence
#     numbers, so network caches do not drop them.
#   * A meter has at most one segmented message in progress (messages to the same destination are
#     sent one at a time). A reading generated while the previous one is in progress waits in a FIFO
#     queue.
# * **Receiver (DCU).** Reassembly per `(SRC, SeqZero)`.
#   * Every **new** segment restarts the **SAR acknowledgment timer** with
#     min(SegN + 0.5, delay increment) × receiver segment interval = min(17.5, 2.5) × 60 ms =
#     150 ms, and the **SAR discard timer** (10 s). A segment that was already received restarts
#     neither. So while segments keep arriving every 60 ms the DCU stays silent; it acknowledges
#     150 ms after the last new segment if some are still missing.
#   * On expiry of the acknowledgment timer the DCU sends a Segment Acknowledgment with the bitmap
#     of the segments received so far. If SegN exceeds the segments threshold, it repeats it
#     `SAR_ACK_RETRANSMISSIONS_COUNT` times, one receiver segment interval apart (default: none).
#   * When the last missing segment arrives, the timer is cancelled and an acknowledgment with all
#     bits set is sent immediately. Because the DCU completes the message at the end of that
#     segment's PDU on one channel while the source is still sending the same segment on the next
#     channels, the acknowledgment's advertising event starts in lock-step with the source's, and
#     its copies often overlap the source's own copies (collisions at listeners, half-duplex at the
#     source). `ACK_DELAY_MS` (default 0–0 ms) adds a uniform host/processing delay before such
#     segment-triggered acknowledgments to study this; timer acknowledgments are not delayed.
#   * A segment of an already completed message is answered with an all-ones acknowledgment, but
#     at most once per delay increment × receiver segment interval (150 ms).
#   * On expiry of the discard timer the partial message is discarded; later segments of it are
#     ignored.
#   * Acknowledgments are unsegmented control messages (one network PDU), flooded and relayed like
#     any other PDU. They can collide and be lost.
# * SeqZero is represented by the full sequence number of the first segment, so it never wraps.
#
# **Check before publishing.** The field defaults above were taken from the Zephyr stack's
# configuration, which uses the specification's defaults; **confirm them against the SAR state
# default table of the Mesh Protocol 1.1 specification cited in the paper.**
#
# ### 1.5 Traffic and measurement window
#
# * `OFFSET_MODE = "random"`: meter $m$ reports at $o_m + kT + J_{m,k}$ with $o_m \sim U[0, T)$ and
#   jitter $J_{m,k} \sim U[0, \text{JITTER\_S}]$. `"synchronized"`: at $kT + J_{m,k}$ (worst case).
# * Each run simulates `WARMUP_PERIODS + N_PERIODS + COOLDOWN_PERIODS` periods. Only readings
#   generated in the `N_PERIODS` measured periods enter the statistics. The warm-up periods fill
#   the queues; the cool-down period keeps the load on while the last measured readings finish, so
#   they do not see an artificially empty network. After the last period the simulation runs until
#   no event is left.
# * The whole site is simulated at once: every planning unit and every DCU, since transmissions
#   from different units interfere.
#
# ### 1.6 Common random numbers and seeds
#
# For a given site, run index and interval, all methods see the same random draws wherever the
# events correspond: the shadowing matrix is drawn over **all meters and all poles** of the site
# (so a method whose DCU is on another pole sees the same shadowing on every shared link), and
# report times, scan phases, inter-PDU gaps and relay back-offs come from separate generators per
# purpose and per node.
#
# **Across reporting intervals** only the shadowing is shared: its seed contains the run index but
# not the interval, so run $k$ sees the same link powers at every interval, while report times,
# scan phases, gaps and back-offs are drawn independently per interval. Comparisons between
# intervals are therefore paired by run index (the dominant run-to-run variation, shadowing, cancels);
# a statistical test of the interval effect should pair runs by index rather than treat intervals as
# independent samples. The generators are seeded from
# `numpy.random.SeedSequence([BASE_SEED, crc32(site), purpose, interval, run, node, …])`; the
# scalar draws inside the event loop use Python's `random.Random` (Mersenne Twister) seeded from
# those sequences, which is fast and platform independent.
#
# ### 1.7 Metrics (per method, site, interval and run)
#
# | Metric | Definition |
# |---|---|
# | PDR | measured readings fully reassembled at their DCU / measured readings |
# | Segment delivery ratio | distinct (reading, segment) pairs received by the destination DCU / (readings × 18) |
# | Latency | first transmission of the first segment → complete reassembly (delivered readings; mean, median, 95th percentile) |
# | Transmissions per reading | network PDU transmissions (each = `…_COUNT` advertising events × 3 channels) belonging to measured readings: data segments, relayed segments, acknowledgments, relayed acknowledgments, total |
# | Collisions | PDU receptions lost to overlap (the receiver was listening on the channel, not transmitting, and $P_{rx}$ was above the RX threshold). Counted at relays and DCUs (affect delivery) and at non-relay meters separately. `…_new` counts only losses of PDUs the relay/DCU had not yet received (losses of duplicates cannot affect delivery) |
# | Half-duplex losses | receptions lost because the receiver was transmitting |
# | SAR rounds | retransmission rounds per reading, fraction of readings with at least one |
# | Sender success | fraction of readings for which the meter received a full acknowledgment |
# | Per-meter PDR | per meter, over the measured readings of the run |

# ## Part 2 — Plan loader
#
# The loader reads the `<site>_plans.json` given on the command line, checks every field the simulator
# needs, and fails with a message naming the file and the missing field. Meter and pole IDs are
# separate name spaces in the files (meter "9" and pole "9" are different devices), so they are kept
# apart internally.

@dataclass
class UnitPlan:
    """One planning unit's plan for one method."""
    unit: int
    dcu_pole: str
    relay_ids: list[str]
    meter_ttl: dict[str, int]
    meter_parent: dict[str, tuple[str, bool]]   # meter -> (parent id, parent is the DCU pole)


@dataclass
class SitePlans:
    """Everything the simulator needs about one site."""
    name: str
    path: Path
    params: dict
    meter_ids: list[str]
    meter_xy: np.ndarray       # (M, 2) local metres
    pole_ids: list[str]
    pole_xy: np.ndarray        # (P, 2) local metres
    units: list[dict]          # planning units: {"unit", "meter_ids", "candidate_pole_ids"}
    uncovered: list[str]
    methods: dict[str, list[UnitPlan]]


REQUIRED_PARAMS = {  # simulator name -> key in the "parameters" block of the plan file
    "eirp_dbm": "p_eirp_dbm", "sensitivity_dbm": "p_sens_dbm", "path_loss_exp": "path_loss_exponent",
    "pl_d0_db": "pl_d0_db", "d0_m": "d0_m", "shadowing_sigma_db": "shadowing_sigma_db",
    "d_max_m": "d_max_m", "fade_margin_db": "fade_margin_db",
}


def _require(obj: dict, key: str, where: str, path: Path):
    """Return obj[key] or raise a KeyError naming the file and location of the missing field."""
    if not isinstance(obj, dict) or key not in obj:
        raise KeyError(f"{path.name}: required field '{key}' missing in {where}")
    return obj[key]


def load_site_plans(path: Path, methods: list[str]) -> SitePlans:
    """Load and validate one ``<site>_plans.json`` file."""
    doc = json.loads(path.read_text())
    name = str(doc.get("site") or path.name.removesuffix("_plans.json"))
    raw = doc.get("parameters", doc.get("params"))
    if raw is None:
        raise KeyError(f"{path.name}: required block 'parameters' (or 'params') missing")
    params = {k: float(_require(raw, v, "'parameters'", path)) for k, v in REQUIRED_PARAMS.items()}

    def points(key: str) -> tuple[list[str], np.ndarray]:
        rows = _require(doc, key, "the top level", path)
        ids = [str(_require(r, "id", f"'{key}' entry", path)) for r in rows]
        xy = np.array([[float(_require(r, "x", f"'{key}' entry", path)),
                        float(_require(r, "y", f"'{key}' entry", path))] for r in rows], dtype=float)
        if len(set(ids)) != len(ids):
            raise ValueError(f"{path.name}: duplicate IDs in '{key}'")
        return ids, xy.reshape(-1, 2)

    meter_ids, meter_xy = points("meters")
    pole_ids, pole_xy = points("poles")
    units = _require(doc, "planning_units", "the top level", path)
    for u in units:
        for k in ("unit", "meter_ids", "candidate_pole_ids"):
            _require(u, k, "a 'planning_units' entry", path)
    uncovered = [str(x) for x in doc.get("uncovered_meter_ids", [])]
    all_methods = _require(doc, "methods", "the top level", path)

    meter_set, pole_set = set(meter_ids), set(pole_ids)
    unit_meters = {int(u["unit"]): [str(x) for x in u["meter_ids"]] for u in units}
    covered = [m for ms_ in unit_meters.values() for m in ms_]
    if len(covered) != len(set(covered)):
        raise ValueError(f"{path.name}: a meter belongs to more than one planning unit")
    if unknown := set(covered) - meter_set:
        raise ValueError(f"{path.name}: planning units reference unknown meters {sorted(unknown)[:5]}")

    plans: dict[str, list[UnitPlan]] = {}
    for m in methods:
        if m not in all_methods:
            raise KeyError(f"{path.name}: method '{m}' missing (file has {list(all_methods)})")
        mu = _require(all_methods[m], "units", f"method '{m}'", path)
        out = []
        for u in mu:
            where = f"method '{m}', unit entry"
            uid = int(_require(u, "unit", where, path))
            dcu = str(_require(u, "dcu_pole_id", where, path))
            relays = [str(x) for x in _require(u, "relay_ids", where, path)]
            rows = _require(u, "meters", where, path)
            ttl, parent, relay_flags = {}, {}, set()
            for r in rows:
                w = f"method '{m}', unit {uid}, meter entry"
                mid = str(_require(r, "id", w, path))
                ttl[mid] = int(_require(r, "ttl", w, path))
                parent[mid] = (str(_require(r, "parent", w, path)), bool(_require(r, "parent_is_dcu", w, path)))
                if r.get("relay", False):
                    relay_flags.add(mid)
            if uid not in unit_meters:
                raise ValueError(f"{path.name}: method '{m}' refers to unknown unit {uid}")
            if set(ttl) != set(unit_meters[uid]):
                raise ValueError(f"{path.name}: method '{m}', unit {uid}: meter list differs from the planning unit")
            if dcu not in pole_set:
                raise ValueError(f"{path.name}: method '{m}', unit {uid}: DCU pole '{dcu}' is not in 'poles'")
            if not set(relays) <= set(ttl):
                raise ValueError(f"{path.name}: method '{m}', unit {uid}: relay outside the unit")
            if "relay" in rows[0] and relay_flags != set(relays):
                raise ValueError(f"{path.name}: method '{m}', unit {uid}: 'relay_ids' disagrees with per-meter 'relay' flags")
            if min(ttl.values()) < 1:
                raise ValueError(f"{path.name}: method '{m}', unit {uid}: TTL must be >= 1")
            out.append(UnitPlan(uid, dcu, relays, ttl, parent))
        if {p.unit for p in out} != set(unit_meters):
            raise ValueError(f"{path.name}: method '{m}' does not plan every planning unit")
        plans[m] = sorted(out, key=lambda p: p.unit)
    return SitePlans(name, path, params, meter_ids, meter_xy, pole_ids, pole_xy, units, uncovered, plans)


def print_plans(sp: SitePlans) -> None:
    """Print a one-line description of each method's plan."""
    print(f"Site {sp.name}: {len(sp.meter_ids)} meters ({len(sp.uncovered)} uncovered), "
          f"{len(sp.pole_ids)} poles, {len(sp.units)} planning units")
    for m, ups in sp.methods.items():
        ttls = [t for up in ups for t in up.meter_ttl.values()]
        print(f"  {m:<22} DCU poles {[up.dcu_pole for up in ups]}, relays {sum(len(up.relay_ids) for up in ups)}, "
              f"TTL mean {np.mean(ttls):.2f} / max {max(ttls)}")
    if sp.uncovered:
        print(f"  note: {len(sp.uncovered)} uncovered meters have no plan and are not simulated")


# ## Part 3 — Propagation model and random streams
#
# `RadioParams` holds the radio parameters of one site after applying the overrides from the
# configuration section. `rx_power_matrix` computes $P_{rx}$ for all pairs of the site's **universe**
# (all meters followed by all poles). Each method then uses the rows and columns of its own nodes.

PURPOSE_SHADOWING, PURPOSE_TRAFFIC, PURPOSE_SCAN, PURPOSE_NODE = 1, 2, 3, 4
STREAM_GAP, STREAM_BACKOFF, STREAM_ACK_DELAY = 0, 1, 2


@dataclass(frozen=True)
class RadioParams:
    """Radio and propagation parameters of one site."""
    eirp_dbm: float
    sensitivity_dbm: float
    path_loss_exp: float
    pl_d0_db: float
    d0_m: float
    shadowing_sigma_db: float
    rx_threshold_dbm: float
    interference_threshold_dbm: float


def radio_params(site_params: dict) -> RadioParams:
    """Combine a plan file's parameters with the overrides in the configuration section."""
    def pick(override, key):
        return float(site_params[key]) if override is None else float(override)
    sens = pick(SENSITIVITY_DBM, "sensitivity_dbm")
    sigma = pick(SHADOWING_SIGMA_DB, "shadowing_sigma_db") if SHADOWING_ENABLED else 0.0
    return RadioParams(
        eirp_dbm=pick(EIRP_DBM, "eirp_dbm"), sensitivity_dbm=sens,
        path_loss_exp=pick(PATH_LOSS_EXP, "path_loss_exp"), pl_d0_db=pick(PL_D0_DB, "pl_d0_db"),
        d0_m=pick(D0_M, "d0_m"), shadowing_sigma_db=sigma, rx_threshold_dbm=sens + RX_MARGIN_DB,
        interference_threshold_dbm=sens if INTERFERENCE_THRESHOLD_DBM is None else float(INTERFERENCE_THRESHOLD_DBM))


def site_hash(name: str) -> int:
    """Stable 32-bit hash of a site name (Python's hash() is salted per process)."""
    return zlib.crc32(name.encode("utf-8"))


def seed_sequence(*keys: int) -> np.random.SeedSequence:
    """SeedSequence derived from BASE_SEED and integer keys."""
    return np.random.SeedSequence([BASE_SEED, *[int(k) for k in keys]])


def py_rng(ss: np.random.SeedSequence) -> random.Random:
    """A Python Mersenne Twister seeded with 128 bits from a SeedSequence."""
    a, b = ss.generate_state(2, dtype=np.uint64)
    return random.Random((int(a) << 64) | int(b))


def mean_rx_power(xy: np.ndarray, rp: RadioParams) -> np.ndarray:
    """Received power without shadowing for all pairs (dBm); distance clamped to d0."""
    d = np.sqrt(((xy[:, None, :] - xy[None, :, :]) ** 2).sum(-1))
    d = np.maximum(d, rp.d0_m)
    return rp.eirp_dbm - rp.pl_d0_db - 10.0 * rp.path_loss_exp * np.log10(d / rp.d0_m)


def rx_power_matrix(xy: np.ndarray, rp: RadioParams, ss: np.random.SeedSequence | None) -> np.ndarray:
    """Received power with static, symmetric shadowing (one draw per unordered pair).

    The standard normal draws fill the upper triangle in row-major order and are mirrored, so the
    matrix depends only on the seed and the universe size, not on the method.
    """
    p = mean_rx_power(xy, rp)
    if rp.shadowing_sigma_db > 0 and ss is not None:
        n = len(xy)
        iu = np.triu_indices(n, k=1)
        z = np.random.default_rng(ss).standard_normal(len(iu[0]))
        x = np.zeros((n, n))
        x[iu] = z * rp.shadowing_sigma_db
        p = p + x + x.T
    np.fill_diagonal(p, -np.inf)
    return p


def report_schedule(n_meters: int, interval_us: int, n_periods: int, ss: np.random.SeedSequence) -> np.ndarray:
    """Report times (us), shape (n_meters, n_periods), for all meters of the site universe.

    The same draws are made in both offset modes (offsets are ignored when synchronized), so the
    jitter stream is identical between modes.
    """
    rng = np.random.default_rng(ss)
    offsets = rng.integers(0, interval_us, size=n_meters, dtype=np.int64)
    jitter = rng.integers(0, int(ms(JITTER_S * 1000)) + 1, size=(n_meters, n_periods), dtype=np.int64)
    base = np.arange(n_periods, dtype=np.int64)[None, :] * interval_us
    if OFFSET_MODE == "random":
        base = base + offsets[:, None]
    return base + jitter


# ## Part 4 — Simulator
#
# ### Events
#
# | Event | Action |
# |---|---|
# | `REPORT` | a meter generates a reading; SAR starts now or after the meter's previous reading |
# | `SEG_SUBMIT` | the sender hands the next unacknowledged segment of the round to its radio queue, or ends the round and starts the retransmissions timer |
# | `TX_START` | a PDU starts on one channel: registered as ongoing on that channel and as "radio busy" at its node |
# | `TX_END` | a PDU ends: reception is evaluated at every node that could hear it |
# | `ADV_END` | an advertising event ends: the radio takes the next queued item |
# | `REPEAT` | a repeated advertising event of the same PDU is queued (count > 1) |
# | `RELAY` | a relay's back-off expires: the rebroadcast is queued |
# | `ACK_TIMER` | DCU SAR acknowledgment timer: send an acknowledgment with the current bitmap |
# | `SEG_TX_TIMER` | sender SAR unicast retransmissions timer: retransmit unacknowledged segments |
# | `DISCARD` | DCU SAR discard timer: discard the partial message |
# | `ACK_SEND` | a segment-triggered acknowledgment is queued after `ACK_DELAY_MS` (only if > 0) |
#
# Events at the same time are ordered by (priority, insertion order): `TX_END` < `ADV_END` <
# `TX_START` < others. Intervals are half-open $[s, e)$, so a PDU that starts exactly when another
# ends does not overlap it, and the order does not affect results.
#
# ### Reception at a node $v$ of a PDU sent by $u$ on channel $c$ during $[s, e)$
#
# 1. $P_{rx}(u, v) \ge$ RX threshold (precomputed candidate list per transmitter);
# 2. $v$'s scanner is on channel $c$ during all of $[s, e)$ — otherwise $v$ did not try to receive
#    (not a loss);
# 3. $v$ is not transmitting during $[s, e)$ — otherwise a **half-duplex loss**;
# 4. no interfering same-channel overlap (`any_overlap`) or enough SIR (`sinr`) — otherwise a
#    **collision**;
# 5. then the PDU passes the network cache check and is processed.
#
# Every PDU of a channel is kept in a per-channel list until no future reception can overlap it, so
# the overlap test at `TX_END` sees every transmission that overlapped the PDU at any time.

EV_TX_END, EV_ADV_END, EV_TX_START, EV_REPORT, EV_SEG_SUBMIT, EV_REPEAT, EV_RELAY, \
    EV_ACK_TIMER, EV_SEG_TX_TIMER, EV_DISCARD, EV_INJECT, EV_ACK_SEND = range(12)
PRIORITY = {EV_TX_END: 0, EV_ADV_END: 1, EV_TX_START: 2}
SEG, ACK = 0, 1
CHANNELS = (37, 38, 39)


@dataclass(frozen=True)
class ProtocolConfig:
    """Protocol, bearer and radio-decision parameters; all times in microseconds."""
    airtime_us: int
    inter_pdu_us: tuple[int, int]
    network_transmit_count: int
    relay_retransmit_count: int
    repetition_us: int
    relay_backoff_us: tuple[int, int]
    ack_delay_us: tuple[int, int]
    scan_interval_us: int
    half_duplex_scope: str
    n_segments: int
    segment_interval_us: int
    retx_count: int
    retx_without_progress_count: int
    retx_interval_step_us: int
    retx_interval_increment_us: int
    segments_threshold: int
    ack_delay_increment: float
    rx_segment_interval_us: int
    ack_retx_count: int
    discard_timeout_us: int
    rx_threshold_dbm: float
    interference_threshold_dbm: float
    collision_model: str
    capture_db: float


def protocol_config(rp: RadioParams) -> ProtocolConfig:
    """ProtocolConfig from the configuration section and a site's radio parameters."""
    return ProtocolConfig(
        airtime_us=AIRTIME_US, inter_pdu_us=(ms(T_INTER_PDU_MS[0]), ms(T_INTER_PDU_MS[1])),
        network_transmit_count=NETWORK_TRANSMIT_COUNT, relay_retransmit_count=RELAY_RETRANSMIT_COUNT,
        repetition_us=ms(REPETITION_MS), relay_backoff_us=(ms(RELAY_BACKOFF_MS[0]), ms(RELAY_BACKOFF_MS[1])),
        ack_delay_us=(ms(ACK_DELAY_MS[0]), ms(ACK_DELAY_MS[1])),
        scan_interval_us=ms(SCAN_INTERVAL_MS), half_duplex_scope=HALF_DUPLEX_SCOPE, n_segments=N_SEGMENTS,
        segment_interval_us=ms(SAR_SEGMENT_INTERVAL_MS),
        retx_count=SAR_UNICAST_RETRANSMISSIONS_COUNT,
        retx_without_progress_count=SAR_UNICAST_RETRANSMISSIONS_WITHOUT_PROGRESS_COUNT,
        retx_interval_step_us=ms(SAR_UNICAST_RETRANS_INTERVAL_STEP_MS),
        retx_interval_increment_us=ms(SAR_UNICAST_RETRANS_INTERVAL_INCREMENT_MS),
        segments_threshold=SAR_SEGMENTS_THRESHOLD, ack_delay_increment=SAR_ACK_DELAY_INCREMENT,
        rx_segment_interval_us=ms(SAR_RECEIVER_SEGMENT_INTERVAL_MS), ack_retx_count=SAR_ACK_RETRANSMISSIONS_COUNT,
        discard_timeout_us=ms(SAR_DISCARD_TIMEOUT_S * 1000), rx_threshold_dbm=rp.rx_threshold_dbm,
        interference_threshold_dbm=rp.interference_threshold_dbm, collision_model=COLLISION_MODEL,
        capture_db=CAPTURE_DB)


@dataclass
class Scenario:
    """The network one simulation run sees: nodes, links and per-node random streams.

    Node indices are 0..N-1. ``prx_dbm[u, v]`` is the power at v when u transmits.
    """
    names: list[str]
    is_dcu: list[bool]
    is_relay: list[bool]
    prx_dbm: np.ndarray
    meter_ttl: dict[int, int]            # meter node -> TTL of its messages
    meter_dcu: dict[int, int]            # meter node -> its DCU node
    scan_phase_us: list[int]
    gap_rng: list[random.Random | None]  # None = use the midpoint (deterministic tests)
    backoff_rng: list[random.Random | None]
    ack_delay_rng: list[random.Random | None] | None = None   # None = no streams (tests)


class NetPdu:
    """A network PDU. ``reading`` and ``sar_round`` are instrumentation, not protocol fields."""
    __slots__ = ("kind", "src", "dst", "seq", "ttl", "seq_zero", "seg_o", "seg_n", "block_ack", "reading", "sar_round")

    def __init__(self, kind: int, src: int, dst: int, seq: int, ttl: int, seq_zero: int,
                 seg_o: int = 0, seg_n: int = 0, block_ack: int = 0, reading: int = -1, sar_round: int = 0):
        self.kind, self.src, self.dst, self.seq, self.ttl = kind, src, dst, seq, ttl
        self.seq_zero, self.seg_o, self.seg_n, self.block_ack = seq_zero, seg_o, seg_n, block_ack
        self.reading, self.sar_round = reading, sar_round

    def relayed(self) -> "NetPdu":
        """Copy with TTL decremented, as sent by a relay."""
        return NetPdu(self.kind, self.src, self.dst, self.seq, self.ttl - 1, self.seq_zero,
                      self.seg_o, self.seg_n, self.block_ack, self.reading, self.sar_round)


class Air:
    """One PDU on one channel: transmitter, channel index (0..2 = 37..39) and [start, end)."""
    __slots__ = ("pdu", "tx", "ch", "start", "end", "relayed")

    def __init__(self, pdu: NetPdu, tx: int, ch: int, start: int, end: int, relayed: bool):
        self.pdu, self.tx, self.ch, self.start, self.end, self.relayed = pdu, tx, ch, start, end, relayed


class TxItem:
    """A queued network PDU with the number of advertising events still to send."""
    __slots__ = ("pdu", "events_left", "relayed", "counted", "sar", "seg")

    def __init__(self, pdu: NetPdu, events: int, relayed: bool, sar: "SarTx | None" = None, seg: int = -1):
        self.pdu, self.events_left, self.relayed, self.counted, self.sar, self.seg = pdu, events, relayed, False, sar, seg


class Reading:
    """One meter reading and everything measured about it."""
    __slots__ = ("id", "meter", "period", "measured", "t_gen", "t_first_tx", "t_complete", "delivered",
                 "seg_mask", "sar_rounds", "sender_outcome", "n_data_tx", "n_relay_seg_tx", "n_ack_tx",
                 "n_relay_ack_tx", "coll_crit", "coll_other", "coll_crit_new", "hd_loss")

    def __init__(self, rid: int, meter: int, period: int, measured: bool, t_gen: int):
        self.id, self.meter, self.period, self.measured, self.t_gen = rid, meter, period, measured, t_gen
        self.t_first_tx = self.t_complete = None
        self.delivered, self.seg_mask, self.sar_rounds, self.sender_outcome = False, 0, 0, None
        self.n_data_tx = self.n_relay_seg_tx = self.n_ack_tx = self.n_relay_ack_tx = 0
        self.coll_crit = self.coll_other = self.coll_crit_new = self.hd_loss = 0


class SarTx:
    """Sender side of one segmented message (lower transport layer of a meter).

    ``attempts_left`` and ``no_progress_left`` count transmission rounds including the first one
    (count + 1 each); a round decrements them when it ends.
    """
    __slots__ = ("reading", "node", "dst", "ttl", "seq_zero", "n_seg", "full", "acked", "round", "sending",
                 "pending", "ptr", "active", "attempts_left", "no_progress_left", "ack_received",
                 "last_adv_start", "timer_token", "step_token")

    def __init__(self, reading: Reading, node: int, dst: int, ttl: int, seq_zero: int, n_seg: int,
                 attempts: int, no_progress_attempts: int):
        self.reading, self.node, self.dst, self.ttl, self.seq_zero, self.n_seg = reading, node, dst, ttl, seq_zero, n_seg
        self.full = (1 << n_seg) - 1
        self.acked, self.round, self.sending, self.active = 0, 0, True, True
        self.pending, self.ptr = list(range(n_seg)), 0
        self.attempts_left, self.no_progress_left = attempts, no_progress_attempts
        self.ack_received = False
        self.last_adv_start: int | None = None   # start of the latest segment's first advertising event
        self.timer_token = self.step_token = 0

    def needs(self, seg: int) -> bool:
        """True while the message is in progress and ``seg`` is not acknowledged."""
        return self.active and not (self.acked >> seg) & 1


class SarRx:
    """Receiver side (DCU) of one segmented message, keyed by (SRC, SeqZero)."""
    __slots__ = ("key", "reading", "seg_n", "full", "mask", "active", "ack_token", "ack_retx_left",
                 "discard_deadline", "discard_scheduled")

    def __init__(self, key: tuple[int, int], reading: Reading, n_seg: int):
        self.key, self.reading, self.full, self.mask, self.active = key, reading, (1 << n_seg) - 1, 0, True
        self.seg_n = n_seg - 1
        self.ack_token, self.ack_retx_left, self.discard_deadline, self.discard_scheduled = 0, 0, 0, False


class NodeState:
    """Per-node protocol and radio state."""
    __slots__ = ("idx", "is_dcu", "is_relay", "txq", "radio_busy", "seq", "cache", "busy", "sar", "backlog",
                 "rx", "completed", "discarded")

    def __init__(self, idx: int, is_dcu: bool, is_relay: bool):
        self.idx, self.is_dcu, self.is_relay = idx, is_dcu, is_relay
        self.txq: deque[TxItem] = deque()
        self.radio_busy = False
        self.seq = 0
        self.cache: set[tuple[int, int]] = set()
        self.busy: deque[tuple[int, int]] = deque()     # recent own transmission intervals
        self.sar: SarTx | None = None
        self.backlog: deque[Reading] = deque()
        self.rx: dict[tuple[int, int], SarRx] = {}
        self.completed: dict[tuple[int, int], int] = {}   # (SRC, SeqZero) -> time of the latest ack sent
        self.discarded: set[tuple[int, int]] = set()


class MeshSimulator:
    """Discrete-event simulator of Bluetooth Mesh (advertising bearer, managed flooding, SAR).

    Parameters
    ----------
    sc : the network (nodes, received powers, TTLs, DCUs, random streams)
    cfg : protocol parameters
    drop_filter : test hook, ``f(air, receiver) -> True`` drops a reception that would succeed
    trace : record every transmission, reception and loss (for tests)
    """

    def __init__(self, sc: Scenario, cfg: ProtocolConfig,
                 drop_filter: Callable[[Air, int], bool] | None = None, trace: bool = False):
        self.sc, self.cfg, self.drop_filter, self.trace = sc, cfg, drop_filter, trace
        n = len(sc.names)
        self.n = n
        self.nodes = [NodeState(i, sc.is_dcu[i], sc.is_relay[i]) for i in range(n)]
        prx = np.array(sc.prx_dbm, dtype=float)
        self.prx: list[list[float]] = prx.tolist()
        with np.errstate(under="ignore"):
            self.prx_mw: list[list[float]] = np.where(np.isfinite(prx), 10.0 ** (prx / 10.0), 0.0).tolist()
        self.rx_cand = [[v for v in range(n) if v != u and prx[u, v] >= cfg.rx_threshold_dbm] for u in range(n)]
        self.q: list = []
        self._seq = 0
        self.now = 0
        self.n_events = 0
        self.active: list[deque[Air]] = [deque(), deque(), deque()]
        self.readings: dict[int, Reading] = {-1: Reading(-1, -1, -1, False, 0)}  # -1: sink for injected PDUs
        self.hd_losses = self.coll_crit = self.coll_other = 0
        self.tx_log: list[tuple] = []     # trace: (t, node, kind, src, seq, seg_o, relayed, sar_round, ttl)
        self.rx_log: list[tuple] = []     # trace: (t, rx, tx, kind, src, seq, seg_o, ttl, ch, new)
        self.loss_log: list[tuple] = []   # trace: (t, rx, tx, reason, kind, src, seq, ch)
        self.ack_log: list[tuple] = []    # trace: (t, meter, block_ack, sar_round_at_rx, state)

    # ---------------------------------------------------------------- event queue
    def _push(self, t: int, kind: int, a=None, b=None) -> None:
        """Schedule an event at integer time ``t`` (us)."""
        self._seq += 1
        heapq.heappush(self.q, (t, PRIORITY.get(kind, 3), self._seq, kind, a, b))

    def add_reports(self, schedule: list[tuple[int, int, int, bool]]) -> None:
        """Schedule readings: (time_us, meter node, period, measured)."""
        for t, meter, period, measured in schedule:
            rid = len(self.readings) - 1
            self.readings[rid] = Reading(rid, meter, period, measured, t)
            self._push(t, EV_REPORT, rid)

    def inject(self, node: int, pdu: NetPdu, t: int, events: int = 1) -> None:
        """Test hook: queue a raw network PDU at ``node`` at time ``t``."""
        self._push(t, EV_INJECT, node, (pdu, events))

    def run(self, until: int | None = None) -> None:
        """Process events until the queue is empty (or ``until``)."""
        q = self.q
        pop = heapq.heappop
        while q:
            if until is not None and q[0][0] > until:
                break
            t, _, _, kind, a, b = pop(q)
            if t < self.now:
                raise RuntimeError("time went backwards")
            self.now = t
            self.n_events += 1
            if kind == EV_TX_END:
                self._on_tx_end(a)
            elif kind == EV_TX_START:
                self._on_tx_start(a)
            elif kind == EV_ADV_END:
                self._on_adv_end(a, b)
            elif kind == EV_RELAY:
                self._submit(a, TxItem(b, self.cfg.relay_retransmit_count, True))
            elif kind == EV_SEG_SUBMIT:
                if a.active and a.sending and a.step_token == b:
                    self._seg_step(a)
            elif kind == EV_REPEAT:
                self._requeue(a, b)
            elif kind == EV_ACK_TIMER:
                self._on_ack_timer(a, b)
            elif kind == EV_SEG_TX_TIMER:
                sar = a
                if sar.active and not sar.sending and sar.timer_token == b:
                    self._retransmit(sar)
            elif kind == EV_DISCARD:
                self._on_discard(a, b)
            elif kind == EV_REPORT:
                self._on_report(self.readings[a])
            elif kind == EV_ACK_SEND:
                self._send_ack(a, *b)
            elif kind == EV_INJECT:
                pdu, events = b
                self.nodes[a].cache.add((pdu.src, pdu.seq))
                self._submit(a, TxItem(pdu, events, False))
            else:
                raise ValueError(kind)

    # ---------------------------------------------------------------- radio: transmit side
    def _submit(self, node: int, item: TxItem) -> None:
        """Queue a PDU for transmission at ``node``; start at once if the radio is idle."""
        st = self.nodes[node]
        st.txq.append(item)
        if not st.radio_busy:
            self._start_adv(node)

    def _requeue(self, node: int, item: TxItem) -> None:
        """Queue the next advertising event of a PDU (count > 1)."""
        self._submit(node, item)

    def _start_adv(self, node: int) -> None:
        """Start the next advertising event of the node's queue (radio idle)."""
        st = self.nodes[node]
        cfg = self.cfg
        while st.txq:
            item = st.txq.popleft()
            if item.sar is not None and not item.sar.needs(item.seg):
                self._seg_item_done(item)       # segment acknowledged or message finished: skip it
                if st.radio_busy:
                    return
                continue
            break
        else:
            return
        now = self.now
        pdu = item.pdu
        if not item.counted:
            item.counted = True
            r = self.readings[pdu.reading]
            if pdu.kind == SEG:
                if item.relayed:
                    r.n_relay_seg_tx += 1
                else:
                    r.n_data_tx += 1
                    if r.t_first_tx is None:
                        r.t_first_tx = now
                    if item.sar is not None:
                        item.sar.last_adv_start = now
            elif item.relayed:
                r.n_relay_ack_tx += 1
            else:
                r.n_ack_tx += 1
            if self.trace:
                self.tx_log.append((now, node, pdu.kind, pdu.src, pdu.seq, pdu.seg_o, item.relayed, pdu.sar_round, pdu.ttl))
        rng = self.sc.gap_rng[node]
        lo, hi = cfg.inter_pdu_us
        t = now
        for ch in range(3):
            self._push(t, EV_TX_START, Air(pdu, node, ch, t, t + cfg.airtime_us, item.relayed))
            if ch < 2:
                t += (lo + hi) // 2 if rng is None else lo + int(rng.random() * (hi - lo + 1))
        end = t + cfg.airtime_us
        st.radio_busy = True
        if cfg.half_duplex_scope == "adv_event":
            self._mark_busy(st, now, end)
        self._push(end, EV_ADV_END, node, (item, now))

    def _mark_busy(self, st: NodeState, s: int, e: int) -> None:
        """Record that the node transmits during [s, e); prune intervals no reception can overlap."""
        b = st.busy
        cutoff = s - self.cfg.airtime_us
        while b and b[0][1] <= cutoff:
            b.popleft()
        b.append((s, e))

    def _on_tx_start(self, air: Air) -> None:
        lst = self.active[air.ch]
        cutoff = air.start - self.cfg.airtime_us
        while lst and lst[0].end <= cutoff:  # all PDUs have the same airtime: list is ordered by end
            lst.popleft()
        lst.append(air)
        if self.cfg.half_duplex_scope == "pdu":
            self._mark_busy(self.nodes[air.tx], air.start, air.end)
        self._push(air.end, EV_TX_END, air)

    def _on_adv_end(self, node: int, arg: tuple[TxItem, int]) -> None:
        item, adv_start = arg
        st = self.nodes[node]
        st.radio_busy = False
        item.events_left -= 1
        if item.events_left > 0:
            self._push(adv_start + self.cfg.repetition_us, EV_REPEAT, node, item)
        elif item.sar is not None:
            self._seg_item_done(item)
        if not st.radio_busy:
            self._start_adv(node)

    # ---------------------------------------------------------------- radio: receive side
    def _on_tx_end(self, air: Air) -> None:
        """Evaluate the reception of one PDU at every node in range."""
        cfg = self.cfg
        s, e, ch, u = air.start, air.end, air.ch, air.tx
        overlap = [y for y in self.active[ch] if y is not air and y.start < e and y.end > s]
        si = cfg.scan_interval_us
        phases = self.sc.scan_phase_us
        nodes = self.nodes
        prx = self.prx
        for v in self.rx_cand[u]:
            ph = phases[v]
            w = (s + ph) // si
            if w % 3 != ch or (e - 1 + ph) // si != w:
                continue                                    # scanner not on this channel for the whole PDU
            stv = nodes[v]
            busy = False
            for bs, be in stv.busy:
                if bs < e and be > s:
                    busy = True
                    break
            if busy:
                self.hd_losses += 1
                self.readings[air.pdu.reading].hd_loss += 1
                if self.trace:
                    self.loss_log.append((e, v, u, "half_duplex", air.pdu.kind, air.pdu.src, air.pdu.seq, ch))
                continue
            if overlap:
                if cfg.collision_model == "any_overlap":
                    thr = cfg.interference_threshold_dbm
                    lost = any(prx[y.tx][v] >= thr for y in overlap if y.tx != v)
                else:
                    interf = sum(self.prx_mw[y.tx][v] for y in overlap if y.tx != v)
                    lost = interf > 0 and prx[u][v] - 10.0 * math.log10(interf) < cfg.capture_db
                if lost:
                    r = self.readings[air.pdu.reading]
                    if stv.is_dcu or stv.is_relay:
                        self.coll_crit += 1
                        r.coll_crit += 1
                        if (air.pdu.src, air.pdu.seq) not in stv.cache:
                            r.coll_crit_new += 1
                    else:
                        self.coll_other += 1
                        r.coll_other += 1
                    if self.trace:
                        self.loss_log.append((e, v, u, "collision", air.pdu.kind, air.pdu.src, air.pdu.seq, ch))
                    continue
            if self.drop_filter is not None and self.drop_filter(air, v):
                if self.trace:
                    self.loss_log.append((e, v, u, "dropped", air.pdu.kind, air.pdu.src, air.pdu.seq, ch))
                continue
            self._receive(v, air)

    def _receive(self, v: int, air: Air) -> None:
        """Network layer: cache check, delivery to the lower transport layer, relaying."""
        pdu = air.pdu
        st = self.nodes[v]
        key = (pdu.src, pdu.seq)
        new = key not in st.cache
        if self.trace:
            self.rx_log.append((self.now, v, air.tx, pdu.kind, pdu.src, pdu.seq, pdu.seg_o, pdu.ttl, air.ch, new))
        if not new:
            return
        st.cache.add(key)
        if pdu.dst == v:
            if pdu.kind == SEG and st.is_dcu:
                self._dcu_rx_segment(v, pdu)
            elif pdu.kind == ACK:
                self._meter_rx_ack(v, pdu)
            return
        if st.is_relay and not st.is_dcu and pdu.ttl >= 2:
            rng = self.sc.backoff_rng[v]
            lo, hi = self.cfg.relay_backoff_us
            delay = (lo + hi) // 2 if rng is None else lo + int(rng.random() * (hi - lo + 1))
            self._push(self.now + delay, EV_RELAY, v, pdu.relayed())

    # ---------------------------------------------------------------- SAR sender (meter)
    def _on_report(self, reading: Reading) -> None:
        st = self.nodes[reading.meter]
        if st.sar is None:
            self._start_reading(reading)
        else:
            st.backlog.append(reading)

    def _start_reading(self, reading: Reading) -> None:
        node = reading.meter
        st = self.nodes[node]
        cfg = self.cfg
        sar = SarTx(reading, node, self.sc.meter_dcu[node], self.sc.meter_ttl[node], st.seq, cfg.n_segments,
                    cfg.retx_count + 1, cfg.retx_without_progress_count + 1)
        st.sar = sar
        self._seg_step(sar)

    def _retx_interval_us(self, ttl: int) -> int:
        """SAR unicast retransmissions timer: step + increment x (TTL - 1), or step if TTL = 0."""
        cfg = self.cfg
        return cfg.retx_interval_step_us + (cfg.retx_interval_increment_us * (ttl - 1) if ttl > 0 else 0)

    def _seg_step(self, sar: SarTx) -> None:
        """Send the next unacknowledged segment of the round, or end the round and start the timer."""
        while sar.ptr < len(sar.pending) and (sar.acked >> sar.pending[sar.ptr]) & 1:
            sar.ptr += 1
        if sar.ptr < len(sar.pending):
            seg = sar.pending[sar.ptr]
            sar.ptr += 1
            st = self.nodes[sar.node]
            pdu = NetPdu(SEG, sar.node, sar.dst, st.seq, sar.ttl, sar.seq_zero, seg, sar.n_seg - 1,
                         reading=sar.reading.id, sar_round=sar.round)
            st.seq += 1
            st.cache.add((pdu.src, pdu.seq))
            sar.ack_received = False
            self._submit(sar.node, TxItem(pdu, self.cfg.network_transmit_count, False, sar, seg))
            return
        # Round finished: one attempt used; it made no progress unless an ack came after the last segment.
        sar.attempts_left -= 1
        if not sar.ack_received:
            sar.no_progress_left -= 1
        timeout = self.cfg.segment_interval_us if sar.ack_received else self._retx_interval_us(sar.ttl)
        sar.ack_received = False
        sar.sending = False
        sar.timer_token += 1
        start = sar.last_adv_start if sar.last_adv_start is not None else self.now
        self._push(max(self.now, start + timeout), EV_SEG_TX_TIMER, sar, sar.timer_token)

    def _seg_item_done(self, item: TxItem) -> None:
        """A segment finished its last advertising event (or was skipped): schedule the next step.

        The next segment starts one segment interval after the start of the previous one, and not
        before the previous one has been transmitted.
        """
        sar = item.sar
        if not sar.active or not sar.sending:
            return
        start = sar.last_adv_start if sar.last_adv_start is not None else self.now
        sar.step_token += 1
        self._push(max(self.now, start + self.cfg.segment_interval_us), EV_SEG_SUBMIT, sar, sar.step_token)

    def _retransmit(self, sar: SarTx) -> None:
        """Retransmit all unacknowledged segments, or cancel when an attempt counter is exhausted."""
        if sar.attempts_left <= 0 or sar.no_progress_left <= 0:
            self._finish_sar(sar, success=False)
            return
        sar.round += 1
        sar.reading.sar_rounds += 1
        sar.pending = [s for s in range(sar.n_seg) if not (sar.acked >> s) & 1]
        sar.ptr, sar.sending = 0, True
        sar.timer_token += 1
        self._seg_step(sar)

    def _meter_rx_ack(self, node: int, pdu: NetPdu) -> None:
        st = self.nodes[node]
        sar = st.sar
        if self.trace:
            self.ack_log.append((self.now, node, pdu.block_ack, sar.round if sar else None,
                                 None if sar is None else ("sending" if sar.sending else "waiting")))
        if sar is None or not sar.active or pdu.src != sar.dst or pdu.seq_zero != sar.seq_zero:
            return
        new = pdu.block_ack & sar.full & ~sar.acked
        if new:
            sar.acked |= new
            sar.no_progress_left = self.cfg.retx_without_progress_count + 1     # progress
        if sar.acked == sar.full:
            self._finish_sar(sar, success=True)
        elif not sar.sending:
            # Timer running: retransmit now (one segment interval after the last segment at the earliest).
            sar.timer_token += 1
            if sar.attempts_left <= 0 or sar.no_progress_left <= 0:
                self._finish_sar(sar, success=False)
                return
            start = sar.last_adv_start if sar.last_adv_start is not None else self.now
            self._push(max(self.now, start + self.cfg.segment_interval_us), EV_SEG_TX_TIMER, sar, sar.timer_token)
        else:
            sar.ack_received = True

    def _finish_sar(self, sar: SarTx, success: bool) -> None:
        sar.active = False
        sar.timer_token += 1
        sar.reading.sender_outcome = "acked" if success else "failed"
        st = self.nodes[sar.node]
        st.sar = None
        if st.backlog:
            self._start_reading(st.backlog.popleft())

    # ---------------------------------------------------------------- SAR receiver (DCU)
    def _ack_delay_us(self, seg_n: int) -> int:
        """SAR acknowledgment timer: min(SegN + 0.5, delay increment) x receiver segment interval."""
        cfg = self.cfg
        return int(round(min(seg_n + 0.5, cfg.ack_delay_increment) * cfg.rx_segment_interval_us))

    def _dcu_rx_segment(self, d: int, pdu: NetPdu) -> None:
        st = self.nodes[d]
        cfg = self.cfg
        key = (pdu.src, pdu.seq_zero)
        reading = self.readings[pdu.reading]
        full = (1 << (pdu.seg_n + 1)) - 1
        if key in st.completed:
            # Already reassembled: acknowledge again, at most once per delay-increment x segment interval.
            if self.now - st.completed[key] > int(round(cfg.ack_delay_increment * cfg.rx_segment_interval_us)):
                st.completed[key] = self.now
                self._ack_on_segment(d, pdu.src, pdu.seq_zero, full, reading)
            return
        if key in st.discarded:
            return                                  # discarded by the discard timer: ignored
        sess = st.rx.get(key)
        if sess is None:
            sess = st.rx[key] = SarRx(key, reading, pdu.seg_n + 1)
        bit = 1 << pdu.seg_o
        if sess.mask & bit:
            return                                  # segment already received: no timer is restarted
        sess.mask |= bit
        reading.seg_mask |= bit
        sess.discard_deadline = self.now + cfg.discard_timeout_us
        if not sess.discard_scheduled:
            sess.discard_scheduled = True
            self._push(sess.discard_deadline, EV_DISCARD, d, sess)
        sess.ack_retx_left = cfg.ack_retx_count
        sess.ack_token += 1                         # (re)start the acknowledgment timer, or cancel it
        if sess.mask == sess.full:
            sess.active = False
            del st.rx[key]
            st.completed[key] = self.now
            reading.delivered = True
            reading.t_complete = self.now
            self._ack_on_segment(d, pdu.src, pdu.seq_zero, full, reading)
        else:
            self._push(self.now + self._ack_delay_us(sess.seg_n), EV_ACK_TIMER, (d, sess), sess.ack_token)

    def _ack_on_segment(self, d: int, meter: int, seq_zero: int, mask: int, reading: Reading) -> None:
        """Acknowledgment triggered by a segment: sent at once, or after ACK_DELAY_MS if configured."""
        lo, hi = self.cfg.ack_delay_us
        if hi <= 0:
            self._send_ack(d, meter, seq_zero, mask, reading)
            return
        rngs = self.sc.ack_delay_rng
        rng = None if rngs is None else rngs[d]
        delay = (lo + hi) // 2 if rng is None else lo + int(rng.random() * (hi - lo + 1))
        self._push(self.now + delay, EV_ACK_SEND, d, (meter, seq_zero, mask, reading))

    def _on_ack_timer(self, arg: tuple[int, SarRx], token: int) -> None:
        """Acknowledgment timer expired: acknowledge the segments received so far.

        For messages with SegN above the segments threshold the acknowledgment is repeated
        ``SAR_ACK_RETRANSMISSIONS_COUNT`` times, one receiver segment interval apart.
        """
        d, sess = arg
        if not sess.active or token != sess.ack_token:
            return
        self._send_ack(d, sess.key[0], sess.key[1], sess.mask, sess.reading)
        if sess.ack_retx_left > 0 and sess.seg_n > self.cfg.segments_threshold:
            sess.ack_retx_left -= 1
            self._push(self.now + self.cfg.rx_segment_interval_us, EV_ACK_TIMER, (d, sess), token)

    def _on_discard(self, d: int, sess: SarRx) -> None:
        if not sess.active:
            return
        if self.now < sess.discard_deadline:    # restarted by a later segment: re-arm
            self._push(sess.discard_deadline, EV_DISCARD, d, sess)
            return
        sess.active = False
        st = self.nodes[d]
        del st.rx[sess.key]
        st.discarded.add(sess.key)

    def _send_ack(self, d: int, meter: int, seq_zero: int, mask: int, reading: Reading) -> None:
        """Queue a Segment Acknowledgment (one unsegmented control PDU) to ``meter``."""
        st = self.nodes[d]
        pdu = NetPdu(ACK, d, meter, st.seq, self.sc.meter_ttl[meter], seq_zero, block_ack=mask, reading=reading.id)
        st.seq += 1
        st.cache.add((pdu.src, pdu.seq))
        self._submit(d, TxItem(pdu, self.cfg.network_transmit_count, False))


# ### Building a scenario from a plan, and summarising a run
#
# The **universe** of a site is all meters (file order) followed by all poles (file order). A method
# uses the covered meters and its DCU poles; universe indices key every random stream, so a node
# has the same scan phase, gap stream and back-off stream in every method.

@dataclass
class SiteUniverse:
    """Site-level data shared by all methods of one site."""
    plans: SitePlans
    rp: RadioParams
    cfg: ProtocolConfig
    xy: np.ndarray            # universe coordinates
    names: list[str]          # "M:<id>" / "P:<id>"
    meter_u: dict[str, int]   # meter id -> universe index
    pole_u: dict[str, int]


def site_universe(sp: SitePlans) -> SiteUniverse:
    """Universe and parameters of one site."""
    rp = radio_params(sp.params)
    xy = np.vstack([sp.meter_xy, sp.pole_xy])
    names = [f"M:{m}" for m in sp.meter_ids] + [f"P:{p}" for p in sp.pole_ids]
    return SiteUniverse(sp, rp, protocol_config(rp), xy, names,
                        {m: i for i, m in enumerate(sp.meter_ids)},
                        {p: len(sp.meter_ids) + i for i, p in enumerate(sp.pole_ids)})


def build_scenario(su: SiteUniverse, method: str, prx_u: np.ndarray, scan_phase_u: np.ndarray,
                   interval_s: int, run: int) -> tuple[Scenario, list[int]]:
    """Scenario for one method; returns it and the universe index of every node."""
    sp = su.plans
    ups = sp.methods[method]
    meters = [m for m in sp.meter_ids if any(m in up.meter_ttl for up in ups)]
    dcu_poles = sorted({up.dcu_pole for up in ups}, key=lambda p: su.pole_u[p])
    uidx = [su.meter_u[m] for m in meters] + [su.pole_u[p] for p in dcu_poles]
    node_of_meter = {m: i for i, m in enumerate(meters)}
    node_of_pole = {p: len(meters) + i for i, p in enumerate(dcu_poles)}
    relays = {r for up in ups for r in up.relay_ids}
    ttl, dcu = {}, {}
    for up in ups:
        for m, t in up.meter_ttl.items():
            ttl[node_of_meter[m]] = t
            dcu[node_of_meter[m]] = node_of_pole[up.dcu_pole]
    h = site_hash(sp.name)
    sc = Scenario(
        names=[su.names[u] for u in uidx],
        is_dcu=[False] * len(meters) + [True] * len(dcu_poles),
        is_relay=[m in relays for m in meters] + [False] * len(dcu_poles),
        prx_dbm=prx_u[np.ix_(uidx, uidx)],
        meter_ttl=ttl, meter_dcu=dcu,
        scan_phase_us=[int(scan_phase_u[u]) for u in uidx],
        gap_rng=[py_rng(seed_sequence(h, PURPOSE_NODE, interval_s, run, u, STREAM_GAP)) for u in uidx],
        backoff_rng=[py_rng(seed_sequence(h, PURPOSE_NODE, interval_s, run, u, STREAM_BACKOFF)) for u in uidx],
        ack_delay_rng=[py_rng(seed_sequence(h, PURPOSE_NODE, interval_s, run, u, STREAM_ACK_DELAY)) for u in uidx],
    )
    return sc, uidx


def summarize_run(sim: MeshSimulator, n_seg: int) -> tuple[dict, dict[int, tuple[int, int]]]:
    """Metrics of one run over the measured readings; also (readings, delivered) per meter node."""
    rs = [r for r in sim.readings.values() if r.measured]
    n = len(rs)
    delivered = [r for r in rs if r.delivered]
    lat = np.array([(r.t_complete - r.t_first_tx) / 1000.0 for r in delivered])
    lat_gen = np.array([(r.t_complete - r.t_gen) / 1000.0 for r in delivered])
    unresolved = sum(1 for r in rs if r.sender_outcome is None)
    tot = {k: sum(getattr(r, k) for r in rs) for k in
           ("n_data_tx", "n_relay_seg_tx", "n_ack_tx", "n_relay_ack_tx", "coll_crit", "coll_other", "coll_crit_new",
            "hd_loss", "sar_rounds")}
    total_tx = tot["n_data_tx"] + tot["n_relay_seg_tx"] + tot["n_ack_tx"] + tot["n_relay_ack_tx"]
    nan = float("nan")
    row = {
        "n_readings": n, "n_delivered": len(delivered),
        "pdr": len(delivered) / n if n else nan,
        "seg_delivery_ratio": sum(bin(r.seg_mask).count("1") for r in rs) / (n * n_seg) if n else nan,
        "latency_mean_ms": float(lat.mean()) if len(lat) else nan,
        "latency_median_ms": float(np.median(lat)) if len(lat) else nan,
        "latency_p95_ms": float(np.percentile(lat, 95)) if len(lat) else nan,
        "latency_from_generation_mean_ms": float(lat_gen.mean()) if len(lat_gen) else nan,
        "tx_data_seg_per_reading": tot["n_data_tx"] / n, "tx_relay_seg_per_reading": tot["n_relay_seg_tx"] / n,
        "tx_ack_per_reading": tot["n_ack_tx"] / n, "tx_relay_ack_per_reading": tot["n_relay_ack_tx"] / n,
        "tx_total_per_reading": total_tx / n,
        "collisions_relay_dcu": tot["coll_crit"], "collisions_nonrelay": tot["coll_other"],
        "collisions_relay_dcu_per_reading": tot["coll_crit"] / n, "collisions_nonrelay_per_reading": tot["coll_other"] / n,
        "collisions_relay_dcu_new_per_reading": tot["coll_crit_new"] / n,
        "halfduplex_losses": tot["hd_loss"],
        "sar_rounds_per_reading": tot["sar_rounds"] / n,
        "frac_readings_retx": sum(1 for r in rs if r.sar_rounds > 0) / n,
        "sender_ack_ratio": sum(1 for r in rs if r.sender_outcome == "acked") / n,
        "unresolved_readings": unresolved,
    }
    per_meter: dict[int, list[int]] = {}
    for r in rs:
        c = per_meter.setdefault(r.meter, [0, 0])
        c[0] += 1
        c[1] += r.delivered
    return row, {k: (v[0], v[1]) for k, v in per_meter.items()}


# ## Part 5 — Validation tests
#
# Deterministic tests with shadowing off and fixed timing, on the path-loss parameters of the site: the inter-PDU spacing and relay back-off
# are the midpoints of their ranges (1.5 ms, 10 ms) and every scan phase is 0, so all nodes listen
# on the same channel at the same time. Nodes lie on a line 100 m apart: with the plan's path-loss
# parameters a 100 m link is above the RX threshold and a 200 m link is below the interference
# threshold, so only adjacent nodes hear each other.
#
# 1. **Line S–R–D**: a reading arrives complete (18 segments) and S receives the acknowledgment.
# 2. **TTL**: TTL 1 at S → nothing is relayed, the DCU receives nothing; TTL 2 → R forwards, D receives.
# 3. **Cache**: a relay that receives the same PDU twice forwards it once.
# 4. **Collision**: two transmissions overlapping on one channel at a receiver are both lost under
#    `any_overlap` (control: 1 ms apart, both received; `sinr` with a 20 dB stronger signal
#    captures it).
# 5. **SAR retransmission**: segment 5 is dropped on purpose in the first round; the acknowledgment
#    reports exactly segment 5 missing and only segment 5 is retransmitted.
# 6. **Half-duplex**: a node that is transmitting does not receive.
# 7. **Retransmissions without progress** (Mesh 1.1): with no acknowledgment the sender stops when
#    the without-progress counter runs out, even if the total counter allows more; when every
#    acknowledgment makes progress the counter is reset and the message completes.
#
# The main simulation does not start unless all tests pass.

TEST_SITE_PARAMS: dict = {}   # path-loss parameters of the site under study (set by run_validation_tests)


def test_config(**overrides) -> ProtocolConfig:
    """ProtocolConfig for the tests (from the configuration section, with overrides)."""
    rp = RadioParams(TEST_SITE_PARAMS["eirp_dbm"], TEST_SITE_PARAMS["sensitivity_dbm"],
                     TEST_SITE_PARAMS["path_loss_exp"], TEST_SITE_PARAMS["pl_d0_db"], TEST_SITE_PARAMS["d0_m"], 0.0,
                     TEST_SITE_PARAMS["sensitivity_dbm"] + RX_MARGIN_DB, TEST_SITE_PARAMS["sensitivity_dbm"])
    cfg = dataclasses.replace(protocol_config(rp), collision_model="any_overlap", half_duplex_scope="pdu",
                              ack_delay_us=(0, 0))
    return dataclasses.replace(cfg, **overrides)


def line_scenario(kinds: str, relays: set[int], ttl: dict[int, int], dcu: int, spacing_m: float = 100.0,
                  prx: np.ndarray | None = None) -> Scenario:
    """Nodes on a line (``kinds``: 'M' meter / 'D' DCU per node), deterministic streams."""
    n = len(kinds)
    if prx is None:
        rp = RadioParams(TEST_SITE_PARAMS["eirp_dbm"], TEST_SITE_PARAMS["sensitivity_dbm"],
                         TEST_SITE_PARAMS["path_loss_exp"], TEST_SITE_PARAMS["pl_d0_db"], TEST_SITE_PARAMS["d0_m"],
                         0.0, 0.0, 0.0)
        xy = np.array([[i * spacing_m, 0.0] for i in range(n)])
        prx = rx_power_matrix(xy, rp, None)
    return Scenario(names=[f"{k}{i}" for i, k in enumerate(kinds)], is_dcu=[k == "D" for k in kinds],
                    is_relay=[i in relays for i in range(n)], prx_dbm=prx, meter_ttl=ttl,
                    meter_dcu={m: dcu for m in ttl}, scan_phase_us=[0] * n, gap_rng=[None] * n, backoff_rng=[None] * n)


def one_reading(sim: MeshSimulator, meter: int, t: int = 0) -> Reading:
    """Schedule one measured reading and run to completion."""
    sim.add_reports([(t, meter, 0, True)])
    sim.run()
    return sim.readings[0]


def test_line_delivery() -> str:
    cfg = test_config()
    p = line_scenario("MMD", relays={1}, ttl={0: 2, 1: 1}, dcu=2).prx_dbm
    assert p[0, 1] >= cfg.rx_threshold_dbm and p[1, 2] >= cfg.rx_threshold_dbm, "100 m links must be in range"
    assert p[0, 2] < cfg.interference_threshold_dbm, "200 m link must be out of interference range"
    sim = MeshSimulator(line_scenario("MMD", relays={1}, ttl={0: 2, 1: 1}, dcu=2), cfg, trace=True)
    r = one_reading(sim, 0)
    segs_at_dcu = {x[6] for x in sim.rx_log if x[1] == 2 and x[3] == SEG and x[4] == 0}
    assert r.delivered, "reading not delivered"
    assert segs_at_dcu == set(range(N_SEGMENTS)), f"DCU got segments {sorted(segs_at_dcu)}"
    assert bin(r.seg_mask).count("1") == N_SEGMENTS
    assert r.sender_outcome == "acked", f"source outcome {r.sender_outcome}"
    assert any(x[1] == 0 and x[2] == (1 << N_SEGMENTS) - 1 for x in sim.ack_log), "no full ack at the source"
    round0 = [x for x in sim.tx_log if x[2] == SEG and x[7] == 0]
    assert sum(not x[6] for x in round0) == N_SEGMENTS and sum(x[6] for x in round0) == N_SEGMENTS, \
        "first round: expected each segment sent once by S and relayed once by R"
    assert r.t_complete < min((x[0] for x in sim.tx_log if x[2] == SEG and x[7] >= 1), default=10**18), \
        "the DCU should complete the reading in the first round"
    extra = r.n_data_tx - N_SEGMENTS
    note = (f"; S missed {sum(1 for x in sim.tx_log if x[2] == ACK) - len(sim.ack_log)} ack transmission(s) while its "
            f"scanner was on another channel, so it resent {extra} segment(s) and got the full ack again") if extra else ""
    return (f"{N_SEGMENTS} segments delivered via the relay in round 0, latency "
            f"{(r.t_complete - r.t_first_tx) / 1000:.1f} ms, full ack received{note}")


def test_ttl() -> str:
    cfg = test_config()
    sim1 = MeshSimulator(line_scenario("MMD", relays={1}, ttl={0: 1, 1: 1}, dcu=2), cfg, trace=True)
    r1 = one_reading(sim1, 0)
    assert any(x[1] == 1 and x[3] == SEG for x in sim1.rx_log), "relay should hear the TTL-1 segments"
    assert r1.n_relay_seg_tx == 0, "TTL 1 must not be relayed"
    assert not any(x[1] == 2 for x in sim1.rx_log), "DCU must receive nothing with TTL 1"
    assert not r1.delivered and r1.sender_outcome == "failed"
    sim2 = MeshSimulator(line_scenario("MMD", relays={1}, ttl={0: 2, 1: 1}, dcu=2), cfg, trace=True)
    r2 = one_reading(sim2, 0)
    assert r2.n_relay_seg_tx >= N_SEGMENTS and r2.delivered
    assert all(x[7] == 1 for x in sim2.rx_log if x[1] == 2 and x[3] == SEG), "relayed segments arrive with TTL 1"
    return f"TTL 1: 0 relayed, DCU silent, sender gave up after {r1.sar_rounds} rounds; TTL 2: relayed and delivered"


def test_cache() -> str:
    cfg = test_config()
    # S(0) and relays R1(1), R2(2) all within 100 m of each other; DCU(3) far away.
    n = 4
    p = np.full((n, n), -200.0)
    for a, b in ((0, 1), (0, 2), (1, 2)):
        p[a, b] = p[b, a] = -60.0
    np.fill_diagonal(p, -np.inf)
    sc = line_scenario("MMMD", relays={1, 2}, ttl={0: 3, 1: 1, 2: 1}, dcu=3, prx=p)
    sc.backoff_rng = [None, None, random.Random(7), None]     # R2 relays at another time than R1
    sim = MeshSimulator(sc, cfg, trace=True)
    sim.inject(0, NetPdu(SEG, 0, 3, seq=0, ttl=3, seq_zero=0), t=0)
    sim.run()
    r1_rx = [x for x in sim.rx_log if x[1] == 1 and x[4] == 0 and x[5] == 0]
    r1_tx = [x for x in sim.tx_log if x[1] == 1 and x[3] == 0 and x[4] == 0]
    assert len(r1_rx) >= 2, f"R1 should receive the PDU at least twice, got {len(r1_rx)}"
    assert sum(x[9] for x in r1_rx) == 1, "only the first copy is new"
    assert len(r1_tx) == 1, f"R1 forwarded {len(r1_tx)} times"
    return f"R1 received the PDU {len(r1_rx)} times (from S and R2), forwarded once"


def test_collision() -> str:
    # A(0) and B(2) both 100 m from C(1); A and B 200 m apart. C is a DCU (a pure receiver).
    out = []
    for gap_us, model, expect in ((0, "any_overlap", (False, False)), (1000, "any_overlap", (True, True))):
        sim = MeshSimulator(line_scenario("MDM", relays=set(), ttl={0: 1, 2: 1}, dcu=1), test_config(collision_model=model), trace=True)
        sim.inject(0, NetPdu(SEG, 0, 1, seq=0, ttl=1, seq_zero=0), t=0)
        sim.inject(2, NetPdu(SEG, 2, 1, seq=0, ttl=1, seq_zero=0), t=gap_us)
        sim.run()
        got = (any(x[1] == 1 and x[4] == 0 for x in sim.rx_log), any(x[1] == 1 and x[4] == 2 for x in sim.rx_log))
        assert got == expect, f"offset {gap_us} us, {model}: received {got}, expected {expect}"
        if gap_us == 0:
            coll = [x for x in sim.loss_log if x[1] == 1 and x[3] == "collision"]
            assert {x[5] for x in coll} == {0, 2} and sim.coll_crit == len(coll) >= 2
            out.append(f"simultaneous: both lost ({len(coll)} collisions at C)")
        else:
            out.append("1 ms apart: both received")
    # sinr: A 20 dB stronger than B at C -> A captured, B lost.
    p = np.array([[-np.inf, -70.0, -200.0], [-70.0, -np.inf, -90.0], [-200.0, -90.0, -np.inf]])
    sim = MeshSimulator(line_scenario("MDM", relays=set(), ttl={0: 1, 2: 1}, dcu=1, prx=p), test_config(collision_model="sinr"), trace=True)
    sim.inject(0, NetPdu(SEG, 0, 1, seq=0, ttl=1, seq_zero=0), t=0)
    sim.inject(2, NetPdu(SEG, 2, 1, seq=0, ttl=1, seq_zero=0), t=0)
    sim.run()
    got = (any(x[1] == 1 and x[4] == 0 for x in sim.rx_log), any(x[1] == 1 and x[4] == 2 for x in sim.rx_log))
    assert got == (True, False), f"sinr capture: {got}"
    out.append("sinr: stronger signal captured")
    return "; ".join(out)


def test_sar_retransmission() -> str:
    cfg = test_config()
    dropped = []

    def drop(air: Air, v: int) -> bool:
        hit = v == 1 and air.pdu.kind == SEG and air.pdu.seg_o == 5 and air.pdu.sar_round == 0
        if hit:
            dropped.append(air.ch)
        return hit

    sim = MeshSimulator(line_scenario("MD", relays=set(), ttl={0: 1}, dcu=1), cfg, drop_filter=drop, trace=True)
    r = one_reading(sim, 0)
    assert dropped, "segment 5 was never offered to the DCU"
    full = (1 << N_SEGMENTS) - 1
    after_round0 = [a for a in sim.ack_log if a[4] == "waiting" and a[3] == 0]
    assert after_round0, "no acknowledgment reached the sender after round 0"
    missing = [s for s in range(N_SEGMENTS) if not (after_round0[0][2] >> s) & 1]
    assert missing == [5], f"acknowledgment reports missing {missing}"
    retx_segs = [x for x in sim.tx_log if x[1] == 0 and x[2] == SEG and x[7] >= 1]
    assert [x[5] for x in retx_segs] == [5], f"retransmitted segments {[x[5] for x in retx_segs]}"
    assert r.delivered and r.sar_rounds == 1 and r.sender_outcome == "acked" and r.n_data_tx == N_SEGMENTS + 1
    assert any(a[2] == full for a in sim.ack_log)
    return "ack bitmap missing only segment 5; only segment 5 retransmitted; reading delivered"


def test_half_duplex() -> str:
    # A(0) -> C(1) while C is transmitting its own PDU on the same channel at the same time.
    sim = MeshSimulator(line_scenario("MMM", relays=set(), ttl={0: 1, 1: 1, 2: 1}, dcu=1), test_config(), trace=True)
    sim.inject(0, NetPdu(SEG, 0, 2, seq=0, ttl=1, seq_zero=0), t=0)
    sim.inject(1, NetPdu(SEG, 1, 2, seq=0, ttl=1, seq_zero=0), t=100)
    sim.run()
    assert not any(x[1] == 1 and x[4] == 0 for x in sim.rx_log), "C received while transmitting"
    hd = [x for x in sim.loss_log if x[1] == 1 and x[3] == "half_duplex"]
    assert hd and sim.hd_losses >= 1
    # Control: C idle -> it receives A's PDU.
    sim = MeshSimulator(line_scenario("MMM", relays=set(), ttl={0: 1, 1: 1, 2: 1}, dcu=1), test_config(), trace=True)
    sim.inject(0, NetPdu(SEG, 0, 2, seq=0, ttl=1, seq_zero=0), t=0)
    sim.run()
    assert any(x[1] == 1 and x[4] == 0 for x in sim.rx_log)
    return f"transmitting node lost {len(hd)} PDU(s) to half-duplex; idle node receives"


def test_without_progress() -> str:
    # Out of range: no acknowledgment ever arrives, so every round is without progress. With 5
    # retransmissions allowed but only 2 without progress, the sender must give up after 2 rounds.
    cfg = test_config(retx_count=5, retx_without_progress_count=2, retx_interval_step_us=ms(400))
    sim = MeshSimulator(line_scenario("MMD", relays=set(), ttl={0: 1, 1: 1}, dcu=2), cfg, trace=True)
    r = one_reading(sim, 0)
    assert r.sender_outcome == "failed" and r.sar_rounds == 2, (r.sender_outcome, r.sar_rounds)
    # In range, segments lost on purpose (5 and 6 in round 0, 5 in round 1): the acknowledgment after
    # each round makes progress, which resets the counter, so with only 1 retransmission without
    # progress allowed a second round still happens and the DCU completes the message. (Whether the
    # sender then hears the final acknowledgment depends on its scanner; it is not part of this test.)
    lost = {0: {5, 6}, 1: {5}}

    def drop(air: Air, v: int) -> bool:
        return v == 1 and air.pdu.kind == SEG and air.pdu.seg_o in lost.get(air.pdu.sar_round, set())

    cfg = test_config(retx_count=5, retx_without_progress_count=1, retx_interval_step_us=ms(400))
    sim = MeshSimulator(line_scenario("MD", relays=set(), ttl={0: 1}, dcu=1), cfg, drop_filter=drop, trace=True)
    r = one_reading(sim, 0)
    # Round 2 (which delivers segment 5) is only allowed because the ack after round 1 made progress:
    # without the reset, the limit of 1 retransmission without progress would cancel after round 1.
    round2 = [x[0] for x in sim.tx_log if x[1] == 0 and x[2] == SEG and x[7] == 2]
    round3 = [x[0] for x in sim.tx_log if x[1] == 0 and x[2] == SEG and x[7] == 3]
    assert r.delivered and round2 and r.t_complete >= min(round2), (r.delivered, r.sar_rounds)
    assert not round3 or r.t_complete < min(round3), "the DCU should complete in round 2"
    extra = (f"; the sender missed the final ack and retransmitted {r.sar_rounds - 2} more round(s)"
             if r.sar_rounds > 2 else "")
    return ("no acknowledgment: cancelled after 2 rounds without progress (5 allowed in total); "
            "progress in every round: round 2 allowed with a without-progress limit of 1, DCU completed "
            f"in round 2{extra}")


TESTS = [("1 line delivery", test_line_delivery), ("2 TTL", test_ttl), ("3 cache", test_cache),
         ("4 collision", test_collision), ("5 SAR retransmission", test_sar_retransmission),
         ("6 half-duplex", test_half_duplex), ("7 retransmissions without progress", test_without_progress)]


def run_validation_tests(site_params: dict) -> None:
    """Run the validation tests on the site's path-loss parameters; raise if any fails."""
    TEST_SITE_PARAMS.clear()
    TEST_SITE_PARAMS.update(site_params)
    failures = []
    for name, fn in TESTS:
        try:
            print(f"PASS  {name}: {fn()}")
        except AssertionError as exc:
            failures.append(name)
            print(f"FAIL  {name}: {exc}")
    if failures:
        raise RuntimeError(f"Validation failed: {failures}. The main simulation is not run.")
    print("All validation tests passed.")

# ## Part 6 — Running the simulations
#
# One job = one site and one reporting interval. Every simulation (run, method) rebuilds its
# shadowing matrix, scan phases, report times and node streams from the seeds, so the three methods of
# a run share their random draws (common random numbers) whichever process runs them and in whatever
# order. The simulations are spread over `--workers` processes; each worker loads the plan file once
# and then takes the next queued simulation whenever it finishes one. All-relay simulations (by far the
# longest) are queued first so that no worker is left with a long one at the end. Rows are sorted by
# run and method before they are saved, so the output does not depend on the number of workers.
#
# **Checks before the main runs.**
# * *Reproducibility*: one short simulation (3 measured periods) is run twice with the same seed in
#   this process; the metrics must be identical.
# * *Workers = single process*: the three methods of that short run are simulated in the worker pool
#   and in this process; the results must be identical.
#
# `sim_runs.csv` is rewritten after every finished simulation, so an interrupted job keeps the rows
# finished so far.

def simulate(su: SiteUniverse, method: str, interval_min: float, run: int,
             n_periods: int = N_PERIODS) -> tuple[dict, list[dict], MeshSimulator]:
    """Simulate one (site, method, interval, run); returns the metrics row and per-meter rows."""
    sp = su.plans
    h = site_hash(sp.name)
    interval_us = ms(interval_min * 60_000)
    interval_s = int(round(interval_min * 60))
    prx_u = rx_power_matrix(su.xy, su.rp, seed_sequence(h, PURPOSE_SHADOWING, run))
    scan_u = np.random.default_rng(seed_sequence(h, PURPOSE_SCAN, interval_s, run)).integers(
        0, 3 * su.cfg.scan_interval_us, size=len(su.names))
    total_periods = WARMUP_PERIODS + n_periods + COOLDOWN_PERIODS
    times = report_schedule(len(sp.meter_ids), interval_us, total_periods,
                            seed_sequence(h, PURPOSE_TRAFFIC, interval_s, run))
    sc, uidx = build_scenario(su, method, prx_u, scan_u, interval_s, run)
    sim = MeshSimulator(sc, su.cfg)
    schedule = []
    for node, u in enumerate(uidx):
        if sc.is_dcu[node]:
            continue
        for k in range(total_periods):
            schedule.append((int(times[u, k]), node, k, WARMUP_PERIODS <= k < WARMUP_PERIODS + n_periods))
    schedule.sort()
    sim.add_reports(schedule)
    t0 = time.perf_counter()
    sim.run()
    wall = time.perf_counter() - t0
    row, per_meter = summarize_run(sim, su.cfg.n_segments)
    row = {"site": sp.name, "method": method, "interval_min": interval_min, "run": run,
           "offset_mode": OFFSET_MODE, "collision_model": su.cfg.collision_model,
           "n_meters": sum(not d for d in sc.is_dcu), "n_relays": sum(sc.is_relay), "n_dcus": sum(sc.is_dcu),
           **row, "n_events": sim.n_events, "sim_time_s": sim.now / 1e6, "wall_time_s": wall}
    pm_rows = [{"site": sp.name, "method": method, "interval_min": interval_min, "run": run,
                "meter_id": sc.names[node][2:], "n_readings": a, "n_delivered": b}
               for node, (a, b) in sorted(per_meter.items())]
    return row, pm_rows, sim


Task = tuple[str, str, float, int, int]   # (site, method, interval in min, run, measured periods)
_WORKER_UNIVERSES: dict[str, "SiteUniverse"] = {}


def _init_worker(plan_paths: list[str]) -> None:
    """Worker initializer: load every plan file once per process."""
    for path in plan_paths:
        su = site_universe(load_site_plans(Path(path), METHODS))
        _WORKER_UNIVERSES[su.plans.name] = su


def _run_task(task: Task) -> tuple[Task, dict, list[dict]]:
    """Worker entry point: simulate one (site, method, interval, run)."""
    site, method, interval, run, n_periods = task
    row, pm, _ = simulate(_WORKER_UNIVERSES[site], method, interval, run, n_periods=n_periods)
    return task, row, pm


def row_key(row: dict) -> tuple:
    """Sort key of a result row: run, then method in METHODS order."""
    return row["run"], METHODS.index(row["method"])


def without_wall_time(row: dict) -> dict:
    """A result row without its (machine-dependent) wall time."""
    return {k: v for k, v in row.items() if k != "wall_time_s"}


def check_reproducibility(su: "SiteUniverse", interval: float) -> None:
    """Simulate a short run twice with the same seed; the results must be identical."""
    a, pa, _ = simulate(su, METHODS[0], interval, 0, n_periods=3)
    b, pb, _ = simulate(su, METHODS[0], interval, 0, n_periods=3)
    if without_wall_time(a) != without_wall_time(b) or pa != pb:
        raise RuntimeError("Two runs with the same seed differ: the simulation is not reproducible")
    print(f"Reproducibility check passed ({METHODS[0]}, 3 periods: PDR {a['pdr']:.4f}, {a['n_events']} events)")


def check_workers(pool: ProcessPoolExecutor, su: "SiteUniverse", interval: float) -> None:
    """Simulate the three methods of a short run in the pool and here; the results must be identical."""
    tasks = [(su.plans.name, m, interval, 0, 3) for m in METHODS]
    pooled = {t: (row, pm) for t, row, pm in pool.map(_run_task, tasks)}
    for t in tasks:
        row, pm, _ = simulate(su, t[1], interval, 0, n_periods=3)
        if without_wall_time(row) != without_wall_time(pooled[t][0]) or pm != pooled[t][1]:
            raise RuntimeError(f"Results differ between the worker pool and this process ({t[1]})")
    print(f"Workers = single process check passed ({len(tasks)} short simulations)")


def run_all(pool: ProcessPoolExecutor, site: str, interval: float, n_runs: int,
            out: Path) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Run every (run, method) simulation of one job; checkpoint sim_runs.csv after each one."""
    order = sorted(METHODS, key=lambda m: m != "All-relay")          # longest simulations first
    tasks = [(site, m, interval, run, N_PERIODS) for m in order for run in range(n_runs)]
    rows: list[dict] = []
    meter_rows: list[dict] = []
    t0 = time.perf_counter()
    futures = [pool.submit(_run_task, t) for t in tasks]
    for f in as_completed(futures):
        (_, method, _, run, _), row, pm = f.result()
        rows.append(row)
        meter_rows.extend(pm)
        elapsed = time.perf_counter() - t0
        eta = elapsed / len(rows) * (len(tasks) - len(rows))
        print(f"[{len(rows):>4}/{len(tasks)} {elapsed / 60:6.1f} min, ~{eta / 60:5.1f} min left] "
              f"{site} {interval:g} min | {method:<20} | "
              f"run {run + 1:>3}/{n_runs} | PDR {100 * row['pdr']:6.2f} % | latency {row['latency_mean_ms']:7.1f} ms | "
              f"coll. {row['collisions_relay_dcu_per_reading']:8.2f}/reading | {row['wall_time_s']:6.1f} s", flush=True)
        if row["unresolved_readings"]:
            print(f"    warning: {row['unresolved_readings']} measured readings still in progress at the end")
        pd.DataFrame(sorted(rows, key=row_key)).to_csv(out / "sim_runs.csv", index=False)
    print(f"All {len(tasks)} simulations done in {(time.perf_counter() - t0) / 60:.1f} min")
    rows.sort(key=row_key)
    meter_rows.sort(key=lambda r: (*row_key(r), r["meter_id"]))
    return pd.DataFrame(rows), pd.DataFrame(meter_rows)


# ## Part 7 — Results
#
# ### Summary table
#
# Mean over runs and the half-width of the 95 % confidence interval,
# $t_{0.975,\,R-1}\, s / \sqrt{R}$ for $R$ runs (Student t; runs are independent replications).

T975 = [12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179, 2.160, 2.145,
        2.131, 2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048,
        2.045, 2.042]


def t_quantile_975(df: int) -> float:
    """97.5 % quantile of Student's t (table for df <= 30, Cornish-Fisher expansion above)."""
    if df <= 0:
        return float("nan")
    if df <= 30:
        return T975[df - 1]
    z = 1.959963984540054
    return (z + (z**3 + z) / (4 * df) + (5 * z**5 + 16 * z**3 + 3 * z) / (96 * df**2)
            + (3 * z**7 + 19 * z**5 + 17 * z**3 - 15 * z) / (384 * df**3))


SUMMARY_METRICS = ["pdr", "seg_delivery_ratio", "latency_mean_ms", "latency_median_ms", "latency_p95_ms",
                   "latency_from_generation_mean_ms", "tx_data_seg_per_reading", "tx_relay_seg_per_reading",
                   "tx_ack_per_reading", "tx_relay_ack_per_reading", "tx_total_per_reading",
                   "collisions_relay_dcu", "collisions_nonrelay", "collisions_relay_dcu_per_reading",
                   "collisions_nonrelay_per_reading", "collisions_relay_dcu_new_per_reading", "halfduplex_losses", "sar_rounds_per_reading",
                   "frac_readings_retx", "sender_ack_ratio"]


def summarize(runs: pd.DataFrame) -> pd.DataFrame:
    """Mean and 95 % CI half-width per (site, method, interval)."""
    rows = []
    order = {m: i for i, m in enumerate(METHODS)}
    for (site, method, interval), g in runs.groupby(["site", "method", "interval_min"], sort=False):
        row = {"site": site, "method": method, "interval_min": interval, "runs": len(g),
               "n_relays": int(g["n_relays"].iloc[0]), "n_dcus": int(g["n_dcus"].iloc[0])}
        for m in SUMMARY_METRICS:
            x = g[m].dropna().to_numpy(dtype=float)
            row[m] = x.mean() if len(x) else float("nan")
            row[m + "_ci95"] = t_quantile_975(len(x) - 1) * x.std(ddof=1) / math.sqrt(len(x)) if len(x) > 1 else float("nan")
        rows.append(row)
    out = pd.DataFrame(rows)
    out["_o"] = out["method"].map(order)
    return out.sort_values(["site", "interval_min", "_o"], ascending=[True, False, True]).drop(columns="_o").reset_index(drop=True)


def print_summary(summary: pd.DataFrame) -> None:
    """Print the main metrics, mean ± 95 % CI half-width, and the transmissions per reading."""
    show = summary[["method", "runs", "n_relays"]].copy()
    for m, scale, fmt in (("pdr", 100, "{:.2f} ± {:.2f}"), ("seg_delivery_ratio", 100, "{:.2f} ± {:.2f}"),
                          ("latency_mean_ms", 1, "{:.0f} ± {:.0f}"), ("latency_p95_ms", 1, "{:.0f} ± {:.0f}"),
                          ("tx_total_per_reading", 1, "{:.1f} ± {:.1f}"),
                          ("collisions_relay_dcu_per_reading", 1, "{:.2f} ± {:.2f}"),
                          ("sar_rounds_per_reading", 1, "{:.3f} ± {:.3f}")):
        show[m] = [fmt.format(a * scale, b * scale) for a, b in zip(summary[m], summary[m + "_ci95"])]
    show = show.rename(columns={"pdr": "PDR (%)", "seg_delivery_ratio": "segments (%)", "latency_mean_ms": "latency (ms)",
                                "latency_p95_ms": "latency p95 (ms)", "tx_total_per_reading": "tx / reading",
                                "collisions_relay_dcu_per_reading": "coll. relay+DCU / reading",
                                "sar_rounds_per_reading": "SAR rounds / reading"})
    print(show.to_string(index=False))
    tx_cols = ["tx_data_seg_per_reading", "tx_relay_seg_per_reading", "tx_ack_per_reading",
               "tx_relay_ack_per_reading", "tx_total_per_reading", "collisions_relay_dcu",
               "collisions_relay_dcu_new_per_reading", "collisions_nonrelay",
               "halfduplex_losses", "frac_readings_retx", "sender_ack_ratio"]
    print("\nNetwork PDU transmissions per measured reading, by type (mean over runs):")
    print(summary[["method"] + tx_cols].round(3).to_string(index=False))


# ### Is the number of runs enough?
#
# The methods of one run share their random draws, so they are compared **run by run**. For every
# metric in `ADEQUACY_METRICS` and pair of methods A, B:
#
# 1. per run, the difference $d_r = \text{metric}_A - \text{metric}_B$;
# 2. its mean $\bar d$ and 95 % confidence interval $\bar d \pm t_{0.975,\,R-1}\, s_d / \sqrt{R}$
#    over the $R$ runs ($t_{0.975,9} = 2.262$ for 10 runs, 1.984 for 100);
# 3. verdict:
#    * **enough** — the interval excludes zero and its half-width is at most
#      `ADEQUACY_MAX_RELATIVE_HALF_WIDTH` × $|\bar d|$ (default 0.5): the sign and the size of the
#      difference are established;
#    * **flag: includes zero** — the runs cannot tell whether A or B is better;
#    * **flag: wide** — the sign is clear, but the size of the difference is not.
#
# For flagged comparisons, *runs needed* estimates how many runs would give a half-width of
# `ADEQUACY_MAX_RELATIVE_HALF_WIDTH` × $|\bar d|$ if the mean and standard deviation of the
# difference stayed as they are. It is a guide only: if the true difference is (close to) zero, no
# number of runs will make it significant.

def runs_needed(sd: float, mean: float, rel: float, limit: int = 10_000) -> int | None:
    """Smallest R with t_{0.975,R-1} * sd / sqrt(R) <= rel * |mean|, or None if above ``limit``."""
    target = rel * abs(mean)
    if sd == 0:
        return 2 if target > 0 else None
    for r in range(2, limit + 1):
        if t_quantile_975(r - 1) * sd / math.sqrt(r) <= target:
            return r
    return None


def run_adequacy(runs: pd.DataFrame) -> pd.DataFrame:
    """Paired per-run differences between methods, their 95 % CI and a verdict on the number of runs."""
    rows = []
    rel = ADEQUACY_MAX_RELATIVE_HALF_WIDTH
    for (site, interval), g in runs.groupby(["site", "interval_min"], sort=False):
        wide = g.pivot_table(index="run", columns="method", values=ADEQUACY_METRICS, aggfunc="first")
        for metric in ADEQUACY_METRICS:
            for i, a in enumerate(METHODS):
                for b in METHODS[i + 1:]:
                    d = (wide[(metric, a)] - wide[(metric, b)]).dropna().to_numpy(dtype=float)
                    n = len(d)
                    mean = float(d.mean()) if n else float("nan")
                    sd = float(d.std(ddof=1)) if n > 1 else float("nan")
                    t = t_quantile_975(n - 1)
                    hw = t * sd / math.sqrt(n) if n > 1 else float("nan")
                    lo, hi = mean - hw, mean + hw
                    if n < 2:
                        verdict = "flag: fewer than 2 runs"
                    elif sd == 0 and mean == 0:
                        verdict = "flag: includes zero (identical in every run)"
                    elif lo <= 0 <= hi:
                        verdict = "flag: includes zero"
                    elif hw > rel * abs(mean):
                        verdict = "flag: wide"
                    else:
                        verdict = "enough"
                    need = None if verdict == "enough" or n < 2 else runs_needed(sd, mean, rel)
                    rows.append({"site": site, "interval_min": interval, "metric": metric,
                                 "comparison": f"{a} - {b}", "runs": n, "mean_diff": mean, "sd_diff": sd,
                                 "t": t, "ci95_half_width": hw, "ci95_low": lo, "ci95_high": hi,
                                 "half_width_over_mean": hw / abs(mean) if mean else float("inf"),
                                 "verdict": verdict, "runs_needed_estimate": need})
    return pd.DataFrame(rows)


def print_adequacy(adequacy: pd.DataFrame, n_runs: int) -> None:
    """Print the verdict on the number of runs for every comparison."""
    scale = {"pdr": 100.0}   # PDR differences in percentage points
    print(f"Is {n_runs} runs enough? (95 % CI of the per-run difference; 'enough' = excludes 0 and "
          f"half-width <= {ADEQUACY_MAX_RELATIVE_HALF_WIDTH:g} x |mean|)")
    for _, r in adequacy.iterrows():
        k = scale.get(r["metric"], 1.0)
        unit = " pp" if r["metric"] == "pdr" else ""
        need = "" if r["verdict"] == "enough" else (
            f"  -> about {int(r['runs_needed_estimate'])} runs needed" if pd.notna(r["runs_needed_estimate"])
            else "  -> no estimate: the mean difference is 0" if r["mean_diff"] == 0
            else "  -> more than 10,000 runs needed (difference ~ 0)")
        print(f"  {r['metric']:<33} | {r['comparison']:<42} | {k * r['mean_diff']:+9.3f} ± "
              f"{k * r['ci95_half_width']:7.3f}{unit} [{k * r['ci95_low']:+9.3f}, {k * r['ci95_high']:+9.3f}] | "
              f"{r['verdict']}{need}")
    n_flag = int((adequacy["verdict"] != "enough").sum())
    print(f"  -> {len(adequacy) - n_flag} of {len(adequacy)} comparisons are conclusive with {n_runs} runs; "
          + ("all differences are established." if n_flag == 0 else
             f"{n_flag} flagged: increase --runs, or report those differences as not significant."))


# ### LaTeX table
#
# One `tabular` in the layout of the paper's table (mean ± 95 % CI). "Collisions" is the column named
# by `LATEX_COLLISION_METRIC` (default: receptions lost to overlap at relays and DCUs, per reading);
# "Latency" is the mean latency of delivered readings.

def latex_escape(text: str) -> str:
    """Escape LaTeX special characters in plain text."""
    special = {"\\": r"\textbackslash{}", "&": r"\&", "%": r"\%", "$": r"\$", "#": r"\#",
               "_": r"\_", "{": r"\{", "}": r"\}", "~": r"\textasciitilde{}", "^": r"\textasciicircum{}"}
    return "".join(special.get(ch, ch) for ch in text)


def latex_table(summary: pd.DataFrame, interval: float, n_runs: int) -> str:
    """LaTeX tabular: Site | Method | PDR (%) | Collisions | Latency (ms), mean ± 95 % CI."""
    s = summary[summary.interval_min == interval]
    c = LATEX_COLLISION_METRIC
    lines = [f"% Reporting interval {interval:g} min, {OFFSET_MODE} offsets, {COLLISION_MODEL} collision model, "
             f"{n_runs} runs x {N_PERIODS} periods; mean $\\pm$ 95 % CI; Collisions = {c}",
             r"\begin{tabular}{llrrr}", r"\hline",
             r"Site & Method & PDR (\%) & Collisions & Latency (ms) \\", r"\hline"]
    for i, (site, g) in enumerate(s.groupby("site", sort=False)):
        if i:
            lines.append(r"\hline")
        for j, (_, r) in enumerate(g.iterrows()):
            lines.append(f"{latex_escape(site) if j == 0 else ''} & {latex_escape(r['method'])} & "
                         f"${100 * r['pdr']:.2f} \\pm {100 * r['pdr_ci95']:.2f}$ & "
                         f"${r[c]:.2f} \\pm {r[c + '_ci95']:.2f}$ & "
                         f"${r['latency_mean_ms']:.0f} \\pm {r['latency_mean_ms_ci95']:.0f}$ \\\\")
    lines += [r"\hline", r"\end{tabular}"]
    return "\n".join(lines)


# ### Plots
#
# PDR by method (error bars: 95 % confidence intervals), and per-meter PDR on the site map in local
# coordinates only (no basemap). Colour = PDR of each meter over all runs; the darkest red is the
# lowest PDR. Meters that share coordinates are drawn on top of each other, so the one plotted last is
# visible. Thin lines are the planned parent links.

METHOD_COLORS = dict(zip(METHODS, ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]))
plt.rcParams.update({"axes.spines.top": False, "axes.spines.right": False, "axes.grid": True,
                     "grid.color": "0.9", "grid.linewidth": 0.6, "axes.axisbelow": True})


def plot_pdr_by_method(summary: pd.DataFrame, site: str, interval: int, path: Path) -> None:
    """Bar chart of the mean PDR per method with 95 % CI error bars."""
    fig, ax = plt.subplots(figsize=(6, 4.4))
    for k, m in enumerate(METHODS):
        r = summary[summary.method == m].iloc[0]
        bar = ax.bar(k, 100 * r["pdr"], 0.7, yerr=100 * r["pdr_ci95"], capsize=4, color=METHOD_COLORS[m],
                     label=m, error_kw={"elinewidth": 1, "ecolor": "0.25"})[0]
        top = 100 * (r["pdr"] + (r["pdr_ci95"] if np.isfinite(r["pdr_ci95"]) else 0.0))
        ax.annotate(f"{100 * r['pdr']:.2f}", (bar.get_x() + bar.get_width() / 2, top), xytext=(0, 3),
                    textcoords="offset points", ha="center", va="bottom", fontsize=8, color="0.2")
    ax.set_xticks(range(len(METHODS)), METHODS)
    ax.set_ylim(max(0.0, 100 * np.nanmin(summary["pdr"] - summary["pdr_ci95"]) - 5),
                max(101.0, 100 * np.nanmax(summary["pdr"] + summary["pdr_ci95"]) + 1.5))
    ax.set_ylabel("PDR (%)")
    ax.set_title(f"Site {site}, {interval} min interval ({OFFSET_MODE} offsets, {COLLISION_MODEL})", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=FIG_DPI)
    plt.close(fig)


def plot_per_meter_map(sp: SitePlans, per_meter: pd.DataFrame, interval: int, n_runs: int, path: Path) -> None:
    """Per-meter PDR of every method on the site map (local coordinates)."""
    mxy = dict(zip(sp.meter_ids, sp.meter_xy))
    pxy = dict(zip(sp.pole_ids, sp.pole_xy))
    vmin = min(0.9, float(per_meter["pdr"].min()))
    fig, axes = plt.subplots(1, len(METHODS), figsize=(6 * len(METHODS), 6), sharex=True, sharey=True, squeeze=False)
    sc_ = None
    for ax, m in zip(axes[0], METHODS):
        ups = sp.methods[m]
        segs = [[mxy[mid], pxy[par] if is_dcu else mxy[par]]
                for up in ups for mid, (par, is_dcu) in up.meter_parent.items()]
        ax.add_collection(LineCollection(segs, colors="0.75", linewidths=0.6, zorder=1))
        ax.scatter(*sp.pole_xy.T, marker="s", s=12, facecolors="none", edgecolors="0.7", linewidths=0.5, zorder=2)
        g = per_meter[per_meter.method == m].set_index("meter_id")
        ids = [i for i in sp.meter_ids if i in g.index]
        xy = np.array([mxy[i] for i in ids])
        relays = {r for up in ups for r in up.relay_ids}
        sc_ = ax.scatter(*xy.T, c=g.loc[ids, "pdr"], cmap=plt.get_cmap("Reds_r"), vmin=vmin, vmax=1.0, s=42,
                         edgecolors=["k" if i in relays else "0.5" for i in ids],
                         linewidths=[1.6 if i in relays else 0.4 for i in ids], zorder=3)
        ax.scatter(*np.array([pxy[up.dcu_pole] for up in ups]).T, marker="*", s=300, c="#eda100",
                   edgecolors="k", zorder=4)
        mean_pdr = g["n_delivered"].sum() / g["n_readings"].sum()
        ax.set_title(f"{m}: {len(relays)} relays, PDR {100 * mean_pdr:.2f} %", fontsize=10)
        ax.set_aspect("equal")
        ax.set_xlabel("x (m, local)")
    axes[0][0].set_ylabel("y (m, local)")
    fig.subplots_adjust(left=0.05, right=0.9, bottom=0.14, top=0.9, wspace=0.12)
    fig.colorbar(sc_, cax=fig.add_axes([0.92, 0.2, 0.012, 0.62]), label="per-meter PDR")
    handles = [Line2D([], [], marker="*", ls="", ms=14, mfc="#eda100", mec="k", label="DCU"),
               Line2D([], [], marker="o", ls="", mfc="w", mec="k", mew=1.6, label="relay meter"),
               Line2D([], [], marker="o", ls="", mfc="w", mec="0.5", label="non-relay meter"),
               Line2D([], [], marker="s", ls="", mfc="none", mec="0.7", label="pole"),
               Line2D([], [], c="0.75", label="planned parent link")]
    fig.legend(handles=handles, loc="lower center", ncol=5, frameon=False)
    fig.suptitle(f"Site {sp.name}: per-meter PDR, {interval} min interval ({n_runs} runs)")
    fig.savefig(path, dpi=FIG_DPI, bbox_inches="tight")
    plt.close(fig)


def write_results(su: "SiteUniverse", interval: int, n_runs: int, runs: pd.DataFrame,
                  per_meter_runs: pd.DataFrame, out: Path) -> None:
    """Summary, check of the number of runs, LaTeX table, per-meter PDR and plots."""
    site = su.plans.name
    summary = summarize(runs)
    summary.to_csv(out / "sim_summary.csv", index=False)
    print()
    print_summary(summary)

    adequacy = run_adequacy(runs)
    adequacy.to_csv(out / "sim_run_adequacy.csv", index=False)
    print()
    print_adequacy(adequacy, n_runs)

    latex = latex_table(summary, interval, n_runs)
    (out / "sim_table.tex").write_text(latex + "\n")
    print()
    print(latex)

    per_meter = (per_meter_runs.groupby(["method", "meter_id"], sort=False)[["n_readings", "n_delivered"]]
                 .sum().reset_index())
    per_meter["pdr"] = per_meter["n_delivered"] / per_meter["n_readings"]
    per_meter.insert(0, "site", site)
    per_meter.insert(2, "interval_min", interval)
    per_meter.to_csv(out / "sim_per_meter.csv", index=False)
    worst = per_meter.sort_values("pdr").groupby("method", sort=False).head(5)
    print("\nFive lowest per-meter PDRs per method:")
    print(worst[["method", "meter_id", "n_readings", "n_delivered", "pdr"]].to_string(index=False))

    plot_pdr_by_method(summary, site, interval, out / "figures" / "pdr_by_method.png")
    plot_per_meter_map(su.plans, per_meter, interval, n_runs, out / "figures" / "per_meter_pdr.png")
    print(f"\nSaved in {out}: sim_runs.csv, sim_per_meter_runs.csv, sim_summary.csv, sim_run_adequacy.csv, "
          f"sim_table.tex, sim_per_meter.csv, sim_metadata.json, figures/")


# ## Command-line entry point

def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Command-line arguments (all optional)."""
    ap = argparse.ArgumentParser(description="Discrete-event simulation of Bluetooth Mesh metering plans: "
                                             "every <site>_plans.json in --plans-dir x every interval.")
    ap.add_argument("--plans-dir", type=Path, default=PLANS_DIR,
                    help=f"folder with <site>_plans.json files (default {PLANS_DIR})")
    ap.add_argument("--intervals", type=int, nargs="+", default=INTERVALS_MIN,
                    help=f"reporting intervals in minutes (default {' '.join(map(str, INTERVALS_MIN))})")
    ap.add_argument("--runs", type=int, default=N_RUNS, help=f"Monte Carlo runs per job (default {N_RUNS})")
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 1,
                    help="worker processes (default: every CPU)")
    ap.add_argument("--out-root", type=Path, default=OUTPUT_ROOT,
                    help=f"outputs go to <out-root>/<site>_<interval>/ (default {OUTPUT_ROOT})")
    args = ap.parse_args(argv)
    if min(args.intervals) < 1:
        ap.error("intervals must be at least 1 minute")
    if len(set(args.intervals)) != len(args.intervals):
        ap.error("intervals must be distinct")
    if args.runs < 2:
        ap.error("--runs must be at least 2 (confidence intervals)")
    if args.workers < 1:
        ap.error("--workers must be at least 1")
    return args


def run_job(pool: ProcessPoolExecutor, su: "SiteUniverse", interval: int, args: argparse.Namespace) -> pd.DataFrame:
    """One site and one interval: checks, all simulations, results. Returns the summary."""
    sp = su.plans
    out = args.out_root / f"{sp.name}_{interval}"
    (out / "figures").mkdir(parents=True, exist_ok=True)
    print(f"\n=== Job: site {sp.name}, {interval} min interval, {args.runs} runs -> {out} ===")
    check_reproducibility(su, interval)
    check_workers(pool, su, interval)
    config = {k: v for k, v in globals().items()
              if k.isupper() and isinstance(v, (int, float, str, list, tuple, type(None), Path))}
    (out / "sim_metadata.json").write_text(json.dumps(
        {"site": sp.name, "interval_min": interval, "runs": args.runs, "workers": args.workers,
         "plan_file": str(sp.path.resolve()), "versions": VERSIONS, "config": config,
         "n_segments": N_SEGMENTS, "airtime_us": AIRTIME_US}, indent=1, default=str))
    runs, per_meter_runs = run_all(pool, sp.name, interval, args.runs, out)
    runs.to_csv(out / "sim_runs.csv", index=False)
    per_meter_runs.to_csv(out / "sim_per_meter_runs.csv", index=False)
    write_results(su, interval, args.runs, runs, per_meter_runs, out)
    return summarize(runs)


def main(argv: list[str] | None = None) -> None:
    """Every <site>_plans.json in the plans folder x every interval: one job each."""
    args = parse_args(argv)
    t_start = time.perf_counter()
    paths = sorted(args.plans_dir.glob("*_plans.json"))
    if not paths:
        raise SystemExit(f"No '*_plans.json' in {args.plans_dir.resolve()}.")
    universes = [site_universe(load_site_plans(p, METHODS)) for p in paths]
    names = [su.plans.name for su in universes]
    if len(set(names)) != len(names):
        raise SystemExit(f"Two plan files have the same site name: {names}")
    intervals = sorted(args.intervals)
    print_setup(intervals, args.runs, args.workers)
    print(f"{len(universes)} site(s) x {len(intervals)} interval(s) = {len(universes) * len(intervals)} jobs, "
          f"{len(universes) * len(intervals) * args.runs * len(METHODS)} simulations")
    for su in universes:
        print(f"  {su.plans.path}")

    summaries = []
    with ProcessPoolExecutor(max_workers=args.workers, initializer=_init_worker,
                             initargs=([str(p.resolve()) for p in paths],)) as pool:
        for su in universes:
            print()
            print_plans(su.plans)
            rp = su.rp
            print(f"Radio: EIRP {rp.eirp_dbm} dBm, PL(d0) {rp.pl_d0_db:.2f} dB, n {rp.path_loss_exp}, "
                  f"sigma {rp.shadowing_sigma_db} dB, RX threshold {rp.rx_threshold_dbm} dBm, "
                  f"interference threshold {rp.interference_threshold_dbm} dBm")
            run_validation_tests(su.plans.params)
            for interval in intervals:
                summaries.append(run_job(pool, su, interval, args))
                all_summary = pd.concat(summaries, ignore_index=True)
                all_summary.to_csv(args.out_root / "summary_all.csv", index=False)

    print("\n=== All jobs: PDR (%), mean latency (ms), collisions at relays and DCUs per reading "
          "(mean ± 95 % CI) ===")
    for _, r in all_summary.iterrows():
        print(f"  {r['site']:<16} {r['interval_min']:>3g} min | {r['method']:<20} | "
              f"PDR {100 * r['pdr']:6.2f} ± {100 * r['pdr_ci95']:5.2f} | "
              f"latency {r['latency_mean_ms']:7.1f} ± {r['latency_mean_ms_ci95']:5.1f} | "
              f"coll. {r['collisions_relay_dcu_per_reading']:9.2f} ± {r['collisions_relay_dcu_per_reading_ci95']:7.2f}")
    print(f"Saved {args.out_root / 'summary_all.csv'}")
    print(f"Total time {(time.perf_counter() - t_start) / 60:.1f} min")


if __name__ == "__main__":
    main()
