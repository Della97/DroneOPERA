"""
rpi5_profiler.py  –  Raspberry Pi 5 compute-energy profiler
=============================================================

PURPOSE
-------
Run this script **on the Raspberry Pi 5** to collect real-world power-consumption
data while simulating the same compute workloads that the NS3 drones perform
(federated-learning local training iterations).

The sampling interval mirrors the NS3 pktInterval so every row in the output CSV
maps directly to one discrete NS3 simulation step.

REQUIREMENTS (RPi5)
--------------------
    pip install numpy psutil

USAGE
-----
    python3 rpi5_profiler.py --config scenario.json --step 1.0 --output rpi5_data.csv

    # Or run standalone with manual parameters:
    python3 rpi5_profiler.py --freq 1500 --ops 6e8 --iters 50 --step 1.0

VCGENCMD POWER READING  (RPi 5 only)
--------------------------------------
    vcgencmd pmic_read_adc

    Relevant rails:
      EXT5V_V / EXT5V_A   → main 5 V input (total board power)
      VDD_CORE_V / VDD_CORE_A → CPU core rail
      VDD_CORE_W           → directly reported CPU core power (Watts)

    We derive:
      P_total_W  = EXT5V_V  * EXT5V_A
      P_core_W   = VDD_CORE_V * VDD_CORE_A   (or use VDD_CORE_W directly)
"""

import subprocess
import time
import csv
import math
import json
import argparse
import re
import sys
from datetime import datetime
from pathlib import Path

# ---------------------------------------------------------------------------
# vcgencmd helpers
# ---------------------------------------------------------------------------

def _run(cmd: str) -> str:
    """Run a shell command and return stdout, or empty string on error."""
    try:
        result = subprocess.run(
            cmd.split(), capture_output=True, text=True, timeout=2
        )
        return result.stdout.strip()
    except Exception:
        return ""


def read_pmic() -> dict:
    """
    Parse `vcgencmd pmic_read_adc` output.

    Returns a dict with keys like:
        EXT5V_V, EXT5V_A, VDD_CORE_V, VDD_CORE_A, VDD_CORE_W, ...

    Values are floats (unit already stripped).
    Falls back to zeros if vcgencmd is unavailable (e.g., when testing on x86).
    """
    raw = _run("vcgencmd pmic_read_adc")
    result = {}

    # Each field looks like:  EXT5V_V  5.0985V   or   VDD_CORE_W  2.4815W
    for token in raw.split():
        m = re.match(r"([A-Z0-9_]+)=([\d.]+)[VAWO]?", token)
        if m:
            result[m.group(1)] = float(m.group(2))

    # Alternative format on some firmware versions: "name  value[unit]" pairs
    if not result:
        for m in re.finditer(r"(\w+)\s+([\d.]+)[VAWO]?", raw):
            result[m.group(1)] = float(m.group(2))

    return result


def read_cpu_freq_mhz() -> float:
    """Current CPU frequency in MHz from vcgencmd."""
    raw = _run("vcgencmd measure_clock arm")   # returns "frequency(48)=1800000000"
    m = re.search(r"=(\d+)", raw)
    if m:
        return int(m.group(1)) / 1e6
    # Fallback: /sys
    try:
        freq = Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq").read_text()
        return int(freq.strip()) / 1e3   # kHz → MHz
    except Exception:
        return 0.0


def read_cpu_temp() -> float:
    """CPU temperature in °C."""
    raw = _run("vcgencmd measure_temp")        # "temp=52.3'C"
    m = re.search(r"temp=([\d.]+)", raw)
    if m:
        return float(m.group(1))
    try:
        t = Path("/sys/class/thermal/thermal_zone0/temp").read_text()
        return int(t.strip()) / 1000.0
    except Exception:
        return 0.0


def read_power_watts() -> tuple[float, float]:
    """
    Returns (P_total_W, P_core_W).

    P_total = EXT5V_V * EXT5V_A   (entire board)
    P_core  = VDD_CORE_W  if available, else VDD_CORE_V * VDD_CORE_A
    """
    pmic = read_pmic()

    p_total = pmic.get("EXT5V_V", 0.0) * pmic.get("EXT5V_A", 0.0)
    if p_total == 0.0:
        # Firmware version that reports W directly
        p_total = pmic.get("EXT5V_W", 0.0)

    p_core = pmic.get("VDD_CORE_W", 0.0)
    if p_core == 0.0:
        p_core = pmic.get("VDD_CORE_V", 0.0) * pmic.get("VDD_CORE_A", 0.0)

    return p_total, p_core


# ---------------------------------------------------------------------------
# Compute workload  (matches drone FL training parameters)
# ---------------------------------------------------------------------------

def _dot_product_workload(num_ops: int) -> None:
    """
    Pure-Python busy-loop that performs approximately `num_ops` floating-point
    multiply-add operations.  Replace with actual ML inference/training code
    if available (e.g., numpy matmul, PyTorch forward pass).
    """
    import numpy as np
    # Each matmul(n,n) @ (n,n) does ~ 2*n^3 FLOP
    # Target n so that 2*n^3 ≈ num_ops
    n = max(8, int((num_ops / 2) ** (1 / 3)))
    a = np.random.rand(n, n).astype(np.float32)
    b = np.random.rand(n, n).astype(np.float32)
    _ = a @ b   # single matmul


def run_workload_for_step(
    num_iters: int,
    num_ops_per_iter: float,
    step_seconds: float,
) -> dict:
    """
    Run `num_iters` training iterations inside one NS3 step window.
    Measures wall-clock power during execution.

    Returns timing and power statistics for this step.
    """
    samples_total: list[float] = []
    samples_core:  list[float]  = []

    t_end = time.monotonic() + step_seconds
    ops_per_iter = int(num_ops_per_iter)

    iters_done = 0
    while time.monotonic() < t_end:
        _dot_product_workload(ops_per_iter)
        iters_done += 1

        # Sample power after each iteration
        p_total, p_core = read_power_watts()
        samples_total.append(p_total)
        samples_core.append(p_core)

    avg_total = sum(samples_total) / max(len(samples_total), 1)
    avg_core  = sum(samples_core)  / max(len(samples_core), 1)

    return {
        "iters_done":   iters_done,
        "avg_total_W":  round(avg_total, 4),
        "avg_core_W":   round(avg_core,  4),
        "n_samples":    len(samples_total),
    }


# ---------------------------------------------------------------------------
# Workload phases  (mirror NS3 mobility states)
# ---------------------------------------------------------------------------

# NS3 mobility states from CustomMobilityModel:
#   0 = Ascending  (atEight==false): drone climbing to max height, Pi is idle
#   1 = Snake scan, outside AoI strip: drone briefly between rows, Pi still computing
#   2 = Snake scan, inside AoI: sensors ON, Pi at full compute load
#   3 = Descending (descend==true): drone going back down, Pi still computing
#
# Key behaviour: once the drone enters the AoI for the FIRST TIME (state 2),
# the Pi runs the FL workload continuously until the end of the simulation.
# It does NOT pause in states 1 or 3.  Hardware sensors (hwA) are handled
# separately in main.cpp and are only active in state 2.

PHASE_IDLE    = "idle"       # state 0: ascending, no computation
PHASE_PARTIAL = "partial"    # state 1: between AoI strips, full compute
PHASE_FULL    = "full"       # state 2: inside AoI, full compute + sensors
PHASE_RETURN  = "return"     # state 3: descending, full compute

# Fraction of full workload per phase.
# partial and return are 1.0 because the Pi keeps running at full intensity
# once it has started — it does not know or care about the drone's position.
PHASE_OPS_FRACTION = {
    PHASE_IDLE:    0.0,   # ascending: no computation yet
    PHASE_PARTIAL: 1.0,   # between AoI strips: Pi still running full workload
    PHASE_FULL:    1.0,   # inside AoI: full workload (sensors added separately)
    PHASE_RETURN:  1.0,   # descending: Pi still running full workload
}


# ---------------------------------------------------------------------------
# Main profiling loop
# ---------------------------------------------------------------------------

def load_scenario(path: str) -> dict:
    """Load the first drone config from the NS3 scenario.json."""
    with open(path) as f:
        data = json.load(f)
    return data["Drones"][0]   # use drone 0 as reference


def build_phases(num_steps: int) -> list[str]:
    """
    Approximate the NS3 simulation phase sequence.
    Divide the total steps into four blocks (ascend, approach, AoI, return).
    """
    q = num_steps // 4
    return (
        [PHASE_IDLE]    * q +
        [PHASE_PARTIAL] * q +
        [PHASE_FULL]    * q +
        [PHASE_RETURN]  * (num_steps - 3 * q)
    )


def profile(
    cpu_freq_mhz:    float,
    ops_per_step:    float,     # total FLOP per NS3 step at full load
    step_seconds:    float,
    num_steps:       int,
    output_csv:      str,
    drone_voltage_v: float = 11.1,   # nominal LiPo pack voltage
) -> None:
    """
    Main loop: for each simulated NS3 step, run the appropriate workload,
    sample power, and write one CSV row.
    """
    phases = build_phases(num_steps)

    fieldnames = [
        "step",
        "timestamp",
        "phase",
        "cpu_freq_mhz",
        "cpu_temp_c",
        "ops_executed",
        "avg_total_W",
        "avg_core_W",
        "energy_step_J",      # avg_total_W * step_seconds
        "drain_A",            # energy_step_J / step_seconds / drone_voltage_v
        "battery_pct",        # cumulative % drained from a 200 Wh pack
    ]

    # Reference battery energy in Joules  (200 Wh default from scenario.json)
    BATTERY_WH   = 200.0
    battery_J    = BATTERY_WH * 3600.0
    battery_J_0  = battery_J

    print(f"\n{'Step':>5}  {'Phase':<8}  {'Freq MHz':>9}  {'Temp °C':>8}  "
          f"{'Total W':>8}  {'Core W':>7}  {'Step J':>8}  {'Bat %':>6}")
    print("-" * 75)

    with open(output_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for step_idx, phase in enumerate(phases):
            frac    = PHASE_OPS_FRACTION[phase]
            step_ops = ops_per_step * frac

            t_start = time.monotonic()

            if frac > 0:
                stats = run_workload_for_step(
                    num_iters=1,
                    num_ops_per_iter=step_ops,
                    step_seconds=step_seconds,
                )
            else:
                # Idle: just wait and sample power
                time.sleep(step_seconds)
                p_total, p_core = read_power_watts()
                stats = {
                    "iters_done":  0,
                    "avg_total_W": p_total,
                    "avg_core_W":  p_core,
                    "n_samples":   1,
                }

            freq = read_cpu_freq_mhz() or cpu_freq_mhz
            temp = read_cpu_temp()

            energy_J   = stats["avg_total_W"] * step_seconds
            drain_A    = energy_J / step_seconds / drone_voltage_v  # = W / V
            battery_J -= energy_J
            bat_pct    = (battery_J / battery_J_0) * 100.0

            row = {
                "step":          step_idx,
                "timestamp":     datetime.utcnow().isoformat(),
                "phase":         phase,
                "cpu_freq_mhz":  round(freq, 1),
                "cpu_temp_c":    round(temp, 1),
                "ops_executed":  int(step_ops),
                "avg_total_W":   stats["avg_total_W"],
                "avg_core_W":    stats["avg_core_W"],
                "energy_step_J": round(energy_J,  4),
                "drain_A":       round(drain_A,   5),
                "battery_pct":   round(bat_pct,   3),
            }
            writer.writerow(row)

            print(
                f"{step_idx:>5}  {phase:<8}  {freq:>9.1f}  {temp:>8.1f}  "
                f"{stats['avg_total_W']:>8.3f}  {stats['avg_core_W']:>7.3f}  "
                f"{energy_J:>8.3f}  {bat_pct:>6.2f}"
            )

    print(f"\nDone. Data written to: {output_csv}")


# ---------------------------------------------------------------------------
# Frequency sweep  (extra calibration run — produces a P vs f table)
# ---------------------------------------------------------------------------

def frequency_sweep(
    freq_steps_mhz: list[float],
    ops_per_step:   float,
    measure_secs:   float,
    output_csv:     str,
) -> None:
    """
    Hold the CPU at each frequency via cpufreq-set (requires cpufrequtils),
    run a fixed workload, and record power.  Used to fit P = a·f^b.

    Requires: sudo cpufreq-set -f <MHz>MHz  (or manual governor pinning)
    """
    fieldnames = ["freq_mhz", "cpu_temp_c", "avg_total_W", "avg_core_W"]

    print("\nFrequency sweep (calibration)")
    print(f"{'Freq MHz':>9}  {'Temp':>6}  {'Total W':>8}  {'Core W':>7}")
    print("-" * 40)

    with open(output_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for freq in freq_steps_mhz:
            # Pin frequency (requires cpufrequtils + sudo)
            _run(f"sudo cpufreq-set -f {int(freq * 1e3)}kHz")
            time.sleep(0.5)   # settle

            stats = run_workload_for_step(
                num_iters=1,
                num_ops_per_iter=ops_per_step,
                step_seconds=measure_secs,
            )
            temp = read_cpu_temp()

            row = {
                "freq_mhz":    freq,
                "cpu_temp_c":  temp,
                "avg_total_W": stats["avg_total_W"],
                "avg_core_W":  stats["avg_core_W"],
            }
            writer.writerow(row)

            print(
                f"{freq:>9.0f}  {temp:>6.1f}  "
                f"{stats['avg_total_W']:>8.3f}  {stats['avg_core_W']:>7.3f}"
            )

    # Restore governor
    _run("sudo cpufreq-set -g ondemand")
    print(f"\nSweep done. Written to: {output_csv}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description="RPi5 compute energy profiler for DroneOPERA")
    sub = p.add_subparsers(dest="command", required=True)

    # --- profile subcommand ---
    prof = sub.add_parser("profile", help="Run NS3-step-aligned power profiling")
    prof.add_argument("--config",   default="../scenario/scenario.json",
                      help="Path to NS3 scenario.json (default: ../scenario/scenario.json)")
    prof.add_argument("--step",     type=float, default=1.0,
                      help="NS3 pktInterval in seconds (default: 1.0)")
    prof.add_argument("--steps",    type=int,   default=200,
                      help="Total number of NS3 steps to simulate (default: 200)")
    prof.add_argument("--freq",     type=float, default=1500.0,
                      help="CPU frequency MHz, overridden by config if available (default: 1500)")
    prof.add_argument("--voltage",  type=float, default=11.1,
                      help="Drone LiPo pack voltage for drain calculation (default: 11.1 V)")
    prof.add_argument("--output",   default="rpi5_profile.csv",
                      help="Output CSV filename (default: rpi5_profile.csv)")

    # --- sweep subcommand ---
    swp = sub.add_parser("sweep", help="Frequency sweep for P=f(freq) calibration")
    swp.add_argument("--freqs", nargs="+", type=float,
                     default=[600, 900, 1200, 1500, 1800, 2400],
                     help="CPU frequency steps in MHz")
    swp.add_argument("--ops",    type=float, default=6e8,
                     help="FLOP per measurement window (default: 6e8)")
    swp.add_argument("--secs",   type=float, default=5.0,
                     help="Seconds per frequency step (default: 5.0)")
    swp.add_argument("--output", default="rpi5_sweep.csv",
                     help="Output CSV filename (default: rpi5_sweep.csv)")

    return p.parse_args()


def main():
    args = parse_args()

    if args.command == "profile":
        # Derive ops_per_step from scenario if available
        ops_per_step = 0.0
        cpu_freq_mhz = args.freq

        if Path(args.config).exists():
            drone = load_scenario(args.config)
            # Total FLOP per training round:
            # cyclexop * opxdata * Dn * numLocalIter
            ops_per_step = (
                drone.get("cpuCyclePerOperation", 3.0)
                * drone.get("operationPerData", 20000.0)
                * drone.get("numbTrainDataSet", 60.0)
                * drone.get("numLocalIter", 10000.0)
            )
            cpu_freq_mhz = drone.get("cpuFreq", 1.5) * 1000.0
            drone_voltage = drone.get("voltage", 11.1)
            print(f"Loaded scenario: {args.config}")
            print(f"  ops/step  = {ops_per_step:.3e}")
            print(f"  cpu freq  = {cpu_freq_mhz} MHz")
        else:
            ops_per_step = 6e8
            drone_voltage = args.voltage
            print(f"Scenario not found, using ops_per_step={ops_per_step:.2e}")

        profile(
            cpu_freq_mhz    = cpu_freq_mhz,
            ops_per_step    = ops_per_step,
            step_seconds    = args.step,
            num_steps       = args.steps,
            output_csv      = args.output,
            drone_voltage_v = drone_voltage,
        )

    elif args.command == "sweep":
        frequency_sweep(
            freq_steps_mhz = args.freqs,
            ops_per_step   = args.ops,
            measure_secs   = args.secs,
            output_csv     = args.output,
        )


if __name__ == "__main__":
    main()
