"""
compute_energy_model.py  –  RPi5 → NS3 compute-energy bridge
=============================================================

PURPOSE
-------
1.  Load the CSV produced by rpi5_profiler.py (real power measurements).
2.  Fit a parametric power model  P_compute = f(freq, ops)  using least squares.
3.  Map the model onto every NS3 discrete simulation step.
4.  Estimate battery depletion including the empirical formula already in energy.cpp
    so you can compare theory vs measurement.
5.  Export fitted coefficients as JSON (feed back into scenario.json or C++).
6.  Plot everything.

MODELS FITTED
-------------
A)  Dynamic power law (physics-motivated, matches energy.cpp calcCompPower):
        P_dyn = alpha * C_eff * f^beta
    where C_eff ≈ switch_capacitance * V^2 and beta ≈ 1–3.

B)  Polynomial regression  P = a0 + a1*f + a2*f^2   (baseline / sanity check)

C)  Per-phase lookup table (idle / partial / full / return)  – simplest to plug
    directly into the C++ simulation as constants read from scenario.json.

NS3 MAPPING
-----------
Each DroneLogic() call = one step of duration Δt (pktInterval seconds).
For step i with mobility state s_i:

    E_compute_step[i]  = P_fit(freq, ops * phase_fraction[s_i]) * Δt   [Joules]
    I_compute_step[i]  = E_compute_step[i] / (Δt * V_pack)             [Amperes]

This I value can replace  drone.calculateComputePower()/1.3  in DroneLogic().

REQUIREMENTS
------------
    pip install numpy scipy matplotlib pandas scikit-learn
"""

from __future__ import annotations

import json
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.optimize import curve_fit
from scipy.stats import pearsonr
def r2_score(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    ss_res = np.sum((y_true - y_pred) ** 2)
    ss_tot = np.sum((y_true - y_true.mean()) ** 2)
    return 1 - ss_res / ss_tot if ss_tot > 0 else 0.0

def rmse(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    return float(np.sqrt(np.mean((y_true - y_pred) ** 2)))


# ──────────────────────────────────────────────────────────────────────────────
# Constants (kept in sync with energy.cpp)
# ──────────────────────────────────────────────────────────────────────────────

# Once the drone enters the AoI for the first time, the Pi runs FL compute
# continuously — it does not pause between AoI strips (state 1) or during
# descent (state 3).  Hardware sensors are handled separately in main.cpp.
PHASE_OPS_FRACTION = {
    "idle":    0.0,   # state 0: ascending, no computation
    "partial": 1.0,   # state 1: between AoI strips, Pi at full compute
    "full":    1.0,   # state 2: inside AoI, full compute (sensors added separately)
    "return":  1.0,   # state 3: descending, Pi at full compute
}

# NS3 mobility state → phase name (matches DroneLogic() in main.cpp)
STATE_TO_PHASE = {0: "idle", 1: "partial", 2: "full", 3: "return"}


# ──────────────────────────────────────────────────────────────────────────────
# Theoretical model  (energy.cpp  calcCompPower)
# ──────────────────────────────────────────────────────────────────────────────

def theoretical_compute_power(
    switch_cap: float,   # γ  (F)
    voltage:    float,   # V  (core voltage, e.g. 1.3 V)
    cyc_per_op: float,   # cycles per operation
    ops_data:   float,   # operations per data sample
    n_data:     float,   # number of training samples Dn
    iters:      float,   # local iterations I
) -> float:
    """
    Replicates  calcCompPower()  from energy.cpp.

        P = (γ · V² · cyc_per_op · ops_data · Dn · I) / V
          = γ · V · cyc_per_op · ops_data · Dn · I
    """
    return (switch_cap * voltage**2 * cyc_per_op * ops_data * n_data * iters) / voltage


# ──────────────────────────────────────────────────────────────────────────────
# Model A – dynamic-power law  (physics-motivated)
# ──────────────────────────────────────────────────────────────────────────────

def power_law_model(X: np.ndarray, alpha: float, beta: float) -> np.ndarray:
    """
    P_dyn(f, ops) = alpha * f^beta

    X has shape (N, 2): columns = [freq_mhz, log10_ops]
    We absorb ops as a linear scale: P = alpha * f^beta * ops / ops_ref
    """
    freq_mhz = X[:, 0]
    ops_norm = X[:, 1]          # normalised ops (ops / ops_ref)
    return alpha * (freq_mhz ** beta) * ops_norm


def fit_power_law(df: pd.DataFrame, ops_ref: float) -> dict:
    """
    Fit  P_total = alpha * freq^beta * (ops / ops_ref)
    using non-linear least squares on the 'full' phase rows where ops > 0.
    """
    mask = df["phase"].isin(["full", "partial", "return"]) & (df["ops_executed"] > 0)
    sub  = df[mask].copy()

    if len(sub) < 3:
        print("[WARN] Not enough non-idle rows to fit power law. Need ≥ 3.")
        return {}

    ops_norm = sub["ops_executed"].values / ops_ref
    freq     = sub["cpu_freq_mhz"].values
    P_meas   = sub["avg_total_W"].values

    X = np.column_stack([freq, ops_norm])

    try:
        popt, pcov = curve_fit(
            power_law_model, X, P_meas,
            p0=[1e-3, 1.5],
            bounds=([0, 0.5], [10, 4.0]),
            maxfev=10_000,
        )
        alpha, beta = popt
        P_pred   = power_law_model(X, *popt)
        r2_val   = r2_score(P_meas, P_pred)
        rmse_val = rmse(P_meas, P_pred)

        print(f"\n[Model A – Power Law]")
        print(f"  P = {alpha:.6f} · f^{beta:.4f} · (ops / {ops_ref:.2e})")
        print(f"  R² = {r2_val:.4f}   RMSE = {rmse_val:.4f} W")

        return {"alpha": alpha, "beta": beta, "ops_ref": ops_ref, "r2": r2_val, "rmse": rmse_val}

    except RuntimeError as e:
        print(f"[WARN] Power law fit failed: {e}")
        return {}


# ──────────────────────────────────────────────────────────────────────────────
# Model B – polynomial regression  P = a0 + a1·f + a2·f²
# ──────────────────────────────────────────────────────────────────────────────

def fit_polynomial(df: pd.DataFrame, degree: int = 2) -> dict:
    """Polynomial regression on cpu_freq_mhz → avg_total_W (full phase only)."""
    mask = df["phase"] == "full"
    sub  = df[mask]

    if len(sub) < (degree + 1):
        print(f"[WARN] Not enough 'full' rows for degree-{degree} poly fit.")
        return {}

    X = sub["cpu_freq_mhz"].values
    y = sub["avg_total_W"].values

    # np.polyfit returns coefficients highest-degree first; reverse for [a0, a1, a2]
    coeffs_hp = np.polyfit(X, y, deg=degree)
    coeffs    = coeffs_hp[::-1].tolist()   # [a0, a1, ..., a_degree]

    y_pred = np.polyval(coeffs_hp, X)
    r2_val = r2_score(y, y_pred)
    rmse_val = rmse(y, y_pred)

    print(f"\n[Model B – Polynomial (degree {degree})]")
    terms = [f"{c:.6f}·f^{i}" for i, c in enumerate(coeffs)]
    print(f"  P = " + " + ".join(terms))
    print(f"  R² = {r2_val:.4f}   RMSE = {rmse_val:.4f} W")

    return {"coeffs": coeffs, "degree": degree, "r2": r2_val, "rmse": rmse_val}


# ──────────────────────────────────────────────────────────────────────────────
# Model C – per-phase mean power  (simplest; plug directly into scenario.json)
# ──────────────────────────────────────────────────────────────────────────────

def fit_phase_means(df: pd.DataFrame) -> dict:
    """Return mean and std of avg_total_W for each phase."""
    result = {}
    for phase in PHASE_OPS_FRACTION:
        sub = df[df["phase"] == phase]["avg_total_W"]
        if len(sub) == 0:
            result[phase] = {"mean_W": 0.0, "std_W": 0.0}
        else:
            result[phase] = {
                "mean_W": round(float(sub.mean()), 4),
                "std_W":  round(float(sub.std()),  4),
            }

    print(f"\n[Model C – Per-phase mean power]")
    for ph, v in result.items():
        print(f"  {ph:<8}  {v['mean_W']:>7.3f} ± {v['std_W']:.3f} W")

    return result


# ──────────────────────────────────────────────────────────────────────────────
# NS3 step-by-step mapping
# ──────────────────────────────────────────────────────────────────────────────

def map_to_ns3_steps(
    df:             pd.DataFrame,
    model_a:        dict,
    model_c:        dict,
    ops_per_step:   float,
    step_s:         float,
    drone_voltage:  float,
    battery_wh:     float,
) -> pd.DataFrame:
    """
    For every row (= NS3 step) in df, compute:
      - E_measured_J : directly from the profiler CSV
      - E_model_A_J  : from the power-law fit
      - E_model_C_J  : from the per-phase mean
      - I_drain_A    : amperes to set in NS3 (= W / V_pack)
      - battery_pct  : cumulative battery state (all three tracks)
    """
    n = len(df)
    records = []

    battery_meas  = battery_wh * 3600
    battery_A     = battery_wh * 3600
    battery_C     = battery_wh * 3600
    batt_0        = battery_wh * 3600

    for _, row in df.iterrows():
        phase      = row["phase"]
        freq       = row["cpu_freq_mhz"]
        ops        = row["ops_executed"]
        ops_norm   = ops / model_a.get("ops_ref", ops_per_step) if model_a else 0.0

        # --- Measured energy ---
        E_meas = row["avg_total_W"] * step_s

        # --- Model A ---
        if model_a:
            P_A  = model_a["alpha"] * (freq ** model_a["beta"]) * ops_norm
            E_A  = P_A * step_s
        else:
            P_A, E_A = row["avg_total_W"], E_meas

        # --- Model C (per-phase mean) ---
        P_C = model_c.get(phase, {}).get("mean_W", 0.0)
        E_C = P_C * step_s

        # Drain in Amperes (I = P / V)
        I_meas = E_meas / step_s / drone_voltage
        I_A    = E_A    / step_s / drone_voltage
        I_C    = E_C    / step_s / drone_voltage

        battery_meas -= E_meas
        battery_A    -= E_A
        battery_C    -= E_C

        records.append({
            "step":          int(row["step"]),
            "phase":         phase,
            "cpu_freq_mhz":  freq,
            "ops":           int(ops),

            "P_measured_W":  round(row["avg_total_W"], 4),
            "P_model_A_W":   round(P_A,  4),
            "P_model_C_W":   round(P_C,  4),

            "E_measured_J":  round(E_meas, 4),
            "E_model_A_J":   round(E_A,   4),
            "E_model_C_J":   round(E_C,   4),

            "I_measured_A":  round(I_meas, 5),
            "I_model_A_A":   round(I_A,   5),
            "I_model_C_A":   round(I_C,   5),

            "bat_meas_pct":  round(battery_meas / batt_0 * 100, 3),
            "bat_A_pct":     round(battery_A    / batt_0 * 100, 3),
            "bat_C_pct":     round(battery_C    / batt_0 * 100, 3),
        })

    return pd.DataFrame(records)


# ──────────────────────────────────────────────────────────────────────────────
# Plotting
# ──────────────────────────────────────────────────────────────────────────────

PHASE_COLORS = {
    "idle":    "#4C72B0",
    "partial": "#DD8452",
    "full":    "#55A868",
    "return":  "#C44E52",
}


def plot_results(
    raw:    pd.DataFrame,
    ns3_df: pd.DataFrame,
    output_prefix: str,
) -> None:
    """Generate all diagnostic and result plots."""

    fig = plt.figure(figsize=(18, 12))
    fig.suptitle("DroneOPERA – RPi5 Compute Energy Model", fontsize=14, fontweight="bold")
    gs  = gridspec.GridSpec(3, 3, figure=fig, hspace=0.45, wspace=0.35)

    # ── 1. Raw power over steps (coloured by phase) ──────────────────────────
    ax1 = fig.add_subplot(gs[0, :2])
    for phase, colour in PHASE_COLORS.items():
        mask = raw["phase"] == phase
        ax1.scatter(raw.loc[mask, "step"], raw.loc[mask, "avg_total_W"],
                    color=colour, label=phase, s=18, alpha=0.8)
    ax1.set_xlabel("NS3 Step")
    ax1.set_ylabel("Power (W)")
    ax1.set_title("Measured Board Power per NS3 Step")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, alpha=0.3)

    # ── 2. Power vs CPU frequency ─────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 2])
    for phase, colour in PHASE_COLORS.items():
        mask = raw["phase"] == phase
        ax2.scatter(raw.loc[mask, "cpu_freq_mhz"], raw.loc[mask, "avg_total_W"],
                    color=colour, label=phase, s=18, alpha=0.7)
    freq_range = np.linspace(raw["cpu_freq_mhz"].min(), raw["cpu_freq_mhz"].max(), 200)
    ax2.set_xlabel("CPU Frequency (MHz)")
    ax2.set_ylabel("Power (W)")
    ax2.set_title("Power vs Frequency")
    ax2.legend(fontsize=7)
    ax2.grid(True, alpha=0.3)

    # ── 3. Battery depletion – measured vs Model A vs Model C ────────────────
    ax3 = fig.add_subplot(gs[1, :])
    ax3.plot(ns3_df["step"], ns3_df["bat_meas_pct"], label="Measured",  linewidth=1.8)
    ax3.plot(ns3_df["step"], ns3_df["bat_A_pct"],    label="Model A (power law)",
             linewidth=1.5, linestyle="--")
    ax3.plot(ns3_df["step"], ns3_df["bat_C_pct"],    label="Model C (phase mean)",
             linewidth=1.5, linestyle=":")
    # Shade phases
    prev_phase = None
    start      = 0
    for _, row in ns3_df.iterrows():
        if row["phase"] != prev_phase:
            if prev_phase is not None:
                ax3.axvspan(start, row["step"] - 1,
                            color=PHASE_COLORS.get(prev_phase, "grey"), alpha=0.07)
            prev_phase = row["phase"]
            start      = row["step"]
    ax3.axvspan(start, ns3_df["step"].max(),
                color=PHASE_COLORS.get(prev_phase, "grey"), alpha=0.07)
    ax3.set_xlabel("NS3 Step")
    ax3.set_ylabel("Battery (%)")
    ax3.set_title("Battery Depletion over Simulation Steps  (compute load only)")
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.set_ylim(0, 105)

    # ── 4. Measured vs Model A power ─────────────────────────────────────────
    ax4 = fig.add_subplot(gs[2, 0])
    ax4.scatter(ns3_df["P_measured_W"], ns3_df["P_model_A_W"], s=14, alpha=0.6)
    lo = min(ns3_df["P_measured_W"].min(), ns3_df["P_model_A_W"].min()) * 0.95
    hi = max(ns3_df["P_measured_W"].max(), ns3_df["P_model_A_W"].max()) * 1.05
    ax4.plot([lo, hi], [lo, hi], "r--", linewidth=1, label="perfect fit")
    ax4.set_xlabel("Measured (W)")
    ax4.set_ylabel("Model A predicted (W)")
    ax4.set_title("Model A: Predicted vs Measured")
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    # ── 5. Drain current per step ─────────────────────────────────────────────
    ax5 = fig.add_subplot(gs[2, 1])
    ax5.plot(ns3_df["step"], ns3_df["I_measured_A"], label="Measured", linewidth=1.5)
    ax5.plot(ns3_df["step"], ns3_df["I_model_A_A"],  label="Model A",  linewidth=1.3, linestyle="--")
    ax5.set_xlabel("NS3 Step")
    ax5.set_ylabel("I_compute (A)")
    ax5.set_title("Compute Current per NS3 Step")
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    # ── 6. CPU temperature ────────────────────────────────────────────────────
    ax6 = fig.add_subplot(gs[2, 2])
    ax6.plot(raw["step"], raw["cpu_temp_c"], color="tomato", linewidth=1.5)
    ax6.set_xlabel("NS3 Step")
    ax6.set_ylabel("Temperature (°C)")
    ax6.set_title("CPU Temperature over Time")
    ax6.grid(True, alpha=0.3)

    out_path = f"{output_prefix}_analysis.png"
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"\nPlot saved → {out_path}")
    plt.show()


# ──────────────────────────────────────────────────────────────────────────────
# Coefficient export
# ──────────────────────────────────────────────────────────────────────────────

def export_coefficients(
    model_a:     dict,
    model_b:     dict,
    model_c:     dict,
    drone_cfg:   dict,
    step_s:      float,
    output_json: str,
) -> None:
    """
    Save fitted coefficients to JSON.

    This JSON can be:
      - Read by the C++ simulation to replace the empirical calcCompPower()
      - Injected into scenario.json under a new "compute_model" key
      - Used by plot.py for overlaid plots
    """
    export = {
        "description": "RPi5-fitted compute energy model for DroneOPERA",
        "ns3_step_s":  step_s,
        "drone_voltage_V": drone_cfg.get("voltage", 1.3),
        "model_A_power_law": model_a,   # P = alpha * f^beta * (ops/ops_ref)
        "model_B_polynomial": model_b,  # P = sum(coeffs[i] * f^i)
        "model_C_phase_mean": model_c,  # {phase: {mean_W, std_W}}
        "how_to_use_in_ns3": {
            "desc":    "For each DroneLogic step, compute I_compute_A = P_W / drone_voltage_V",
            "example": "I = model_A['alpha'] * pow(cpu_freq_mhz, model_A['beta']) * (ops / model_A['ops_ref']) / drone_voltage_V",
            "then":    "battery->SetCurrentA(I_mobility + I_compute + I_hw + I_degradation)",
        },
        "drone_params_used": {
            k: drone_cfg.get(k)
            for k in ["cpuFreq", "voltage", "switchCapacitance",
                       "cpuCyclePerOperation", "operationPerData",
                       "numbTrainDataSet", "numLocalIter"]
        },
    }

    with open(output_json, "w") as f:
        json.dump(export, f, indent=2)

    print(f"Coefficients exported → {output_json}")


# ──────────────────────────────────────────────────────────────────────────────
# Synthetic data generator  (used when no RPi5 CSV is available yet)
# ──────────────────────────────────────────────────────────────────────────────

def generate_synthetic_data(
    num_steps:     int,
    ops_per_step:  float,
    step_s:        float,
    cpu_freq_mhz:  float,
    noise_std:     float = 0.3,
) -> pd.DataFrame:
    """
    Generate plausible synthetic profiler data so you can run this script
    before you have the real RPi5 measurements.

    The synthetic power model:
        P_idle    ≈  3.5 W   (RPi5 idle, passive cooling)
        P_full    ≈  8.5 W   (RPi5 under full CPU load)
        noise     ~  N(0, noise_std)

    Adjust these to match your board.
    """
    rng    = np.random.default_rng(42)
    phases = build_phases_list(num_steps)

    P_BASE  = 3.5   # idle board power (W)
    P_DELTA = 5.0   # extra power at full load (W)
    T_START = 42.0  # starting temp (°C)

    rows = []
    temp = T_START
    for i, phase in enumerate(phases):
        frac   = PHASE_OPS_FRACTION[phase]
        ops    = int(ops_per_step * frac)

        # Simple thermal model
        temp += 0.08 * frac - 0.03 * max(0, temp - T_START)
        temp  = max(T_START, min(temp, 80.0))

        # Synthetic power
        P = P_BASE + P_DELTA * frac + rng.normal(0, noise_std)
        P = max(P_BASE * 0.9, P)

        rows.append({
            "step":         i,
            "timestamp":    f"2026-02-18T00:00:{i:02d}",
            "phase":        phase,
            "cpu_freq_mhz": cpu_freq_mhz + rng.normal(0, 5),
            "cpu_temp_c":   round(temp, 1),
            "ops_executed": ops,
            "avg_total_W":  round(P, 4),
            "avg_core_W":   round(P * 0.45, 4),
            "energy_step_J": round(P * step_s, 4),
            "drain_A":      round(P * step_s / step_s / 11.1, 5),
            "battery_pct":  0.0,
        })

    return pd.DataFrame(rows)


def build_phases_list(num_steps: int) -> list[str]:
    q = num_steps // 4
    return (
        ["idle"]    * q +
        ["partial"] * q +
        ["full"]    * q +
        ["return"]  * (num_steps - 3 * q)
    )


# ──────────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Fit compute-energy model from RPi5 data and map to NS3 steps"
    )
    p.add_argument("--csv",      default="rpi5_profile.csv",
                   help="Input CSV from rpi5_profiler.py (default: rpi5_profile.csv)")
    p.add_argument("--config",   default="../scenario/scenario.json",
                   help="NS3 scenario.json for drone parameters")
    p.add_argument("--step",     type=float, default=1.0,
                   help="NS3 pktInterval in seconds (default: 1.0)")
    p.add_argument("--voltage",  type=float, default=11.1,
                   help="Drone LiPo pack voltage (default: 11.1 V)")
    p.add_argument("--battery",  type=float, default=200.0,
                   help="Battery capacity in Wh (default: 200.0)")
    p.add_argument("--output",   default="compute_model",
                   help="Output prefix for plots and JSON (default: compute_model)")
    p.add_argument("--synthetic",action="store_true",
                   help="Generate synthetic data instead of loading CSV (useful before RPi5 run)")
    p.add_argument("--steps",    type=int, default=200,
                   help="Steps for synthetic generation (default: 200)")
    p.add_argument("--no-plot",  action="store_true", help="Skip plotting")
    return p.parse_args()


def main():
    args = parse_args()

    # ── Load drone config ───────────────────────────────────────────────────
    drone_cfg = {}
    if Path(args.config).exists():
        with open(args.config) as f:
            drone_cfg = json.load(f)["Drones"][0]
        print(f"Loaded drone config: {args.config}")
    else:
        print(f"[WARN] {args.config} not found – using CLI defaults")

    ops_per_step = (
        drone_cfg.get("cpuCyclePerOperation", 3.0)
        * drone_cfg.get("operationPerData", 20000.0)
        * drone_cfg.get("numbTrainDataSet", 60.0)
        * drone_cfg.get("numLocalIter", 10000.0)
    )
    cpu_freq_mhz   = drone_cfg.get("cpuFreq", 1.5) * 1000.0
    drone_voltage  = drone_cfg.get("voltage", args.voltage)
    battery_wh     = drone_cfg.get("energy",  args.battery)

    print(f"  ops/step   = {ops_per_step:.3e}")
    print(f"  cpu freq   = {cpu_freq_mhz:.0f} MHz")
    print(f"  pack volt  = {drone_voltage} V")
    print(f"  battery    = {battery_wh} Wh")

    # ── Theoretical baseline ────────────────────────────────────────────────
    P_theory = theoretical_compute_power(
        switch_cap = drone_cfg.get("switchCapacitance",    8e-11),
        voltage    = drone_cfg.get("voltage",              1.3),
        cyc_per_op = drone_cfg.get("cpuCyclePerOperation", 3.0),
        ops_data   = drone_cfg.get("operationPerData",     20000.0),
        n_data     = drone_cfg.get("numbTrainDataSet",     60.0),
        iters      = drone_cfg.get("numLocalIter",         10000.0),
    )
    print(f"\n[Theoretical calcCompPower] = {P_theory:.6f} W")
    print(f"  → This is the CPU-dynamic power only; real board power is higher")
    print(f"    (add idle board power ~3.5 W for RPi5)")

    # ── Load or generate data ───────────────────────────────────────────────
    if args.synthetic:
        print(f"\n[Synthetic mode] Generating {args.steps} steps of synthetic RPi5 data …")
        raw = generate_synthetic_data(
            num_steps    = args.steps,
            ops_per_step = ops_per_step,
            step_s       = args.step,
            cpu_freq_mhz = cpu_freq_mhz,
        )
        raw.to_csv(f"{args.output}_synthetic.csv", index=False)
        print(f"Synthetic CSV written → {args.output}_synthetic.csv")
    else:
        csv_path = Path(args.csv)
        if not csv_path.exists():
            print(f"[ERROR] CSV not found: {csv_path}")
            print("  Run rpi5_profiler.py first, or use --synthetic for a demo run.")
            sys.exit(1)
        raw = pd.read_csv(csv_path)
        print(f"Loaded {len(raw)} rows from {csv_path}")

    # ── Fit models ──────────────────────────────────────────────────────────
    model_a = fit_power_law(raw, ops_ref=ops_per_step)
    model_b = fit_polynomial(raw, degree=2)
    model_c = fit_phase_means(raw)

    # ── Map to NS3 steps ────────────────────────────────────────────────────
    ns3_df = map_to_ns3_steps(
        df            = raw,
        model_a       = model_a if model_a else {"alpha": 0, "beta": 1, "ops_ref": ops_per_step},
        model_c       = model_c,
        ops_per_step  = ops_per_step,
        step_s        = args.step,
        drone_voltage = drone_voltage,
        battery_wh    = battery_wh,
    )

    ns3_csv = f"{args.output}_ns3_steps.csv"
    ns3_df.to_csv(ns3_csv, index=False)
    print(f"\nNS3 step mapping → {ns3_csv}")

    # Summary table
    print("\n── NS3 step mapping preview (first 10 rows) ──────────────────────────")
    print(ns3_df[["step", "phase", "P_measured_W", "P_model_A_W",
                  "I_model_A_A", "bat_meas_pct"]].head(10).to_string(index=False))

    # ── Export coefficients ─────────────────────────────────────────────────
    export_coefficients(
        model_a     = model_a,
        model_b     = model_b,
        model_c     = model_c,
        drone_cfg   = drone_cfg,
        step_s      = args.step,
        output_json = f"{args.output}_coefficients.json",
    )

    # ── Plot ────────────────────────────────────────────────────────────────
    if not args.no_plot:
        plot_results(raw, ns3_df, output_prefix=args.output)


if __name__ == "__main__":
    main()
