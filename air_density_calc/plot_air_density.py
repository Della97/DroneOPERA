#!/usr/bin/env python3
"""Plot air density (rho) from sea level to 120 meters using ISA troposphere."""

import math
from pathlib import Path

import matplotlib.pyplot as plt

# International Standard Atmosphere (troposphere)
T0_K = 288.15       # sea level standard temperature (K)
P0_PA = 101325.0    # sea level standard pressure (Pa)
L_K_PER_M = 0.0065  # temperature lapse rate (K/m)
G_M_S2 = 9.80665    # gravity (m/s^2)
R_J_KG_K = 287.05   # specific gas constant for dry air (J/(kg*K))


def air_density_isa(h_m: float) -> float:
    """Return air density at altitude h_m (m) using ISA troposphere model."""
    t = T0_K - L_K_PER_M * h_m
    if t <= 0:
        return float("nan")
    p = P0_PA * (t / T0_K) ** (G_M_S2 / (R_J_KG_K * L_K_PER_M))
    rho = p / (R_J_KG_K * t)
    return rho


def main() -> None:
    h_values = list(range(0, 121))
    rho_values = [air_density_isa(h) for h in h_values]

    plt.figure(figsize=(8, 5))
    plt.plot(h_values, rho_values, linewidth=2.0)
    plt.xlabel("Altitude (m)")
    plt.ylabel("Air density (kg/m^3)")
    plt.title("Air density vs altitude (ISA, 0-120 m)")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()

    out_path = Path("air_density_0_120m.png")
    plt.savefig(out_path, dpi=160)
    print(f"Wrote: {out_path}")


if __name__ == "__main__":
    main()
