#!/usr/bin/env python3
"""Plot battery usage (cumulative energy) over time from flights.csv."""

from __future__ import annotations

import argparse
import csv
import statistics
from bisect import bisect_right
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.size": 16,
    "axes.titlesize": 20,
    "axes.labelsize": 18,
    "legend.fontsize": 14,
    "xtick.labelsize": 14,
    "ytick.labelsize": 14,
})


TIME_CANDIDATES = [
    "time",
    "timestamp",
    "t",
    "time_s",
    "time_sec",
    "time_seconds",
    "sec",
    "secs",
    "ros_time",
    "time_ms",
    "time_us",
]

CURRENT_KEYWORDS = ["current", "battery_current", "bat_current", "current_a"]
VOLTAGE_KEYWORDS = ["voltage", "battery_voltage", "bat_voltage", "voltage_v"]


def _pick_column(headers: List[str], keywords: List[str]) -> Optional[str]:
    headers_l = [h.lower() for h in headers]
    for key in keywords:
        key_l = key.lower()
        for idx, h in enumerate(headers_l):
            if key_l == h:
                return headers[idx]
    # Fallback: substring match
    for key in keywords:
        key_l = key.lower()
        for idx, h in enumerate(headers_l):
            if key_l in h:
                return headers[idx]
    return None


def _infer_time_unit(median_dt: float) -> str:
    # Heuristic based on typical ROS/CSV logs.
    if median_dt > 1e4:
        return "us"
    if median_dt > 10:
        return "ms"
    return "s"


def _convert_time(value: float, unit: str) -> float:
    if unit == "s":
        return value
    if unit == "ms":
        return value * 1e-3
    if unit == "us":
        return value * 1e-6
    raise ValueError(f"Unsupported time unit: {unit}")


def _read_rows(
    csv_path: Path,
    flight_id: Optional[int],
    time_col: Optional[str],
    current_col: str,
    voltage_col: str,
) -> Tuple[List[float], List[float], List[float]]:
    times: List[float] = []
    currents: List[float] = []
    voltages: List[float] = []

    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if flight_id is not None:
                try:
                    if int(float(row.get("flight", ""))) != flight_id:
                        continue
                except Exception:
                    continue
            try:
                v = float(row[voltage_col])
                i = float(row[current_col])
            except Exception:
                continue
            if time_col:
                try:
                    t = float(row[time_col])
                except Exception:
                    continue
                times.append(t)
            currents.append(i)
            voltages.append(v)

    return times, currents, voltages


def _read_parameters(params_path: Path) -> dict:
    if not params_path.exists():
        return {}
    flights: dict = {}
    with params_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                flight_id = int(float(row.get("flight", "")))
            except Exception:
                continue
            flights[flight_id] = row
    return flights


def _compute_durations(csv_path: Path, time_col: str) -> Tuple[dict, float]:
    mins: dict = {}
    maxs: dict = {}
    times: List[float] = []

    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                flight_id = int(float(row.get("flight", "")))
                t = float(row.get(time_col, ""))
            except Exception:
                continue
            times.append(t)
            mins[flight_id] = t if flight_id not in mins else min(mins[flight_id], t)
            maxs[flight_id] = t if flight_id not in maxs else max(maxs[flight_id], t)

    dts = [times[i + 1] - times[i] for i in range(len(times) - 1)]
    dts = [dt for dt in dts if dt > 0]
    median_dt = statistics.median(dts) if dts else 0.0
    return {k: maxs[k] - mins[k] for k in mins}, median_dt


def _infer_time_unit_from_csv(csv_path: Path, time_col: str, max_dts: int = 10000) -> str:
    dts: List[float] = []
    last_time: Optional[float] = None
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = float(row.get(time_col, ""))
            except Exception:
                continue
            if last_time is not None:
                dt = t - last_time
                if dt > 0:
                    dts.append(dt)
                    if len(dts) >= max_dts:
                        break
            last_time = t
    median_dt = statistics.median(dts) if dts else 0.0
    return _infer_time_unit(median_dt)


def _compute_energy_per_flight(
    csv_path: Path,
    time_col: Optional[str],
    current_col: str,
    voltage_col: str,
    time_unit: Optional[str],
    dt_s: Optional[float],
) -> Tuple[dict, dict]:
    energy_j: dict = {}
    last_time: dict = {}
    min_time: dict = {}
    max_time: dict = {}

    if time_col and not time_unit:
        time_unit = _infer_time_unit_from_csv(csv_path, time_col)

    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                flight_id = int(float(row.get("flight", "")))
                v = float(row[voltage_col])
                i = float(row[current_col])
            except Exception:
                continue

            if time_col:
                try:
                    t_raw = float(row.get(time_col, ""))
                except Exception:
                    continue
                t = _convert_time(t_raw, time_unit or "s")
                if flight_id not in min_time:
                    min_time[flight_id] = t
                    max_time[flight_id] = t
                else:
                    min_time[flight_id] = min(min_time[flight_id], t)
                    max_time[flight_id] = max(max_time[flight_id], t)

                prev = last_time.get(flight_id)
                delta = 0.0 if prev is None else max(t - prev, 0.0)
                last_time[flight_id] = t
            else:
                if dt_s is None:
                    raise ValueError("No time column found; provide --dt to set a fixed sample interval.")
                delta = dt_s

            power_w = v * abs(i)
            energy_j[flight_id] = energy_j.get(flight_id, 0.0) + power_w * delta

    durations_s = {fid: max_time[fid] - min_time[fid] for fid in min_time}
    return energy_j, durations_s


def _load_flight_samples(
    csv_path: Path,
    time_col: Optional[str],
    current_col: str,
    voltage_col: str,
    time_unit: Optional[str],
    dt_s: Optional[float],
) -> dict:
    flights: dict = {}

    if time_col and not time_unit:
        time_unit = _infer_time_unit_from_csv(csv_path, time_col)

    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                flight_id = int(float(row.get("flight", "")))
                v = float(row[voltage_col])
                i = float(row[current_col])
            except Exception:
                continue

            if time_col:
                try:
                    t_raw = float(row.get(time_col, ""))
                except Exception:
                    continue
                t = _convert_time(t_raw, time_unit or "s")
            else:
                if dt_s is None:
                    raise ValueError("No time column found; provide --dt to set a fixed sample interval.")
                t = None

            flights.setdefault(flight_id, []).append((t, v, i))

    if not time_col:
        for flight_id, samples in flights.items():
            flights[flight_id] = [(idx * dt_s, v, i) for idx, (_, v, i) in enumerate(samples)]

    return flights


def _energy_series_from_samples(samples: List[Tuple[float, float, float]]) -> Tuple[List[float], List[float]]:
    samples = sorted(samples, key=lambda x: x[0])
    times = [s[0] for s in samples]
    energies: List[float] = []
    cumulative = 0.0
    for idx, (t, v, i) in enumerate(samples):
        if idx == 0:
            delta = 0.0
        else:
            delta = max(t - times[idx - 1], 0.0)
        cumulative += v * abs(i) * delta
        energies.append(cumulative)
    return times, energies


def _interp_energy(times: List[float], energies: List[float], t: float) -> Optional[float]:
    if not times:
        return None
    if t < times[0] or t > times[-1]:
        return None
    idx = bisect_right(times, t)
    if idx == 0:
        return energies[0]
    if idx >= len(times):
        return energies[-1]
    t0, t1 = times[idx - 1], times[idx]
    e0, e1 = energies[idx - 1], energies[idx]
    if t1 == t0:
        return e0
    ratio = (t - t0) / (t1 - t0)
    return e0 + ratio * (e1 - e0)


def _energy_from_samples(
    times_s: List[float],
    currents: List[float],
    voltages: List[float],
    dt_s: Optional[float],
) -> Tuple[List[float], List[float]]:
    n = min(len(currents), len(voltages))
    currents = currents[:n]
    voltages = voltages[:n]

    if times_s:
        # Sort by time, then align samples.
        pairs = sorted(zip(times_s, currents, voltages), key=lambda x: x[0])
        times_s = [p[0] for p in pairs]
        currents = [p[1] for p in pairs]
        voltages = [p[2] for p in pairs]
    else:
        if dt_s is None:
            raise ValueError("No time column found; provide --dt to set a fixed sample interval.")
        times_s = [i * dt_s for i in range(n)]

    energy_j: List[float] = []
    cumulative = 0.0
    for idx in range(n):
        if idx == 0:
            delta = 0.0
        else:
            delta = times_s[idx] - times_s[idx - 1]
            if delta < 0:
                delta = 0.0
        power_w = voltages[idx] * abs(currents[idx])
        cumulative += power_w * delta
        energy_j.append(cumulative)

    return times_s, energy_j


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot battery usage (J) over time from flights.csv")
    parser.add_argument(
        "--csv",
        default=str(Path("12683453") / "flights.csv"),
        help="Path to flights.csv",
    )
    parser.add_argument("--flight", type=int, default=None, help="Filter by flight id")
    parser.add_argument("--time-col", default=None, help="Override time column name")
    parser.add_argument("--current-col", default=None, help="Override current column name")
    parser.add_argument("--voltage-col", default=None, help="Override voltage column name")
    parser.add_argument("--time-unit", choices=["s", "ms", "us"], default=None, help="Time unit")
    parser.add_argument("--dt", type=float, default=None, help="Fixed dt (seconds) if no time column")
    parser.add_argument(
        "--out",
        default=None,
        help="Output PNG path (defaults to 12683453/battery_usage_f<id>.png)",
    )
    parser.add_argument(
        "--print-specs",
        action="store_true",
        help="Print flight specifications and duration to the terminal",
    )
    parser.add_argument(
        "--plot-all",
        action="store_true",
        help="Plot all flights together with average curve",
    )

    args = parser.parse_args()
    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    with csv_path.open("r", newline="") as f:
        reader = csv.reader(f)
        headers = next(reader)

    time_col = args.time_col or _pick_column(headers, TIME_CANDIDATES)
    current_col = args.current_col or _pick_column(headers, CURRENT_KEYWORDS)
    voltage_col = args.voltage_col or _pick_column(headers, VOLTAGE_KEYWORDS)

    if not current_col or not voltage_col:
        raise SystemExit(
            "Could not detect current/voltage columns. Use --current-col and --voltage-col."
        )

    if args.flight is None and not args.plot_all:
        energy_by_flight, durations_s = _compute_energy_per_flight(
            csv_path,
            time_col,
            current_col,
            voltage_col,
            args.time_unit,
            args.dt,
        )
        if not energy_by_flight:
            raise SystemExit("No data rows matched the flight filter.")

        summary_path = Path("12683453") / "battery_usage_per_flight.csv"
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        with summary_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["flight", "energy_j", "duration_s"])
            for fid in sorted(energy_by_flight.keys()):
                writer.writerow([fid, f"{energy_by_flight[fid]:.6f}", f"{durations_s.get(fid, 0.0):.6f}"])

        print("Per-flight energy (J):")
        for fid in sorted(energy_by_flight.keys()):
            dur = durations_s.get(fid)
            dur_str = f"{dur:.2f}s" if dur is not None else "n/a"
            print(f"flight={fid} energy_j={energy_by_flight[fid]:.3f} duration={dur_str}")
        print(f"Wrote: {summary_path}")
        return

    if args.plot_all:
        flights = _load_flight_samples(
            csv_path,
            time_col,
            current_col,
            voltage_col,
            args.time_unit,
            args.dt,
        )
        if not flights:
            raise SystemExit("No data rows found.")

        series = {}
        total_energy = []
        durations = []
        for fid, samples in flights.items():
            times, energies = _energy_series_from_samples(samples)
            if not times:
                continue
            series[fid] = (times, energies)
            total_energy.append(energies[-1])
            durations.append(times[-1] - times[0])

        if not series:
            raise SystemExit("No usable flight series to plot.")

        max_duration = max(durations)
        grid_n = 400
        grid = [max_duration * i / (grid_n - 1) for i in range(grid_n)]
        avg_curve: List[float] = []
        for t in grid:
            values = []
            for times, energies in series.values():
                v = _interp_energy(times, energies, t)
                if v is not None:
                    values.append(v)
            avg_curve.append(statistics.mean(values) if values else float("nan"))

        if args.out:
            out_path = Path(args.out)
        else:
            out_path = Path("12683453") / "battery_usage_all_flights.png"

        out_path.parent.mkdir(parents=True, exist_ok=True)

        params_path = csv_path.parent / "parameters.csv"
        params = _read_parameters(params_path)
        speeds = []
        payloads = []
        altitudes = []
        weights = []
        weight_keys = ["weight", "overall_weight", "total_weight"]
        for fid in series.keys():
            row = params.get(fid, {})
            try:
                speed = float(row.get("speed", ""))
                speeds.append(speed)
            except Exception:
                pass
            try:
                payload = float(row.get("payload", ""))
                payloads.append(payload)
            except Exception:
                pass
            try:
                altitude = float(row.get("altitude", ""))
                altitudes.append(altitude)
            except Exception:
                pass
            for key in weight_keys:
                try:
                    weight = float(row.get(key, ""))
                    weights.append(weight)
                    break
                except Exception:
                    continue

        mean_speed = statistics.mean(speeds) if speeds else None
        mean_payload = statistics.mean(payloads) if payloads else None
        mean_altitude = statistics.mean(altitudes) if altitudes else None
        mean_weight = statistics.mean(weights) if weights else None

        plt.figure(figsize=(10, 6))
        for fid, (times, energies) in series.items():
            plt.plot(times, energies, linewidth=0.8, alpha=0.25)
        avg_label = "Average"
        label_parts = []
        if mean_speed is not None:
            label_parts.append(f"speed={mean_speed:.2f} m/s")
        if mean_payload is not None:
            label_parts.append(f"payload={mean_payload:.2f} g")
        if mean_weight is not None:
            label_parts.append(f"weight={mean_weight:.2f}")
        if mean_altitude is not None:
            label_parts.append(f"altitude={mean_altitude:.2f} m")
        if label_parts:
            avg_label += " (mean " + ", ".join(label_parts) + ")"
        plt.plot(grid, avg_curve, linewidth=2.2, color="black", label=avg_label)
        plt.xlabel("Time (s)")
        plt.ylabel("Cumulative energy (J)")
        plt.title("Battery usage per flight (with average)")
        plt.grid(True, alpha=0.3)
        plt.legend(loc="upper left")

        mean_e = statistics.mean(total_energy)
        med_e = statistics.median(total_energy)
        std_e = statistics.pstdev(total_energy) if len(total_energy) > 1 else 0.0
        mean_d = statistics.mean(durations)
        med_d = statistics.median(durations)
        std_d = statistics.pstdev(durations) if len(durations) > 1 else 0.0
        if speeds:
            mean_s = statistics.mean(speeds)
            med_s = statistics.median(speeds)
            std_s = statistics.pstdev(speeds) if len(speeds) > 1 else 0.0
        else:
            mean_s = med_s = std_s = None
        if payloads:
            mean_p = statistics.mean(payloads)
            med_p = statistics.median(payloads)
            std_p = statistics.pstdev(payloads) if len(payloads) > 1 else 0.0
        else:
            mean_p = med_p = std_p = None
        if altitudes:
            mean_a = statistics.mean(altitudes)
            med_a = statistics.median(altitudes)
            std_a = statistics.pstdev(altitudes) if len(altitudes) > 1 else 0.0
        else:
            mean_a = med_a = std_a = None
        if weights:
            mean_w = statistics.mean(weights)
            med_w = statistics.median(weights)
            std_w = statistics.pstdev(weights) if len(weights) > 1 else 0.0
        else:
            mean_w = med_w = std_w = None
        stats_text = (
            f"Flights: {len(total_energy)}\n"
            f"Energy mean/med/std (J): {mean_e:.1f} / {med_e:.1f} / {std_e:.1f}\n"
            f"Duration mean/med/std (s): {mean_d:.1f} / {med_d:.1f} / {std_d:.1f}"
        )
        if mean_s is not None:
            stats_text += (
                f"\nSpeed mean/med/std (m/s): {mean_s:.2f} / {med_s:.2f} / {std_s:.2f}"
            )
        if mean_p is not None:
            stats_text += (
                f"\nPayload mean/med/std (g): {mean_p:.2f} / {med_p:.2f} / {std_p:.2f}"
            )
        if mean_w is not None:
            stats_text += (
                f"\nWeight mean/med/std: {mean_w:.2f} / {med_w:.2f} / {std_w:.2f}"
            )
        if mean_a is not None:
            stats_text += (
                f"\nAltitude mean/med/std (m): {mean_a:.2f} / {med_a:.2f} / {std_a:.2f}"
            )
        plt.gca().text(
            0.98,
            0.02,
            stats_text,
            transform=plt.gca().transAxes,
            ha="right",
            va="bottom",
            fontsize=9,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.8},
        )

        plt.tight_layout()
        plt.savefig(out_path, dpi=160)

        print(f"Wrote: {out_path}")
        print(f"Flights plotted: {len(total_energy)}")
        return

    times_raw, currents, voltages = _read_rows(
        csv_path, args.flight, time_col, current_col, voltage_col
    )

    if not currents or not voltages:
        raise SystemExit("No data rows matched the requested flight filter.")

    times_s: List[float] = []
    if times_raw:
        dts = [times_raw[i + 1] - times_raw[i] for i in range(len(times_raw) - 1)]
        dts = [dt for dt in dts if dt > 0]
        median_dt = statistics.median(dts) if dts else 0.0
        time_unit = args.time_unit or _infer_time_unit(median_dt)
        times_s = [_convert_time(t, time_unit) for t in times_raw]
    else:
        time_unit = "s"

    times_s, energy_j = _energy_from_samples(times_s, currents, voltages, args.dt)

    if args.out:
        out_path = Path(args.out)
    else:
        out_path = Path("12683453") / f"battery_usage_f{args.flight}.png"

    out_path.parent.mkdir(parents=True, exist_ok=True)

    plt.figure(figsize=(9, 5))
    plt.plot(times_s, energy_j, linewidth=1.5)
    plt.xlabel("Time (s)")
    plt.ylabel("Cumulative energy (J)")
    title = "Battery usage (cumulative energy)"
    title += f" - flight {args.flight}"
    plt.title(title)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=160)

    print(f"Wrote: {out_path}")
    print(f"Samples: {len(energy_j)}")
    print(f"Energy end: {energy_j[-1]:.3f} J")

    if args.print_specs:
        params_path = csv_path.parent / "parameters.csv"
        params = _read_parameters(params_path)
        if time_col:
            durations_raw, median_dt_all = _compute_durations(csv_path, time_col)
            time_unit_all = args.time_unit or _infer_time_unit(median_dt_all)
            durations_s = {k: _convert_time(v, time_unit_all) for k, v in durations_raw.items()}
        else:
            durations_s = {}
            time_unit_all = "s"

        flight_ids: Iterable[int]
        if args.flight is not None:
            flight_ids = [args.flight]
        else:
            flight_ids = sorted(params.keys()) if params else []

        print("\nFlight specifications:")
        for flight_id in flight_ids:
            row = params.get(flight_id, {})
            duration = durations_s.get(flight_id)
            duration_str = f"{duration:.2f}s" if duration is not None else "n/a"
            speed = row.get("speed", "n/a")
            payload = row.get("payload", "n/a")
            altitude = row.get("altitude", "n/a")
            date = row.get("date", "n/a")
            local_time = row.get("local_time", "n/a")
            route = row.get("route", "n/a")
            print(
                f"flight={flight_id} speed={speed} payload={payload} altitude={altitude} "
                f"date={date} time={local_time} route={route} duration={duration_str}"
            )


if __name__ == "__main__":
    main()
