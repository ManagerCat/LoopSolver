#!/usr/bin/env python3
"""
Series vs Parallel Loop Comparison Tool (series_vs_parallel.py)

Compares hydraulic and thermal metrics between 4_Motor_Series_Counterclockwise
and 4_Motor_Parallel loop configurations under identical hardware conditions.

Metrics compared:
- Pressure drop the radiator sees (dP_rad) and total loop dP
- LPM at motor and at radiator
- Radiator thermal resistance (R_rad)
- Per-motor convective sleeve thermal resistance (sleeve_r) for FL, FR, RL, RR
- Water temperatures (radiator out/in, motor in/out)
- Motor core temperatures

Usage:
    python series_vs_parallel.py
    python series_vs_parallel.py --rad pc -L 2 --pump ta60
"""

import argparse
import math
from pathlib import Path
import sys
import yaml

from architectures import ARCHITECTURES, eval_4_motor_parallel, eval_4_motor_series_ccw
from orchestrator import compute_loop_mass_and_power
from hydraulics import (
    COMPONENTS,
    PUMPS,
    get_radiator_impedance,
    solve_hydraulics,
    solve_hydraulics_parallel,
)
from motor import solve_motor
from utils.cp import calculate_cp_water
from utils.v import getv

# ==============================================================================
# CONFIGURABLE NOMINAL DEFAULTS
# ==============================================================================
DEFAULT_RAD_TYPE = "kart"          # "kart" or "pc"
DEFAULT_L = 10.0                   # Core length/height in inches (or fan count for PC)
DEFAULT_PUMP = "intg3-570"         # Pump model from pumps.yaml
DEFAULT_TUBING = "thick"           # "thick" or "thin"
DEFAULT_FAN = "No_Fan_Passive"     # Fan model from fans.yaml (uses catalog max_cfm & aero sweep)
# ==============================================================================

_BASE_DIR = Path(__file__).parent

with open(_BASE_DIR / "loops.yaml") as f:
    LOOP_CONFIGS = yaml.safe_load(f)

with open(_BASE_DIR / "thermals.yaml") as f:
    THERMALS = yaml.safe_load(f)

with open(_BASE_DIR / "fans.yaml") as f:
    FANS = yaml.safe_load(f)

MOTOR_HEATS = THERMALS["motors"]["heat"]


def solve_detailed_series(hydraulics, L, rad_type, cfm):
    """
    Evaluates 4_Motor_Series_Counterclockwise and captures detailed per-motor
    inlet temps, outlet temps, sleeve resistances, and core temps.
    Flow order: RL -> RR -> FR -> FL -> Radiator.
    """
    eval_res = eval_4_motor_series_ccw(hydraulics, L, rad_type, cfm)
    lpm = hydraulics["lpm"]
    t_rad_out = eval_res["water_temps"]["radiator"]

    motor_order = ("RL", "RR", "FR", "FL")
    t_curr = t_rad_out
    motor_details = {}

    for m in motor_order:
        spec_heat = calculate_cp_water(t_curr)
        _, t_out, tc_t, sleeve_r = solve_motor(
            heat=MOTOR_HEATS[m], t_in=t_curr, lpm=lpm, spec_heat=spec_heat
        )
        motor_details[m] = {
            "t_in": t_curr,
            "t_out": t_out,
            "sleeve_r": sleeve_r,
            "motor_temp": tc_t,
        }
        t_curr = t_out

    # Radiator inlet is the outlet of the last motor (FL)
    t_rad_in = motor_details["FL"]["t_out"]

    return {
        "lpm_rad": lpm,
        "lpm_motor": lpm,
        "dp_sys": hydraulics["dp"],
        "radiator_r": eval_res["radiator_r"],
        "t_rad_out": t_rad_out,
        "t_rad_in": t_rad_in,
        "motors": motor_details,
    }


def solve_detailed_parallel(hydraulics, L, rad_type, cfm):
    """
    Evaluates 4_Motor_Parallel and captures detailed per-motor inlet temps,
    outlet temps, sleeve resistances, and core temps.
    Banks:
        Left:  FL -> RL
        Right: FR -> RR
    """
    eval_res = eval_4_motor_parallel(hydraulics, L, rad_type, cfm)
    lpm_rad = hydraulics["lpm_rad"]
    lpm_motor = hydraulics["lpm_motor"]
    t_rad_out = eval_res["water_temps"]["radiator"]

    banks = {
        "left": ("FL", "RL"),
        "right": ("FR", "RR"),
    }
    motor_details = {}

    for bank_name, (front, rear) in banks.items():
        # Front motor
        cp_f = calculate_cp_water(t_rad_out)
        _, t_out_f, tc_f, sleeve_r_f = solve_motor(
            heat=MOTOR_HEATS[front], t_in=t_rad_out, lpm=lpm_motor, spec_heat=cp_f
        )
        motor_details[front] = {
            "t_in": t_rad_out,
            "t_out": t_out_f,
            "sleeve_r": sleeve_r_f,
            "motor_temp": tc_f,
        }

        # Rear motor
        cp_r = calculate_cp_water(t_out_f)
        _, t_out_r, tc_r, sleeve_r_r = solve_motor(
            heat=MOTOR_HEATS[rear], t_in=t_out_f, lpm=lpm_motor, spec_heat=cp_r
        )
        motor_details[rear] = {
            "t_in": t_out_f,
            "t_out": t_out_r,
            "sleeve_r": sleeve_r_r,
            "motor_temp": tc_r,
        }

    t_rad_in = eval_res["water_temps"]["mix"]

    return {
        "lpm_rad": lpm_rad,
        "lpm_motor": lpm_motor,
        "dp_sys": hydraulics["dp"],
        "radiator_r": eval_res["radiator_r"],
        "t_rad_out": t_rad_out,
        "t_rad_in": t_rad_in,
        "motors": motor_details,
    }


def compute_rad_dp(rad_type, L, lpm):
    """Calculates pressure drop across the radiator core in psi at a given flow rate."""
    rad_key = rad_type.lower().strip()
    fan_scale = L if rad_key == "pc" else 2
    H = L if rad_key == "kart" else None
    a_rad, b_rad = get_radiator_impedance(rad_type=rad_key, fan_scale=fan_scale, H=H, num_radiators=1)
    return a_rad * (lpm ** 2) + b_rad * lpm


def format_row(metric, series_val, par_val, unit="", precision=3, higher_is_better=None):
    delta = par_val - series_val
    if abs(series_val) > 1e-9:
        pct = (delta / series_val) * 100.0
        pct_str = f"{pct:+.1f}%"
    else:
        pct_str = "N/A"

    s_str = f"{series_val:.{precision}f} {unit}".strip()
    p_str = f"{par_val:.{precision}f} {unit}".strip()
    d_str = f"{delta:+.{precision}f} {unit}".strip()

    return f"| {metric:<34} | {s_str:>16} | {p_str:>16} | {d_str:>16} | {pct_str:>10} |"


def main():
    parser = argparse.ArgumentParser(
        description="Compare Series vs Parallel cooling loop performance."
    )
    parser.add_argument(
        "--rad", choices=["kart", "pc"], default=DEFAULT_RAD_TYPE, help="Radiator type"
    )
    parser.add_argument(
        "-L", type=float, default=DEFAULT_L, help="Radiator geometry (in for kart, fans for pc)"
    )
    parser.add_argument(
        "--pump", default=DEFAULT_PUMP, help="Pump model from pumps.yaml"
    )
    parser.add_argument(
        "--tubing", choices=["thick", "thin"], default=DEFAULT_TUBING, help="Tubing type"
    )
    parser.add_argument(
        "--fan", default=DEFAULT_FAN, choices=list(FANS.keys()), help="Fan model from fans.yaml"
    )
    args = parser.parse_args()

    # Look up rated fan CFM and correlate radiator face velocity
    fan_cfg = FANS[args.fan]
    cfm = fan_cfg["max_cfm"]
    face_v = getv(cfm, args.rad)

    # Solve hydraulics
    series_cfg = LOOP_CONFIGS["4_Motor_Series_Counterclockwise"]
    parallel_cfg = LOOP_CONFIGS["4_Motor_Parallel"]

    h_series = solve_hydraulics(
        series_cfg,
        tubing_type=args.tubing,
        radiator_type=args.rad,
        L=args.L,
        pump=args.pump,
    )
    h_par = solve_hydraulics_parallel(
        parallel_cfg,
        tubing_type=args.tubing,
        radiator_type=args.rad,
        L=args.L,
        pump=args.pump,
    )

    # Radiator dP
    dp_rad_series = compute_rad_dp(args.rad, args.L, h_series["lpm"])
    dp_rad_par = compute_rad_dp(args.rad, args.L, h_par["lpm_rad"])

    # Detailed thermals
    det_series = solve_detailed_series(h_series, args.L, args.rad, cfm)
    det_par = solve_detailed_parallel(h_par, args.L, args.rad, cfm)

    # Print Configuration Summary
    print("=" * 105)
    print(" COOLING LOOP COMPARISON: SERIES (CCW) vs PARALLEL")
    print("=" * 105)
    print(f"  Radiator Type : {args.rad.upper():<14} | Core Parameter L: {args.L} ({'in' if args.rad=='kart' else 'fans'})")
    print(f"  Pump Model    : {args.pump:<14} | Tubing Type     : {args.tubing}")
    print(f"  Fan Model     : {args.fan:<14} | Rated CFM: {cfm:.1f} CFM  -->  Correlated Face Velocity: {face_v:.2f} m/s")
    print("=" * 105)

    header = f"| {'Metric':<34} | {'Series (CCW)':>16} | {'Parallel':>16} | {'Delta (Par-Ser)':>16} | {'% Change':>10} |"
    sep_line = "+" + "-" * 36 + "+" + "-" * 18 + "+" + "-" * 18 + "+" + "-" * 18 + "+" + "-" * 12 + "+"
    sec_div  = "| " + "-" * 101 + " |"

    print(sep_line)
    print(header)
    print(sep_line)

    # Section 1: HYDRAULICS
    print(f"| {'[HYDRAULICS: FLOW & PRESSURE]':<101} |")
    print(sep_line)
    print(format_row("Radiator Flow Rate (lpm_rad)", det_series["lpm_rad"], det_par["lpm_rad"], "LPM", 2))
    print(format_row("Motor Flow Rate (lpm_motor)", det_series["lpm_motor"], det_par["lpm_motor"], "LPM", 2))
    print(format_row("Radiator Pressure Drop (dP_rad)", dp_rad_series, dp_rad_par, "psi", 2))
    print(format_row("Total System Pressure Drop", det_series["dp_sys"], det_par["dp_sys"], "psi", 2))
    print(format_row("dP_rad / dP_sys Fraction", (dp_rad_series / det_series["dp_sys"]) * 100, (dp_rad_par / det_par["dp_sys"]) * 100, "%", 1))

    # Section 2: RADIATOR THERMALS
    print(sep_line)
    print(f"| {'[RADIATOR THERMAL PERFORMANCE]':<101} |")
    print(sep_line)
    print(format_row("Radiator Resistance (R_rad)", det_series["radiator_r"], det_par["radiator_r"], "K/W", 5))
    print(format_row("Radiator Outlet Temp (Cold Supply)", det_series["t_rad_out"], det_par["t_rad_out"], "°C", 2))
    print(format_row("Radiator Inlet Temp (Hot Return)", det_series["t_rad_in"], det_par["t_rad_in"], "°C", 2))
    dT_rad_series = det_series["t_rad_in"] - det_series["t_rad_out"]
    dT_rad_par = det_par["t_rad_in"] - det_par["t_rad_out"]
    print(format_row("Coolant Temp Drop Across Rad (dT)", dT_rad_series, dT_rad_par, "°C", 2))

    # Section 3: MOTOR CONVECTIVE THERMAL RESISTANCE
    print(sep_line)
    print(f"| {'[MOTOR CONVECTIVE THERMAL RESISTANCE (SLEEVE R)]':<101} |")
    print(sep_line)
    corners = ["FL", "FR", "RL", "RR"]
    for c in corners:
        s_r = det_series["motors"][c]["sleeve_r"]
        p_r = det_par["motors"][c]["sleeve_r"]
        heat = MOTOR_HEATS[c]
        print(format_row(f"{c} Sleeve R ({heat} W)", s_r, p_r, "K/W", 5))

    # Section 4: MOTOR WATER OUTLET TEMPERATURES
    print(sep_line)
    print(f"| {'[WATER OUTLET TEMPERATURES]':<101} |")
    print(sep_line)
    for c in corners:
        s_w = det_series["motors"][c]["t_out"]
        p_w = det_par["motors"][c]["t_out"]
        print(format_row(f"{c} Water Out", s_w, p_w, "°C", 2))

    # Section 5: MOTOR CORE TEMPERATURES
    print(sep_line)
    print(f"| {'[MOTOR CORE / WINDING TEMPERATURES]':<101} |")
    print(sep_line)
    for c in corners:
        s_t = det_series["motors"][c]["motor_temp"]
        p_t = det_par["motors"][c]["motor_temp"]
        print(format_row(f"{c} Core Temp", s_t, p_t, "°C", 2))
    max_s_t = max(det_series["motors"][c]["motor_temp"] for c in corners)
    max_p_t = max(det_par["motors"][c]["motor_temp"] for c in corners)
    print(format_row("Max Motor Core Temp (Worst Corner)", max_s_t, max_p_t, "°C", 2))

    # Section 6: WEIGHT & ELECTRICAL POWER BREAKDOWN
    series_mass, series_pwr, series_breakdown = compute_loop_mass_and_power(
        "4_Motor_Series_Counterclockwise", ARCHITECTURES["4_Motor_Series_Counterclockwise"],
        args.rad, args.L, args.pump, args.tubing, args.fan
    )
    par_mass, par_pwr, par_breakdown = compute_loop_mass_and_power(
        "4_Motor_Parallel", ARCHITECTURES["4_Motor_Parallel"],
        args.rad, args.L, args.pump, args.tubing, args.fan
    )

    print(sep_line)
    print(f"| {'[WEIGHT & ELECTRICAL POWER BREAKDOWN]':<101} |")
    print(sep_line)
    print(format_row("Radiator Mass", series_breakdown["radiator"], par_breakdown["radiator"], "lbs", 3))
    print(format_row("Pump Mass", series_breakdown["pump"], par_breakdown["pump"], "lbs", 3))
    print(format_row("Fan Mass", series_breakdown["fan"], par_breakdown["fan"], "lbs", 3))
    print(format_row("Tubing Mass", series_breakdown["tubing"], par_breakdown["tubing"], "lbs", 3))
    print(format_row("Fittings / Components Mass", series_breakdown["fittings"], par_breakdown["fittings"], "lbs", 3))
    print(format_row("Initial Vehicle Mass", series_breakdown["initial"], par_breakdown["initial"], "lbs", 3))
    print(format_row("Total Loop Mass", series_mass, par_mass, "lbs", 3))
    print(format_row("Total Electrical Power", series_pwr, par_pwr, "W", 1))

    print(sep_line)
    print()

    # Key Physical Insights
    print("KEY PHYSICAL TAKEAWAYS:")
    print("-" * 105)
    rad_dp_delta = dp_rad_par - dp_rad_series
    rad_dp_pct = (rad_dp_delta / dp_rad_series) * 100
    print(f"1. Radiator sees {det_par['lpm_rad'] - det_series['lpm_rad']:+.2f} LPM (+{(det_par['lpm_rad'] - det_series['lpm_rad'])/det_series['lpm_rad']*100:.1f}%) in Parallel.")
    print(f"   -> Consequently, Radiator dP jumps by {rad_dp_delta:+.2f} psi ({rad_dp_pct:+.1f}%), going from {dp_rad_series:.2f} psi ({dp_rad_series/det_series['dp_sys']*100:.1f}% of total) to {dp_rad_par:.2f} psi ({dp_rad_par/det_par['dp_sys']*100:.1f}% of total).")
    motor_lpm_delta = det_par["lpm_motor"] - det_series["lpm_motor"]
    motor_lpm_pct = (motor_lpm_delta / det_series["lpm_motor"]) * 100
    sleeve_r_delta = det_par["motors"]["FL"]["sleeve_r"] - det_series["motors"]["FL"]["sleeve_r"]
    sleeve_r_pct = (sleeve_r_delta / det_series["motors"]["FL"]["sleeve_r"]) * 100
    print(f"2. Each motor sees {motor_lpm_delta:+.2f} LPM ({motor_lpm_pct:+.1f}%) in Parallel (branch split).")
    print(f"   -> Slower coolant velocity inside the motor cooling jacket increases sleeve thermal resistance by {sleeve_r_delta:+.5f} K/W (+{sleeve_r_pct:.1f}%).")
    print(f"3. Water temperature rise across radiator is {dT_rad_par:.2f} °C in Parallel vs {dT_rad_series:.2f} °C in Series.")
    print("4. Architecture trade-off:")
    print("   - Series keeps motor sleeve R minimal by maintaining full loop flow (9.1+ LPM) through every motor jacket,")
    print("     but coolant progressively warms up through each successive motor (RL -> RR -> FR -> FL).")
    print("   - Parallel delivers cooler inlet water to the front motors (FL & FR get fresh radiator outlet water),")
    print("     but suffers from degraded sleeve heat transfer coefficient (higher sleeve R) due to halved branch flow.")
    mass_delta = par_mass - series_mass
    mass_pct = (mass_delta / series_mass) * 100
    print(f"5. Mass difference: Parallel is {mass_delta:+.3f} lbs ({mass_pct:+.1f}%) heavier than Series due to longer tubing routing ({LOOP_CONFIGS['4_Motor_Parallel']['tubing']} in vs {LOOP_CONFIGS['4_Motor_Series_Counterclockwise']['tubing']} in).")
    print("-" * 105)


if __name__ == "__main__":
    main()

