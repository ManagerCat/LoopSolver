#!/usr/bin/env python3
"""
Single Loop Maximum Heat Rejection Calculator (single.py)

Evaluates the Single cooling loop architecture for a given hardware configuration
(array of pumps in series, radiator type and L, fan type, and tubing size) and finds
the maximum heat rejection each motor and the coldplate can handle when all heat
sources are scaled proportionally (constant heat ratio) until the first thermal
limit is reached (Motor <= 120 °C, Coldplate <= 65 °C).

Usage:
    python single.py
    python single.py --pumps intg3-570 intg3-570 --rad kart -L 10 --fan OD1238-24HB --tubing thick
    python single.py --pumps intg3-570 ta60 --rad pc -L 2 --fan 12038VA-24R-GA-00 --tubing thin
"""

import argparse
import math
from pathlib import Path
import yaml

from architectures import (
    COLDPLATE_HEAT,
    KG_PER_S_PER_LPM,
    MAX_ITER,
    MOTOR_HEATS,
    T_AMB,
    TOL,
    _get_radiator_params,
)
from coldplate import solve_coldplate
from hydraulics import PUMPS, solve_hydraulics
from motor import solve_motor
from utils.cp import calculate_cp_water

# ==============================================================================
# CONFIGURABLE NOMINAL DEFAULTS
# ==============================================================================
DEFAULT_PUMPS = ["intg3-570", "intg3-570"]  # Array of pump models in series from pumps.yaml
# DEFAULT_RAD_TYPE = "pc"          # "kart" or "pc"
# DEFAULT_L = 2               
DEFAULT_RAD_TYPE = "kart"          # "kart" or "pc"
DEFAULT_L = 11.423               
# DEFAULT_FAN = "No_Fan_Passive"    # Core length/height in inches (or 1 / 2 fans for PC)
DEFAULT_FAN = "12038VA-24R-GA-00"     # Fan model from fans.yaml
DEFAULT_TUBING = "thin"           # "thick" or "thin"
# ==============================================================================

_BASE_DIR = Path(__file__).parent

with open(_BASE_DIR / "loops.yaml") as f:
    LOOP_CONFIGS = yaml.safe_load(f)

with open(_BASE_DIR / "thermals.yaml") as f:
    THERMALS = yaml.safe_load(f)

with open(_BASE_DIR / "fans.yaml") as f:
    FANS = yaml.safe_load(f)

MAX_MOTOR_TEMP = float(THERMALS["motors"]["max_temp"])
MAX_COLDPLATE_TEMP = float(THERMALS["coldplate"]["max_temp"])


def eval_single_scaled(lpm, L, rad_type, cfm, scale=1.0):
    """
    Evaluates the Single Loop with all heat loads scaled by `scale`:
    Pump -> Right Radiator -> RR -> Coldplate -> Left Radiator
         -> RL -> FL -> FR -> Pump -> Right Radiator
    """
    rad_r, t_amb_rad = _get_radiator_params(rad_type, lpm, L, cfm)
    mdot = lpm * KG_PER_S_PER_LPM

    scaled_motors = {k: v * scale for k, v in MOTOR_HEATS.items()}
    scaled_cp = COLDPLATE_HEAT * scale

    q_seg1 = scaled_motors["RR"] + scaled_cp
    q_seg2 = scaled_motors["RL"] + scaled_motors["FL"] + scaled_motors["FR"]

    t_rad_right_out = float(T_AMB)
    t_rad_left_out = float(T_AMB)

    for _ in range(MAX_ITER):
        spec_heat = calculate_cp_water(t_rad_right_out)
        alpha = math.exp(-1.0 / (mdot * spec_heat * rad_r))
        t_out_cp = t_rad_right_out + q_seg1 / (mdot * spec_heat)
        t_rad_left_out = float(t_amb_rad + (t_out_cp - t_amb_rad) * alpha)
        t_in = t_rad_left_out + q_seg2 / (mdot * spec_heat)
        t_rad_right_next = float(t_amb_rad + (t_in - t_amb_rad) * alpha)
        if abs(t_rad_right_next - t_rad_right_out) < TOL:
            t_rad_right_out = t_rad_right_next
            break
        t_rad_right_out = t_rad_right_next

    spec_heat = calculate_cp_water(t_rad_right_out)
    alpha = math.exp(-1.0 / (mdot * spec_heat * rad_r))
    motor_temps = {}
    _, t_out_rr, motor_temps["RR"], _ = solve_motor(
        heat=scaled_motors["RR"], t_in=t_rad_right_out, lpm=lpm, spec_heat=spec_heat
    )
    _, t_out_cp, t_cp, _ = solve_coldplate(
        heat=scaled_cp, t_in=t_out_rr, lpm=lpm, spec_heat=spec_heat
    )
    t_rad_left_out = float(t_amb_rad + (t_out_cp - t_amb_rad) * alpha)
    t_in = t_rad_left_out
    for m_key in ("RL", "FL", "FR"):
        _, t_out, motor_temps[m_key], _ = solve_motor(
            heat=scaled_motors[m_key], t_in=t_in, lpm=lpm, spec_heat=spec_heat
        )
        t_in = t_out

    return {
        "scale": scale,
        "motor_heats": scaled_motors,
        "coldplate_heat": scaled_cp,
        "total_heat": q_seg1 + q_seg2,
        "motor_temps": motor_temps,
        "coldplate_temp": float(t_cp),
    }


def solve_max_heat_rejection(
    pumps=DEFAULT_PUMPS,
    rad_type=DEFAULT_RAD_TYPE,
    L=DEFAULT_L,
    fan=DEFAULT_FAN,
    tubing=DEFAULT_TUBING,
):
    """
    Solves hydraulics and finds the maximum proportional heat scaling factor `scale`
    such that all motors remain <= MAX_MOTOR_TEMP and coldplate <= MAX_COLDPLATE_TEMP.
    """
    if isinstance(pumps, str):
        pump_list = [pumps]
    else:
        pump_list = list(pumps)

    if len(pump_list) < 1:
        raise ValueError("At least one pump must be specified in `pumps`.")

    for p in pump_list:
        if p not in PUMPS:
            raise ValueError(f"Unknown pump '{p}'. Valid pumps: {list(PUMPS.keys())}")

    loop_cfg = dict(LOOP_CONFIGS["Single"])
    loop_cfg["pumps"] = len(pump_list)

    rad_key = rad_type.lower().strip()
    L_val = int(L) if rad_key == "pc" else float(L)

    hyd = solve_hydraulics(
        config=loop_cfg,
        tubing_type=tubing,
        radiator_type=rad_key,
        L=L_val,
        pump=pump_list,
    )
    lpm = hyd["lpm"]
    cfm = FANS[fan]["max_cfm"]

    def _WorstMargin(s):
        res = eval_single_scaled(lpm, L_val, rad_key, cfm, scale=s)
        motor_margin = min(MAX_MOTOR_TEMP - t for t in res["motor_temps"].values())
        cp_margin = MAX_COLDPLATE_TEMP - res["coldplate_temp"]
        return min(motor_margin, cp_margin)

    # Bracket the root [lo, hi] where _WorstMargin(scale) == 0
    lo = 1e-6
    hi = 1.0
    while _WorstMargin(hi) > 0:
        lo = hi
        hi *= 2.0

    # Bisection search to high precision
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if _WorstMargin(mid) > 0:
            lo = mid
        else:
            hi = mid
        if hi - lo < 1e-12:
            break

    best_scale = 0.5 * (lo + hi)
    result = eval_single_scaled(lpm, L_val, rad_key, cfm, scale=best_scale)
    result["lpm"] = lpm
    result["dp"] = hyd["dp"]
    return result


def main():
    parser = argparse.ArgumentParser(
        description="Compute maximum proportional heat rejection for the Single loop."
    )
    parser.add_argument(
        "--pumps",
        "--pump",
        nargs="+",
        default=DEFAULT_PUMPS,
        choices=list(PUMPS.keys()),
        help="Array of pump models in series from pumps.yaml",
    )
    parser.add_argument(
        "--rad",
        choices=["kart", "pc"],
        default=DEFAULT_RAD_TYPE,
        help="Radiator type ('kart' or 'pc')",
    )
    parser.add_argument(
        "-L",
        type=float,
        default=DEFAULT_L,
        help="Radiator parameter L (inches for kart, 1 or 2 fans for pc)",
    )
    parser.add_argument(
        "--fan",
        default=DEFAULT_FAN,
        choices=list(FANS.keys()),
        help="Fan model from fans.yaml",
    )
    parser.add_argument(
        "--tubing",
        choices=["thick", "thin"],
        default=DEFAULT_TUBING,
        help="Tubing size ('thick' or 'thin')",
    )
    args = parser.parse_args()

    res = solve_max_heat_rejection(
        pumps=args.pumps,
        rad_type=args.rad,
        L=args.L,
        fan=args.fan,
        tubing=args.tubing,
    )

    heats = res["motor_heats"]
    cp_heat = res["coldplate_heat"]
    total_heat = res["total_heat"]

    print(f"Max Heat Rejection (Scale = {res['scale']:.4f}x nominal):")
    print(f"  LPM       : {res['lpm']:.2f} LPM")
    print(f"  FL        : {heats['FL']:.2f} W")
    print(f"  FR        : {heats['FR']:.2f} W")
    print(f"  RL        : {heats['RL']:.2f} W")
    print(f"  RR        : {heats['RR']:.2f} W")
    print(f"  Coldplate : {cp_heat:.2f} W")
    print(f"  Total     : {total_heat:.2f} W")


if __name__ == "__main__":
    main()

