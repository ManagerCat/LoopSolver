import math
from pathlib import Path
import yaml

from motor import solve_motor
from coldplate import solve_coldplate
import kart_radiator
import pc_radiator
from utils.cp import calculate_cp_water

# Load thermal constants from thermals.yaml
_THERMALS_FILE = Path(__file__).parent / "thermals.yaml"
with open(_THERMALS_FILE) as f:
    THERMALS = yaml.safe_load(f)

MOTOR_HEATS = THERMALS["motors"]["heat"]      # {"FL": 877, "FR": 592, "RL": 744, "RR": 525}
COLDPLATE_HEAT = THERMALS["coldplate"]["heat"]  # 720 W
T_AMB = THERMALS["ambient"]["t"]              # 35 deg C
RHO = float(THERMALS["coolant"]["rho"])       # kg/m^3
KG_PER_S_PER_LPM = RHO / 60000.0              # mdot [kg/s] = rho * LPM / 60000

TOL = 1e-9
MAX_ITER = 5000


def _get_radiator_solver(rad_type):
    rad_key = rad_type.lower().strip()
    if rad_key == "pc":
        solver = pc_radiator.solve_radiator
        r_func = pc_radiator._predict_radiator_thermal_resistance
    elif rad_key == "kart":
        solver = kart_radiator.solve_radiator
        r_func = kart_radiator.predict_radiator_thermal_resistance
    else:
        raise ValueError(f"Unknown radiator type '{rad_type}'. Expected 'pc' or 'kart'.")

    def _solve(t_in, lpm, L, cfm, *args, **kwargs):
        res = solver(t_in, lpm, L, cfm, *args, **kwargs)
        if isinstance(res, tuple):
            t_out, r = res
        else:
            t_out = res
            r = r_func(cfm, lpm, L)
        return (float(t_out) if t_out is not None else None), float(r)

    return _solve


def _get_radiator_params(rad_type, lpm, L, cfm):
    """Precomputes radiator thermal resistance r and ambient temperature."""
    rad_key = rad_type.lower().strip()
    if rad_key == "pc":
        r = float(pc_radiator._predict_radiator_thermal_resistance(cfm, lpm, L))
        t_amb_rad = pc_radiator.t_amb
    elif rad_key == "kart":
        r = float(kart_radiator.predict_radiator_thermal_resistance(cfm, lpm, L))
        t_amb_rad = kart_radiator.t_amb
    else:
        raise ValueError(f"Unknown radiator type '{rad_type}'. Expected 'pc' or 'kart'.")
    return r, t_amb_rad


def eval_coldplate(hydraulics, L, rad_type, cfm, q_cp=COLDPLATE_HEAT):
    """
    Solves the standalone closed coldplate loop:
    Radiator -> Pump -> Coldplate -> Radiator
    """
    if isinstance(hydraulics, (int, float)):
        lpmColdplate = float(hydraulics)
    elif "coldplate" in hydraulics and hydraulics["coldplate"] is not None:
        lpmColdplate = hydraulics["coldplate"]["lpm"]
    else:
        lpmColdplate = hydraulics["lpm"]

    rad_r, t_amb_rad = _get_radiator_params(rad_type, lpmColdplate, L, cfm)
    mdot_cp = lpmColdplate * KG_PER_S_PER_LPM

    t_rad_out = float(T_AMB)
    for _ in range(MAX_ITER):
        spec_heat = calculate_cp_water(t_rad_out)
        t_out_cp = t_rad_out + q_cp / (mdot_cp * spec_heat)
        alpha = math.exp(-1.0 / (mdot_cp * spec_heat * rad_r))
        t_rad_next = float(t_amb_rad + (t_out_cp - t_amb_rad) * alpha)
        if abs(t_rad_next - t_rad_out) < TOL:
            t_rad_out = t_rad_next
            break
        t_rad_out = t_rad_next

    spec_heat = calculate_cp_water(t_rad_out)
    _, t_out_cp, t_cp, cp_r = solve_coldplate(heat=q_cp, t_in=t_rad_out, lpm=lpmColdplate, spec_heat=spec_heat)
    alpha = math.exp(-1.0 / (mdot_cp * spec_heat * rad_r))
    t_rad_out = float(t_amb_rad + (t_out_cp - t_amb_rad) * alpha)
    heat_cooled = (mdot_cp * spec_heat) * (t_out_cp - t_rad_out)

    return {
        "water_temps": {
            "radiator": float(t_rad_out),
            "coldplate": float(t_out_cp),
        },
        "coldplate_temp": float(t_cp),
        "coldplate_r": float(cp_r),
        "radiator_r": float(rad_r),
        "heat_cooled": float(heat_cooled),
    }


# -------------------------------------------------------------
# Routing Functions for each architecture
# -------------------------------------------------------------

def eval_4_motor_series_ccw(hydraulics, L, rad_type, cfm):
    """
    4 Motor Series (Counter-Clockwise as shown in diagram):
    - Motor loop: Radiator -> Pump -> RL -> RR -> FR -> FL -> Radiator
    """
    lpmMotor = hydraulics["motor"]["lpm"] if "motor" in hydraulics else hydraulics["lpm"]
    rad_r, t_amb_rad = _get_radiator_params(rad_type, lpmMotor, L, cfm)
    motor_order = ("RL", "RR", "FR", "FL")
    mdot = lpmMotor * KG_PER_S_PER_LPM
    total_motor_heat = sum(MOTOR_HEATS[m_key] for m_key in motor_order)

    t_rad_out = float(T_AMB)
    for _ in range(MAX_ITER):
        spec_heat = calculate_cp_water(t_rad_out)
        t_in = t_rad_out + total_motor_heat / (mdot * spec_heat)
        alpha = math.exp(-1.0 / (mdot * spec_heat * rad_r))
        t_rad_next = float(t_amb_rad + (t_in - t_amb_rad) * alpha)
        if abs(t_rad_next - t_rad_out) < TOL:
            t_rad_out = t_rad_next
            break
        t_rad_out = t_rad_next

    spec_heat = calculate_cp_water(t_rad_out)
    motor_temps = {}
    water_temps = {}
    t_in = t_rad_out
    sleeve_r = 0.0
    for m_key in motor_order:
        _, t_out, tc_t, sleeve_r = solve_motor(heat=MOTOR_HEATS[m_key], t_in=t_in, lpm=lpmMotor, spec_heat=spec_heat)
        motor_temps[m_key] = float(tc_t)
        water_temps[m_key] = float(t_out)
        t_in = t_out

    alpha = math.exp(-1.0 / (mdot * spec_heat * rad_r))
    t_rad_out = float(t_amb_rad + (t_in - t_amb_rad) * alpha)
    water_temps["radiator"] = float(t_rad_out)
    heat_cooled = (mdot * spec_heat) * (t_in - t_rad_out)

    return {
        "water_temps": water_temps,
        "motor_temps": {
            "FL": motor_temps["FL"],
            "FR": motor_temps["FR"],
            "RL": motor_temps["RL"],
            "RR": motor_temps["RR"],
        },
        "radiator_r": float(rad_r),
        "sleeve_r": float(sleeve_r),
        "heat_cooled": float(heat_cooled),
    }


def eval_4_motor_series_cw(hydraulics, L, rad_type, cfm):
    """
    4 Motor Series (Clockwise variation):
    - Motor loop: Radiator -> Pump -> FL -> FR -> RR -> RL -> Radiator
    """
    lpmMotor = hydraulics["motor"]["lpm"] if "motor" in hydraulics else hydraulics["lpm"]
    rad_r, t_amb_rad = _get_radiator_params(rad_type, lpmMotor, L, cfm)
    motor_order = ("FL", "FR", "RR", "RL")
    mdot = lpmMotor * KG_PER_S_PER_LPM
    total_motor_heat = sum(MOTOR_HEATS[m_key] for m_key in motor_order)

    t_rad_out = float(T_AMB)
    for _ in range(MAX_ITER):
        spec_heat = calculate_cp_water(t_rad_out)
        t_in = t_rad_out + total_motor_heat / (mdot * spec_heat)
        alpha = math.exp(-1.0 / (mdot * spec_heat * rad_r))
        t_rad_next = float(t_amb_rad + (t_in - t_amb_rad) * alpha)
        if abs(t_rad_next - t_rad_out) < TOL:
            t_rad_out = t_rad_next
            break
        t_rad_out = t_rad_next

    spec_heat = calculate_cp_water(t_rad_out)
    motor_temps = {}
    water_temps = {}
    t_in = t_rad_out
    sleeve_r = 0.0
    for m_key in motor_order:
        _, t_out, tc_t, sleeve_r = solve_motor(heat=MOTOR_HEATS[m_key], t_in=t_in, lpm=lpmMotor, spec_heat=spec_heat)
        motor_temps[m_key] = float(tc_t)
        water_temps[m_key] = float(t_out)
        t_in = t_out

    alpha = math.exp(-1.0 / (mdot * spec_heat * rad_r))
    t_rad_out = float(t_amb_rad + (t_in - t_amb_rad) * alpha)
    water_temps["radiator"] = float(t_rad_out)
    heat_cooled = (mdot * spec_heat) * (t_in - t_rad_out)

    return {
        "water_temps": water_temps,
        "motor_temps": {
            "FL": motor_temps["FL"],
            "FR": motor_temps["FR"],
            "RL": motor_temps["RL"],
            "RR": motor_temps["RR"],
        },
        "radiator_r": float(rad_r),
        "sleeve_r": float(sleeve_r),
        "heat_cooled": float(heat_cooled),
    }


def eval_4_motor_parallel(hydraulics, L, rad_type, cfm):
    r"""
    4 Motor Parallel -- symmetric 2x2 series-parallel architecture
    (Left Bank and Right Bank in parallel, each containing 2 motors in series):

    - Main loop (Q_total = lpm_rad):
        Pump -> Radiator -> Supply Splitter Tee ... Return Collector Tee -> Pump
    - Branch legs (Q_branch = Q_total / 2 = lpm_motor):
        * Left Bank:  Supply Tee -> FL (front) -> RL (rear) -> Return Tee
        * Right Bank: Supply Tee -> FR (front) -> RR (rear) -> Return Tee

    Thermal march (at the hydraulic operating point Q_total*, Q_branch*):
        T_cold        = T_rad,out
        T_in,front    = T_cold
        T_out,front   = T_in,front + Qdot_front / (rho * c_p * Q_branch*)
        T_in,rear     = T_out,front                  (pre-heated by the front motor)
        T_out,rear    = T_in,rear + Qdot_rear / (rho * c_p * Q_branch*)
        T_rad,in      = flow-weighted mix of the two bank outlets (RL, RR)

    With identical bank heat loads, T_rad,in reduces to T_out,rear. With the
    asymmetric loads in thermals.yaml, each bank is marched with its own heats
    and the return-tee mix conserves energy.
    """
    motor_hyd = hydraulics["motor"] if "motor" in hydraulics else hydraulics
    lpmMotor = motor_hyd["lpm_motor"]  # Q_branch*
    lpmRad = motor_hyd["lpm_rad"]      # Q_total*
    rad_r, t_amb_rad = _get_radiator_params(rad_type, lpmRad, L, cfm)

    mdot_branch = lpmMotor * KG_PER_S_PER_LPM
    mdot_rad = lpmRad * KG_PER_S_PER_LPM
    # Banks: (front motor, rear motor)
    banks = {"left": ("FL", "RL"), "right": ("FR", "RR")}
    bank_mdot = {name: mdot_branch for name in banks}
    bank_mdot_total = sum(bank_mdot.values())

    def _mix(bank_outlets):
        """Flow-weighted (enthalpy) mix at the return collector tee."""
        return sum(bank_mdot[b] * bank_outlets[b] for b in banks) / bank_mdot_total

    bank_heat = {name: MOTOR_HEATS[f] + MOTOR_HEATS[r] for name, (f, r) in banks.items()}

    # Fixed-point iteration on the radiator outlet temperature (T_cold)
    t_rad_out = float(T_AMB)
    for _ in range(MAX_ITER):
        spec_heat = calculate_cp_water(t_rad_out)
        bank_outlets = {b: t_rad_out + bank_heat[b] / (bank_mdot[b] * spec_heat) for b in banks}
        t_mix = _mix(bank_outlets)
        alpha = math.exp(-1.0 / (mdot_rad * spec_heat * rad_r))
        t_rad_next = float(t_amb_rad + (t_mix - t_amb_rad) * alpha)
        if abs(t_rad_next - t_rad_out) < TOL:
            t_rad_out = t_rad_next
            break
        t_rad_out = t_rad_next

    # Thermal march through each bank at Q_branch*
    spec_heat = calculate_cp_water(t_rad_out)
    t_cold = t_rad_out
    motor_temps = {}
    water_out = {}
    bank_outlets = {}
    sleeve_r = 0.0
    for b, (front, rear) in banks.items():
        # Upstream (front) motor: inlet is radiator outlet
        _, t_out_front, motor_temps[front], sleeve_r = solve_motor(
            heat=MOTOR_HEATS[front], t_in=t_cold, lpm=lpmMotor, spec_heat=spec_heat
        )
        # Downstream (rear) motor: inlet is pre-heated front-motor outlet
        _, t_out_rear, motor_temps[rear], _ = solve_motor(
            heat=MOTOR_HEATS[rear], t_in=t_out_front, lpm=lpmMotor, spec_heat=spec_heat
        )
        water_out[front] = t_out_front
        water_out[rear] = t_out_rear
        bank_outlets[b] = t_out_rear

    # Radiator inlet = bulk mixed fluid at the return collector tee
    t_mix = _mix(bank_outlets)
    t_out_fl, t_out_rl = water_out["FL"], water_out["RL"]
    t_out_fr, t_out_rr = water_out["FR"], water_out["RR"]
    alpha = math.exp(-1.0 / (mdot_rad * spec_heat * rad_r))
    t_rad_out = float(t_amb_rad + (t_mix - t_amb_rad) * alpha)
    heat_cooled = (mdot_rad * spec_heat) * (t_mix - t_rad_out)

    return {
        "water_temps": {
            "radiator": float(t_rad_out),
            "FL": float(t_out_fl),
            "RL": float(t_out_rl),
            "FR": float(t_out_fr),
            "RR": float(t_out_rr),
            "mix": float(t_mix),
        },
        "motor_temps": {
            "FL": float(motor_temps["FL"]),
            "FR": float(motor_temps["FR"]),
            "RL": float(motor_temps["RL"]),
            "RR": float(motor_temps["RR"]),
        },
        "radiator_r": float(rad_r),
        "sleeve_r": float(sleeve_r),
        "heat_cooled": float(heat_cooled),
    }


def eval_left_right(hydraulics, L, rad_type, cfm):
    """
    Left/Right Split (2 independent closed loops, each cooling half the coldplate):
    - Left side:  Radiator -> Pump -> Coldplate (heat / 2) -> RL -> FL -> Radiator
    - Right side: Radiator -> Pump -> Coldplate (half)     -> RR -> FR -> Radiator
    """
    lpmMotor = hydraulics["motor"]["lpm"]
    rad_r, t_amb_rad = _get_radiator_params(rad_type, lpmMotor, L, cfm)
    q_cp_half = COLDPLATE_HEAT / 2.0
    mdot = lpmMotor * KG_PER_S_PER_LPM
    motor_temps = {}
    water_temps = {}

    def _solve_side(side_name, rear_key, front_key):
        q_side = q_cp_half + MOTOR_HEATS[rear_key] + MOTOR_HEATS[front_key]
        t_rad_out = float(T_AMB)

        for _ in range(MAX_ITER):
            spec_heat = calculate_cp_water(t_rad_out)
            t_after_front = t_rad_out + q_side / (mdot * spec_heat)
            alpha = math.exp(-1.0 / (mdot * spec_heat * rad_r))
            t_rad_next = float(t_amb_rad + (t_after_front - t_amb_rad) * alpha)
            if abs(t_rad_next - t_rad_out) < TOL:
                t_rad_out = t_rad_next
                break
            t_rad_out = t_rad_next

        spec_heat = calculate_cp_water(t_rad_out)
        _, t_after_cp, t_cp_side, cp_r = solve_coldplate(heat=q_cp_half, t_in=t_rad_out, lpm=lpmMotor, spec_heat=spec_heat)
        _, t_after_rear, motor_temps[rear_key], sleeve_r = solve_motor(heat=MOTOR_HEATS[rear_key], t_in=t_after_cp, lpm=lpmMotor, spec_heat=spec_heat)
        _, t_after_front, motor_temps[front_key], _ = solve_motor(heat=MOTOR_HEATS[front_key], t_in=t_after_rear, lpm=lpmMotor, spec_heat=spec_heat)

        alpha = math.exp(-1.0 / (mdot * spec_heat * rad_r))
        t_rad_out = float(t_amb_rad + (t_after_front - t_amb_rad) * alpha)
        q_cooled_side = (mdot * spec_heat) * (t_after_front - t_rad_out)

        water_temps[f"radiator_{side_name}"] = float(t_rad_out)
        water_temps[f"coldplate_{side_name}"] = float(t_after_cp)
        water_temps[rear_key] = float(t_after_rear)
        water_temps[front_key] = float(t_after_front)
        return float(t_cp_side), float(cp_r), float(rad_r), float(sleeve_r), float(q_cooled_side)

    t_cp_left, cp_r, rad_r, sleeve_r, q_cooled_left = _solve_side("left", "RL", "FL")
    t_cp_right, _, _, _, q_cooled_right = _solve_side("right", "RR", "FR")

    return {
        "water_temps": water_temps,
        "motor_temps": {
            "FL": float(motor_temps["FL"]),
            "FR": float(motor_temps["FR"]),
            "RL": float(motor_temps["RL"]),
            "RR": float(motor_temps["RR"]),
        },
        "coldplate_temp": float(max(t_cp_left, t_cp_right)),
        "coldplate_r": float(cp_r),
        "radiator_r": float(rad_r),
        "sleeve_r": float(sleeve_r),
        "heat_cooled": float(q_cooled_left + q_cooled_right),
    }


def eval_single(hydraulics, L, rad_type, cfm):
    """
    Single Loop:
    Pump -> Right Radiator -> RR -> Coldplate (full heat) -> Left Radiator
         -> RL -> FL -> FR -> Pump -> Right Radiator
    """
    lpmMotor = hydraulics["motor"]["lpm"]
    rad_r, t_amb_rad = _get_radiator_params(rad_type, lpmMotor, L, cfm)
    mdot = lpmMotor * KG_PER_S_PER_LPM
    q_seg1 = MOTOR_HEATS["RR"] + COLDPLATE_HEAT
    q_seg2 = MOTOR_HEATS["RL"] + MOTOR_HEATS["FL"] + MOTOR_HEATS["FR"]

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
    water_temps = {}
    _, t_out_rr, motor_temps["RR"], sleeve_r = solve_motor(heat=MOTOR_HEATS["RR"], t_in=t_rad_right_out, lpm=lpmMotor, spec_heat=spec_heat)
    _, t_out_cp, t_cp, cp_r = solve_coldplate(heat=COLDPLATE_HEAT, t_in=t_out_rr, lpm=lpmMotor, spec_heat=spec_heat)
    t_rad_left_out = float(t_amb_rad + (t_out_cp - t_amb_rad) * alpha)
    t_in = t_rad_left_out
    for m_key in ("RL", "FL", "FR"):
        _, t_out, motor_temps[m_key], _ = solve_motor(heat=MOTOR_HEATS[m_key], t_in=t_in, lpm=lpmMotor, spec_heat=spec_heat)
        water_temps[m_key] = float(t_out)
        t_in = t_out

    t_rad_right_out = float(t_amb_rad + (t_in - t_amb_rad) * alpha)
    q_cooled_left = (mdot * spec_heat) * (t_out_cp - t_rad_left_out)
    q_cooled_right = (mdot * spec_heat) * (t_in - t_rad_right_out)

    water_temps = {
        "radiator_right": float(t_rad_right_out),
        "RR": float(t_out_rr),
        "coldplate": float(t_out_cp),
        "radiator_left": float(t_rad_left_out),
        "RL": water_temps["RL"],
        "FL": water_temps["FL"],
        "FR": water_temps["FR"],
    }

    return {
        "water_temps": water_temps,
        "motor_temps": {
            "FL": float(motor_temps["FL"]),
            "FR": float(motor_temps["FR"]),
            "RL": float(motor_temps["RL"]),
            "RR": float(motor_temps["RR"]),
        },
        "coldplate_temp": float(t_cp),
        "coldplate_r": float(cp_r),
        "radiator_r": float(rad_r),
        "sleeve_r": float(sleeve_r),
        "heat_cooled": float(q_cooled_left + q_cooled_right),
    }


# -------------------------------------------------------------
# Master Architecture Registry
# -------------------------------------------------------------
ARCHITECTURES = {
    "4_Motor_Series_Counterclockwise": {
        "eval_func": eval_4_motor_series_ccw,
        "multiplier": 1,
        "separate_coldplate": True
    },
    "4_Motor_Series_Clockwise": {
        "eval_func": eval_4_motor_series_cw,
        "multiplier": 1,
        "separate_coldplate": True
    },
    "4_Motor_Parallel": {
        "eval_func": eval_4_motor_parallel,
        "multiplier": 1,
        "separate_coldplate": True
    },
    "Left_Right_Split": {

        "eval_func": eval_left_right,
        "multiplier": 2,
        "separate_coldplate": False
    },
    "Single": {
        "eval_func": eval_single,
        "multiplier": 1,
        "separate_coldplate": False
    },
}
