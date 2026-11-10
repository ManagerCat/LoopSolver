import csv
from pathlib import Path
import sys
import time
import yaml

from architectures import ARCHITECTURES, eval_coldplate
from hydraulics import solve_hydraulics, solve_hydraulics_parallel

_BASE_DIR = Path(__file__).parent

with open(_BASE_DIR / "loops.yaml") as f:
    LOOP_CONFIGS = yaml.safe_load(f)

with open(_BASE_DIR / "pumps.yaml") as f:
    PUMPS = yaml.safe_load(f)

with open(_BASE_DIR / "fans.yaml") as f:
    FANS = yaml.safe_load(f)

with open(_BASE_DIR / "thermals.yaml") as f:
    THERMALS = yaml.safe_load(f)

with open(_BASE_DIR / "components.yaml") as f:
    COMPONENTS = yaml.safe_load(f)

MAX_MOTOR_TEMP = THERMALS["motors"]["max_temp"]
MAX_COLDPLATE_TEMP = THERMALS["coldplate"]["max_temp"]
TUBING_TYPES = ["thin", "thick"]
KART_AREA_FACTOR = 4.470333333
INITIAL_MASS = COMPONENTS["initial"]["mass"]


def compute_loop_mass_and_power(arch_name, arch_cfg, radiator, L, pump, tubing, fan_name):
    """
    Computes total loop weight (lbs), electrical power consumption (W),
    and detailed weight breakdown (lbs).

    Weight:
      - Uses components.yaml and pumps.yaml (all masses in lbs).
      - For PC rad: if L == 1, rad mass is divided by 2. If L == 2, full pc_rad mass.
      - For Kart rad: mass is kart_rad mass per inch of L (i.e. mass * L).
      - Multiplies component/pump counts by arch_cfg["multiplier"] (e.g. 2 for Left_Right_Split).
      - Adds separate Coldplate loop components if arch_cfg["separate_coldplate"] is True.
      - Adds fan mass:
          * PC rad: L fans per radiator (1 for L=1, 2 for L=2).
          * Kart rad: 1 fan per radiator if L <= 5, 2 fans per radiator if L > 5.

    Power (wattage):
      - Pump wattage from pumps.yaml times number of pumps.
      - Fan power from fans.yaml times number of fans.

    Returns:
      (total_mass, total_power, mass_breakdown)
    """
    multiplier = arch_cfg.get("multiplier", 1)
    motor_loop = LOOP_CONFIGS[arch_name]

    # Gather loop configs scaled by multiplier
    active_loops = [(motor_loop, multiplier)]
    if arch_cfg.get("separate_coldplate", False):
        active_loops.append((LOOP_CONFIGS["Coldplate"], 1))

    initial_mass = float(INITIAL_MASS)
    total_pumps = 0
    total_rads = 0
    tubing_mass_total = 0.0
    fittings_mass_total = 0.0

    for loop_cfg, mult in active_loops:
        num_pumps = loop_cfg.get("pumps", 1) * mult
        num_rads = loop_cfg.get("radiators", 1) * mult
        total_pumps += num_pumps
        total_rads += num_rads

        # Base components in loop
        part_counts = [
            ("sleeve", loop_cfg.get("sleeves", 0) * mult),
            ("coldplate", loop_cfg.get("coldplates", 0) * mult),
            ("closeout", (loop_cfg.get("closeouts", 0) or loop_cfg.get("closeout", 0)) * mult),
            ("bleeding_port", (loop_cfg.get("bleed_ports", 0) or loop_cfg.get("bleeding_ports", 0) or loop_cfg.get("bleed_port", 0) or loop_cfg.get("bleeding_port", 0)) * mult),
            (f"qd_{tubing}", loop_cfg.get("qds", 0) * mult),
            (f"barb_{tubing}", loop_cfg.get("barbs", 0) * mult),
            (f"tubing_{tubing}", loop_cfg.get("tubing", 0) * mult),
            (f"90_{tubing}", loop_cfg.get("90s", 0) * mult),
        ]
        for part_key, count in part_counts:
            part_mass = float(COMPONENTS.get(part_key, {}).get("mass", 0.0))
            if part_key.startswith("tubing_"):
                tubing_mass_total += part_mass * count
            else:
                fittings_mass_total += part_mass * count

    # Pump mass & power
    pump_mass = float(PUMPS[pump].get("mass", 0.0))
    pump_wattage = float(PUMPS[pump].get("wattage", 0.0))
    pump_mass_total = pump_mass * total_pumps
    total_power = pump_wattage * total_pumps

    # Radiator mass & Fan mass/power
    if radiator == "pc":
        rad_mass_unit = COMPONENTS["pc_rad"]["mass"] / 2.0 if L == 1 else COMPONENTS["pc_rad"]["mass"]
        fans_per_rad = 1 if L == 1 else 2
    else:  # kart
        rad_mass_unit = COMPONENTS["kart_rad"]["mass"] * L
        fans_per_rad = 1 if L <= 5 else 2

    rad_mass_total = float(rad_mass_unit) * total_rads

    total_fans = fans_per_rad * total_rads
    fan_mass = float(FANS[fan_name].get("mass", 0.0))
    fan_power = float(FANS[fan_name].get("power", 0.0))
    fan_mass_total = fan_mass * total_fans
    total_power += fan_power * total_fans

    total_mass = (
        initial_mass
        + rad_mass_total
        + pump_mass_total
        + fan_mass_total
        + tubing_mass_total
        + fittings_mass_total
    )

    mass_breakdown = {
        "initial": initial_mass,
        "radiator": rad_mass_total,
        "pump": pump_mass_total,
        "fan": fan_mass_total,
        "tubing": tubing_mass_total,
        "fittings": fittings_mass_total,
        "total": total_mass,
    }

    return total_mass, total_power, mass_breakdown


_HYDRAULICS_CACHE = {}
_THERMALS_CACHE = {}


def _get_cached_hydraulics(arch_name, arch_cfg, radiator, L, pump, tubing):
    hyd_key = (arch_name, tubing, radiator, L, pump)
    cached = _HYDRAULICS_CACHE.get(hyd_key)
    if cached is not None:
        return cached

    hydraulics = {"motor": None, "coldplate": None}
    if arch_name == "4_Motor_Parallel":
        hydraulics["motor"] = solve_hydraulics_parallel(
            LOOP_CONFIGS[arch_name], tubing, radiator, L, pump
        )
        pump_lpm = hydraulics["motor"]["lpm_rad"]
    else:
        hydraulics["motor"] = solve_hydraulics(
            LOOP_CONFIGS[arch_name], tubing, radiator, L, pump
        )
        pump_lpm = hydraulics["motor"]["lpm"]

    if arch_cfg["separate_coldplate"]:
        cp_hyd_key = ("_Coldplate_", tubing, radiator, L, pump)
        cp_hyd = _HYDRAULICS_CACHE.get(cp_hyd_key)
        if cp_hyd is None:
            cp_hyd = solve_hydraulics(
                LOOP_CONFIGS["Coldplate"], tubing, radiator, L, pump
            )
            _HYDRAULICS_CACHE[cp_hyd_key] = cp_hyd
        hydraulics["coldplate"] = cp_hyd
        coldplate_pump_lpm = hydraulics["coldplate"]["lpm"]
    else:
        coldplate_pump_lpm = None

    res = (hydraulics, pump_lpm, coldplate_pump_lpm)
    _HYDRAULICS_CACHE[hyd_key] = res
    return res


def evaluate_config(arch_name, arch_cfg, radiator, L, pump, tubing, fan_name, cfm, no_targets=False):
    """Evaluates hydraulics and thermals for a single configuration at a given L."""
    therm_key = (arch_name, radiator, L, pump, tubing, cfm)
    cached_base = _THERMALS_CACHE.get(therm_key)

    if cached_base is None:
        hydraulics, pump_lpm, coldplate_pump_lpm = _get_cached_hydraulics(
            arch_name, arch_cfg, radiator, L, pump, tubing
        )

        motor_thermals = arch_cfg["eval_func"](hydraulics, L, radiator, cfm)
        if arch_cfg["separate_coldplate"]:
            cp_thermals = eval_coldplate(hydraulics, L, radiator, cfm)
            coldplate_temp = cp_thermals["coldplate_temp"]
        else:
            cp_thermals = None
            coldplate_temp = motor_thermals["coldplate_temp"]

        motor_temps = motor_thermals["motor_temps"]

        # Core area: for kart radiator, multiply L by 4.470333333
        core_area = (L * KART_AREA_FACTOR) if radiator == "kart" else None

        # Thermal resistance (R) values and Heat Cooled (W)
        rad_r = motor_thermals.get("radiator_r")
        sleeve_r = motor_thermals.get("sleeve_r")
        heat_cooled_w = motor_thermals.get("heat_cooled", 0.0)
        if cp_thermals is not None:
            coldplate_r = cp_thermals.get("coldplate_r")
            coldplate_rad_r = cp_thermals.get("radiator_r")
            heat_cooled_w += cp_thermals.get("heat_cooled", 0.0)
        else:
            coldplate_r = motor_thermals.get("coldplate_r")
            coldplate_rad_r = None

        # Collect water temperatures across components
        m_wt = motor_thermals.get("water_temps", {})
        water_temps = {
            "FL": m_wt.get("FL"),
            "FR": m_wt.get("FR"),
            "RL": m_wt.get("RL"),
            "RR": m_wt.get("RR"),
        }
        if cp_thermals is not None:
            cp_wt = cp_thermals.get("water_temps", {})
            water_temps["coldplate"] = cp_wt.get("coldplate")
            water_temps["coldplate_rad"] = cp_wt.get("radiator")
            water_temps["radiator"] = m_wt.get("radiator")
        elif arch_name == "Left_Right_Split":
            water_temps["coldplate_left"] = m_wt.get("coldplate_left")
            water_temps["coldplate_right"] = m_wt.get("coldplate_right")
            cp_vals = [v for v in (m_wt.get("coldplate_left"), m_wt.get("coldplate_right")) if v is not None]
            water_temps["coldplate"] = max(cp_vals) if cp_vals else None
            water_temps["radiator_left"] = m_wt.get("radiator_left")
            water_temps["radiator_right"] = m_wt.get("radiator_right")
            rad_vals = [v for v in (m_wt.get("radiator_left"), m_wt.get("radiator_right")) if v is not None]
            water_temps["radiator"] = max(rad_vals) if rad_vals else None
        elif arch_name == "Single":
            water_temps["coldplate"] = m_wt.get("coldplate")
            water_temps["radiator_left"] = m_wt.get("radiator_left")
            water_temps["radiator_right"] = m_wt.get("radiator_right")
            rad_vals = [v for v in (m_wt.get("radiator_left"), m_wt.get("radiator_right")) if v is not None]
            water_temps["radiator"] = max(rad_vals) if rad_vals else None
        else:
            water_temps["coldplate"] = None
            water_temps["radiator"] = m_wt.get("radiator")
        if "mix" in m_wt:
            water_temps["mix"] = m_wt["mix"]

        cached_base = {
            "architecture": arch_name,
            "radiator": radiator,
            "L": L,
            "core_area": core_area,
            "pump": pump,
            "pump_lpm": pump_lpm,
            "coldplate_pump_lpm": coldplate_pump_lpm,
            "tubing": tubing,
            "cfm": cfm,
            "motor_temps": motor_temps,
            "coldplate_temp": coldplate_temp,
            "water_temps": water_temps,
            "radiator_r": rad_r,
            "coldplate_r": coldplate_r,
            "sleeve_r": sleeve_r,
            "coldplate_rad_r": coldplate_rad_r,
            "heat_cooled_w": heat_cooled_w,
            "hydraulics": hydraulics,
            "motor_thermals": motor_thermals,
            "coldplate_thermals": cp_thermals,
        }
        _THERMALS_CACHE[therm_key] = cached_base

    motor_temps = cached_base["motor_temps"]
    max_motor_temp = max(motor_temps.values())
    coldplate_temp = cached_base["coldplate_temp"]
    if no_targets:
        passed = True
    else:
        passed = (max_motor_temp < MAX_MOTOR_TEMP) and (coldplate_temp < MAX_COLDPLATE_TEMP)

    # Weight and total electrical power - only calculated for configurations that pass
    if passed:
        total_mass, total_power, mass_breakdown = compute_loop_mass_and_power(
            arch_name, arch_cfg, radiator, L, pump, tubing, fan_name
        )
    else:
        total_mass, total_power, mass_breakdown = None, None, {}

    res = dict(cached_base)
    res["passed"] = passed
    res["fan"] = fan_name
    res["total_power_w"] = total_power
    res["total_mass_lbs"] = total_mass
    res["mass_breakdown"] = mass_breakdown
    res["initial_mass_lbs"] = mass_breakdown.get("initial") if passed else None
    res["rad_mass_lbs"] = mass_breakdown.get("radiator") if passed else None
    res["pump_mass_lbs"] = mass_breakdown.get("pump") if passed else None
    res["fan_mass_lbs"] = mass_breakdown.get("fan") if passed else None
    res["tubing_mass_lbs"] = mass_breakdown.get("tubing") if passed else None
    res["fittings_mass_lbs"] = mass_breakdown.get("fittings") if passed else None
    return res


def solve_kart_recursive(arch_name, arch_cfg, pump, tubing, fan_name, cfm, L=6, max_L=10, no_targets=False):
    """Finds the smallest L (in 0.25 increments up to max_L) where thermal limits are met."""
    result_min = evaluate_config(arch_name, arch_cfg, "kart", L, pump, tubing, fan_name, cfm, no_targets=no_targets)
    if result_min["passed"] or L >= max_L:
        return result_min

    num_steps = int(round((max_L - L) / 0.25))
    if num_steps <= 0:
        return result_min

    high_L = L + num_steps * 0.25
    result_max = evaluate_config(arch_name, arch_cfg, "kart", high_L, pump, tubing, fan_name, cfm, no_targets=no_targets)
    if not result_max["passed"]:
        return result_max

    # Binary search over step indices [1, num_steps - 1]
    lo = 1
    hi = num_steps - 1
    best_result = result_max
    while lo <= hi:
        mid = (lo + hi) // 2
        mid_L = L + mid * 0.25
        res_mid = evaluate_config(arch_name, arch_cfg, "kart", mid_L, pump, tubing, fan_name, cfm, no_targets=no_targets)
        if res_mid["passed"]:
            best_result = res_mid
            hi = mid - 1
        else:
            lo = mid + 1

    return best_result


class ProgressBar:
    """A lightweight, zero-dependency console progress bar."""

    def __init__(self, total, prefix="Evaluating Permutations", length=30):
        self.total = max(1, total)
        self.prefix = prefix
        self.length = length
        self.current = 0
        self.start_time = time.time()

    def update(self, n=1, desc=""):
        self.current = min(self.current + n, self.total)
        elapsed = time.time() - self.start_time
        rate = self.current / elapsed if elapsed > 0 else 0.0
        eta = (self.total - self.current) / rate if rate > 0 else 0.0
        percent = (self.current / self.total) * 100.0
        filled = int(self.length * self.current // self.total)
        bar = "█" * filled + "░" * (self.length - filled)

        elapsed_str = f"{int(elapsed // 60):02d}:{int(elapsed % 60):02d}"
        eta_str = f"{int(eta // 60):02d}:{int(eta % 60):02d}"

        suffix = f"[{elapsed_str}<{eta_str}, {rate:4.1f}it/s]"
        if desc:
            suffix += f" ({desc})"

        sys.stdout.write(f"\r{self.prefix}: |{bar}| {self.current}/{self.total} ({percent:5.1f}%) {suffix}")
        sys.stdout.flush()

    def close(self):
        sys.stdout.write("\n")
        sys.stdout.flush()


def run_orchestrator(include_failed=False, show_progress=True, return_stats=False, no_targets=False):
    total_permutations = len(ARCHITECTURES) * len(PUMPS) * len(TUBING_TYPES) * len(FANS) * 3
    passed_count = 0
    failed_count = 0
    results = []

    pbar = ProgressBar(total_permutations, prefix="Solving loops") if show_progress else None

    for arch_name, arch_cfg in ARCHITECTURES.items():
        for pump in PUMPS.keys():
            for tubing in TUBING_TYPES:
                for fan_name, fan_cfg in FANS.items():
                    cfm = fan_cfg["max_cfm"]

                    # 1. Two discrete types of PC radiators (L = 1 and L = 2), evaluated directly
                    for pc_L in (1, 2):
                        res_pc = evaluate_config(
                            arch_name, arch_cfg, "pc", pc_L, pump, tubing, fan_name, cfm, no_targets=no_targets
                        )
                        if res_pc["passed"]:
                            passed_count += 1
                            results.append(res_pc)
                        else:
                            failed_count += 1
                            if include_failed:
                                results.append(res_pc)
                        if pbar:
                            pbar.update(1)

                    # 2. Kart radiator: recursively adjust L from 1 up to 10
                    res_kart = solve_kart_recursive(
                        arch_name, arch_cfg, pump, tubing, fan_name, cfm, L=1, max_L=10, no_targets=no_targets
                    )
                    if res_kart["passed"]:
                        passed_count += 1
                        results.append(res_kart)
                    else:
                        failed_count += 1
                        if include_failed:
                            results.append(res_kart)
                    if pbar:
                        pbar.update(1)

    if pbar:
        pbar.close()

    total_attempts = passed_count + failed_count
    failed_pct = (failed_count / total_attempts * 100) if total_attempts > 0 else 0.0
    passed_pct = (passed_count / total_attempts * 100) if total_attempts > 0 else 0.0

    stats = {
        "total": total_attempts,
        "passed": passed_count,
        "failed": failed_count,
        "passed_pct": passed_pct,
        "failed_pct": failed_pct,
    }

    if return_stats:
        return results, stats
    return results


def load_output_config(config_path=_BASE_DIR / "output.yaml"):
    """
    Loads column and file configuration from output.yaml.
    Supports list of strings, list of dicts, or dict of booleans.
    Returns:
        (columns: list[str], output_filename: str)
    """
    default_columns = [
        "Architecture",
        "Radiator",
        "L",
        "Core_Area_in2",
        "Pump",
        "Pump_LPM",
        "Coldplate_Pump_LPM",
        "Tubing",
        "Fan",
        "Fan_CFM",
        "FL_Temp_C",
        "FR_Temp_C",
        "RL_Temp_C",
        "RR_Temp_C",
        "Coldplate_Temp_C",
        "FL_Water_Temp_C",
        "FR_Water_Temp_C",
        "RL_Water_Temp_C",
        "RR_Water_Temp_C",
        "Coldplate_Water_Temp_C",
        "Radiator_Water_Temp_C",
        "Radiator_R_CW",
        "Coldplate_R_CW",
        "Sleeve_R_CW",
        "Coldplate_Loop_Radiator_R_CW",
        "Heat_Cooled_W",
        "Total_Power_W",
        "Total_Mass_lbs",
        "Radiator_Mass_lbs",
        "Pump_Mass_lbs",
        "Fan_Mass_lbs",
        "Tubing_Mass_lbs",
        "Fittings_Mass_lbs",
        "Initial_Mass_lbs",
    ]
    output_filename = "results.csv"

    if not config_path.exists():
        return default_columns, output_filename

    with open(config_path) as f:
        cfg = yaml.safe_load(f) or {}

    output_filename = cfg.get("output_file", output_filename)
    raw_columns = cfg.get("columns", default_columns)

    if isinstance(raw_columns, list):
        columns = []
        for item in raw_columns:
            if isinstance(item, str):
                columns.append(item)
            elif isinstance(item, dict):
                name = item.get("name")
                enabled = item.get("enabled", True)
                if name and enabled:
                    columns.append(name)
        return columns, output_filename
    elif isinstance(raw_columns, dict):
        columns = [col for col, enabled in raw_columns.items() if enabled]
        return columns, output_filename

    return default_columns, output_filename


def format_column_value(r, col_name):
    """Formats a single column value for a given evaluation result."""
    m = r.get("motor_temps", {})
    wt = r.get("water_temps", {})

    mapping = {
        "Architecture": lambda: r.get("architecture"),
        "Radiator": lambda: r.get("radiator"),
        "L": lambda: r.get("L"),
        "Core_Area_in2": lambda: f"{r['core_area']:.2f}" if r.get("core_area") is not None else "N/A",
        "Pump": lambda: r.get("pump"),
        "Pump_LPM": lambda: round(r["pump_lpm"], 2) if r.get("pump_lpm") is not None else "N/A",
        "Coldplate_Pump_LPM": lambda: round(r["coldplate_pump_lpm"], 2) if r.get("coldplate_pump_lpm") is not None else "N/A",
        "Tubing": lambda: r.get("tubing"),
        "Fan": lambda: r.get("fan"),
        "Fan_CFM": lambda: r.get("cfm"),
        # Motor temps
        "FL_Temp_C": lambda: round(m["FL"], 2) if "FL" in m else "N/A",
        "FR_Temp_C": lambda: round(m["FR"], 2) if "FR" in m else "N/A",
        "RL_Temp_C": lambda: round(m["RL"], 2) if "RL" in m else "N/A",
        "RR_Temp_C": lambda: round(m["RR"], 2) if "RR" in m else "N/A",
        "Max_Motor_Temp_C": lambda: round(max(m.values()), 2) if m else "N/A",
        # Coldplate temp
        "Coldplate_Temp_C": lambda: round(r["coldplate_temp"], 2) if r.get("coldplate_temp") is not None else "N/A",
        # Coolant / Water outlet temps
        "FL_Water_Temp_C": lambda: round(wt["FL"], 2) if wt.get("FL") is not None else "N/A",
        "FR_Water_Temp_C": lambda: round(wt["FR"], 2) if wt.get("FR") is not None else "N/A",
        "RL_Water_Temp_C": lambda: round(wt["RL"], 2) if wt.get("RL") is not None else "N/A",
        "RR_Water_Temp_C": lambda: round(wt["RR"], 2) if wt.get("RR") is not None else "N/A",
        "Coldplate_Water_Temp_C": lambda: round(wt["coldplate"], 2) if wt.get("coldplate") is not None else "N/A",
        "Radiator_Water_Temp_C": lambda: round(wt["radiator"], 2) if wt.get("radiator") is not None else "N/A",
        "Coldplate_Loop_Radiator_Water_Temp_C": lambda: round(wt["coldplate_rad"], 2) if wt.get("coldplate_rad") is not None else "N/A",
        "Radiator_Left_Water_Temp_C": lambda: round(wt["radiator_left"], 2) if wt.get("radiator_left") is not None else "N/A",
        "Radiator_Right_Water_Temp_C": lambda: round(wt["radiator_right"], 2) if wt.get("radiator_right") is not None else "N/A",
        "Coldplate_Left_Water_Temp_C": lambda: round(wt["coldplate_left"], 2) if wt.get("coldplate_left") is not None else "N/A",
        "Coldplate_Right_Water_Temp_C": lambda: round(wt["coldplate_right"], 2) if wt.get("coldplate_right") is not None else "N/A",
        "Parallel_Mix_Water_Temp_C": lambda: round(wt["mix"], 2) if wt.get("mix") is not None else "N/A",
        # Resistances (deg C/W or K/W)
        "Radiator_R_CW": lambda: round(r["radiator_r"], 5) if r.get("radiator_r") is not None else "N/A",
        "Coldplate_R_CW": lambda: round(r["coldplate_r"], 5) if r.get("coldplate_r") is not None else "N/A",
        "Sleeve_R_CW": lambda: round(r["sleeve_r"], 5) if r.get("sleeve_r") is not None else "N/A",
        "Coldplate_Loop_Radiator_R_CW": lambda: round(r["coldplate_rad_r"], 5) if r.get("coldplate_rad_r") is not None else "N/A",
        "Radiator_R_KW": lambda: round(r["radiator_r"], 5) if r.get("radiator_r") is not None else "N/A",
        "Coldplate_R_KW": lambda: round(r["coldplate_r"], 5) if r.get("coldplate_r") is not None else "N/A",
        "Sleeve_R_KW": lambda: round(r["sleeve_r"], 5) if r.get("sleeve_r") is not None else "N/A",
        "Coldplate_Loop_Radiator_R_KW": lambda: round(r["coldplate_rad_r"], 5) if r.get("coldplate_rad_r") is not None else "N/A",
        # Heat cooled, power & mass
        "Heat_Cooled_W": lambda: round(r["heat_cooled_w"], 2) if r.get("heat_cooled_w") is not None else "N/A",
        "Total_Power_W": lambda: round(r["total_power_w"], 2) if r.get("total_power_w") is not None else "N/A",
        "Total_Mass_lbs": lambda: round(r["total_mass_lbs"], 3) if r.get("total_mass_lbs") is not None else "N/A",
        "Radiator_Mass_lbs": lambda: round(r["rad_mass_lbs"], 3) if r.get("rad_mass_lbs") is not None else "N/A",
        "Rad_Mass_lbs": lambda: round(r["rad_mass_lbs"], 3) if r.get("rad_mass_lbs") is not None else "N/A",
        "Pump_Mass_lbs": lambda: round(r["pump_mass_lbs"], 3) if r.get("pump_mass_lbs") is not None else "N/A",
        "Fan_Mass_lbs": lambda: round(r["fan_mass_lbs"], 3) if r.get("fan_mass_lbs") is not None else "N/A",
        "Tubing_Mass_lbs": lambda: round(r["tubing_mass_lbs"], 3) if r.get("tubing_mass_lbs") is not None else "N/A",
        "Fittings_Mass_lbs": lambda: round(r["fittings_mass_lbs"], 3) if r.get("fittings_mass_lbs") is not None else "N/A",
        "Initial_Mass_lbs": lambda: round(r["initial_mass_lbs"], 3) if r.get("initial_mass_lbs") is not None else "N/A",
        "Passed": lambda: r.get("passed"),
    }

    if col_name in mapping:
        return mapping[col_name]()

    if col_name in r:
        val = r[col_name]
        return round(val, 3) if isinstance(val, float) else val
    if col_name in wt:
        val = wt[col_name]
        return round(val, 2) if isinstance(val, float) else val
    if col_name in m:
        val = m[col_name]
        return round(val, 2) if isinstance(val, float) else val

    return "N/A"


def export_to_csv(results, output_path=None, config_path=_BASE_DIR / "output.yaml"):
    fieldnames, default_filename = load_output_config(config_path)
    if output_path is None:
        output_path = _BASE_DIR / default_filename
    else:
        output_path = Path(output_path)

    with open(output_path, mode="w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            row = {col: format_column_value(r, col) for col in fieldnames}
            writer.writerow(row)
    return output_path


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Run cooling loop orchestrator over permutations."
    )
    parser.add_argument(
        "--no-targets",
        "--no_targets",
        "-nt",
        dest="no_targets",
        action="store_true",
        default=False,
        help="Ignore thermal targets so that none of the runs fail.",
    )
    parser.add_argument(
        "--include-failed",
        "--include_failed",
        dest="include_failed",
        action="store_true",
        default=False,
        help="Include failed configurations in results.",
    )
    parser.add_argument(
        "--no-progress",
        "--no_progress",
        dest="no_progress",
        action="store_true",
        default=False,
        help="Disable progress bar.",
    )
    parser.add_argument(
        "-o",
        "--output",
        dest="output",
        default=None,
        help="Custom output CSV path.",
    )
    args = parser.parse_args()

    all_results, stats = run_orchestrator(
        include_failed=args.include_failed,
        show_progress=not args.no_progress,
        return_stats=True,
        no_targets=args.no_targets,
    )
    csv_path = export_to_csv(all_results, output_path=args.output)

    print("\n" + "=" * 55)
    print("RUN SUMMARY")
    print("=" * 55)
    print(f"Total Permutations Evaluated: {stats['total']}")
    print(f"Passed Configurations:        {stats['passed']} ({stats['passed_pct']:.2f}%)")
    print(f"Failed Configurations:        {stats['failed']} ({stats['failed_pct']:.2f}%)")
    print("=" * 55)
    print(f"Exported {len(all_results)} passing configurations to {csv_path.name}")

