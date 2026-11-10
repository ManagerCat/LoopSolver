import math
from pathlib import Path
import yaml
from utils.cp import calculate_cp_water
from utils.v import getv

_COEFFS_FILE = Path(__file__).parent / "radiator.yaml"
with open(_COEFFS_FILE) as f:
    PC_RAD_COEFFS = yaml.safe_load(f)["pc_radiator"]

_PC_A = PC_RAD_COEFFS["a"]
_PC_B = PC_RAD_COEFFS["b"]
_PC_C = PC_RAD_COEFFS["c"]

_THERMALS_FILE = Path(__file__).parent / "thermals.yaml"
with open(_THERMALS_FILE) as f:
    THERMALS = yaml.safe_load(f)
t_amb = THERMALS["ambient"]["t"]
_KG_PER_S_PER_LPM = float(THERMALS["coolant"]["rho"]) / 60000.0  # mdot = rho * LPM / 60000

def solve_radiator(t_in, lpm, L, cfm, spec_heat=None):
    """
    Solves PC radiator outlet water temperature and thermal resistance.

    Parameters
    ----------
    t_in : float
        Inlet water temperature (deg C).
    lpm : float
        Coolant volumetric flow rate (LPM).

    Returns
    -------
    t_out : float
        Outlet water temperature (deg C).
    r : float
        Radiator thermal resistance (K/W).
    """
    if spec_heat is None:
        spec_heat = calculate_cp_water(t_in)
    r = _predict_radiator_thermal_resistance(cfm, lpm, L)
    return t_amb + (t_in - t_amb) * math.exp(-1 / (((_KG_PER_S_PER_LPM * lpm) * spec_heat) * r)), r

def _predict_radiator_thermal_resistance(cfm, Q, L, coeffs=PC_RAD_COEFFS):
    if coeffs is PC_RAD_COEFFS:
        a, b, c = _PC_A, _PC_B, _PC_C
    else:
        a = coeffs["a"]
        b = coeffs["b"]
        c = coeffs["c"]

    v = getv(cfm, "pc")
    r = a * (v**b) * (Q**c)
    if L == 1:
        r *= 2
    return r


    