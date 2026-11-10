from pathlib import Path
import yaml
from utils.cp import calculate_cp_water

_THERMALS_FILE = Path(__file__).parent / "thermals.yaml"
with open(_THERMALS_FILE) as f:
    _THERMALS = yaml.safe_load(f)
_MAX_TEMP = _THERMALS["coldplate"]["max_temp"]
_KG_PER_S_PER_LPM = float(_THERMALS["coolant"]["rho"]) / 60000.0  # mdot = rho * LPM / 60000


def solve_coldplate(heat, t_in, lpm, spec_heat=None):
    if spec_heat is None:
        spec_heat = calculate_cp_water(t_in)
    r = 0.0283 * (lpm ** -0.448177)
    dT = heat / (_KG_PER_S_PER_LPM * lpm * spec_heat)
    t_out = t_in + dT
    t_avg = (t_in + t_out) / 2
    t_coldplate = t_avg + (heat * r)
    thermal_margin = _MAX_TEMP - t_coldplate
    return (float(thermal_margin), float(t_out), float(t_coldplate), float(r))
