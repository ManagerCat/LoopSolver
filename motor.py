import math
import yaml
from pathlib import Path
from utils.cp import calculate_cp_water

_MOTORS_FILE = Path(__file__).parent / "thermals.yaml"
with open(_MOTORS_FILE) as f:
    _THERMALS = yaml.safe_load(f)
MOTORS_DATA = _THERMALS["motors"]
motor_r = MOTORS_DATA.get("motor_r")
motor_max_temp = MOTORS_DATA.get("max_temp")

# Precomputed physical and geometric constants
_H = 0.0047625
_D = 0.1006729
_L_AVG = 1.150804862
_W = 0.018796
_RHO = float(_THERMALS["coolant"]["rho"])  # kg/m^3
_KG_PER_S_PER_LPM = _RHO / 60000.0  # mdot [kg/s] = rho * LPM / 60000
_D_HYD = (4 * _H * _W) / (2 * _H + 2 * _W)
_MU = 0.000547
_K = 0.6406
_HW = _H * _W
_RHO_D_MU = _RHO * _D_HYD / _MU
_DD_RATIO = _D_HYD / _D
_DD_05_3 = 0.3 * (_DD_RATIO ** 0.5)
_DD_2 = _DD_RATIO ** 2
_DD_0077 = _DD_RATIO ** 0.077
_MU_OVER_K = _MU / _K
_K_OVER_D = _K / _D_HYD
_AREA_SCALE = _L_AVG * _W / 1.53


def solve_motor(heat, t_in, lpm, spec_heat=None):
    if spec_heat is None:
        spec_heat = calculate_cp_water(t_in)
    v = (lpm / 60000) / _HW
    Re = v * _RHO_D_MU
    re_dd2_02 = (Re * _DD_2) ** 0.2
    f_MN = (_DD_05_3 / re_dd2_02) * (1 + (0.112 / re_dd2_02))
    Pr = _MU_OVER_K * spec_heat
    MN = 0.04 * _DD_0077 * f_MN * (Re ** 1.13) * (Pr ** 0.4)
    htc = MN * _K_OVER_D
    c = htc * _AREA_SCALE
    sleeve_r = 1 / c

    # print(c)
    dT = heat / (_KG_PER_S_PER_LPM * lpm * spec_heat)
    # print(dT)
    t_out = t_in + dT
    exp_val = math.exp(dT / (heat / c))
    t_motor = (exp_val * (t_out) - t_in) / (exp_val - 1)
    tc_t = t_motor + heat * motor_r
    thermal_margin = motor_max_temp - tc_t
    return (float(thermal_margin), float(t_out), float(tc_t), float(sleeve_r))