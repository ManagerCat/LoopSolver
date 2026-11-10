r"""
Kart Radiator Thermal Model (kart_radiator.py)

Predicts radiator thermal resistance $R_{\text{rad}}$ and solves coolant outlet temperatures.

Governing Equation:
    $$\ln(R) = c_0 + c_v \ln(v) + c_Q \ln(Q) + c_L \ln(L)
             + c_{v^2} \ln^2(v) + c_{Q^2} \ln^2(Q) + c_{L^2} \ln^2(L)
             + c_{vQ} \ln(v)\ln(Q) + c_{vL} \ln(v)\ln(L) + c_{QL} \ln(Q)\ln(L)$$

    $$R = \exp\left(\ln(R)\right)$$
"""

from pathlib import Path
import math
import yaml

from utils.cp import calculate_cp_water
from utils.v import getv

_COEFFS_FILE = Path(__file__).parent / "radiator.yaml"
with open(_COEFFS_FILE) as f:
    KART_RAD_COEFFS = yaml.safe_load(f)["kart_radiator"]

_C_INTERCEPT = KART_RAD_COEFFS["intercept"]
_C_LV = KART_RAD_COEFFS["lv"]
_C_LQ = KART_RAD_COEFFS["lq"]
_C_LL = KART_RAD_COEFFS["ll"]
_C_LV2 = KART_RAD_COEFFS["lv2"]
_C_LQ2 = KART_RAD_COEFFS["lq2"]
_C_LL2 = KART_RAD_COEFFS["ll2"]
_C_LV_LQ = KART_RAD_COEFFS["lv_lq"]
_C_LV_LL = KART_RAD_COEFFS["lv_ll"]
_C_LQ_LL = KART_RAD_COEFFS["lq_ll"]

_THERMALS_FILE = Path(__file__).parent / "thermals.yaml"
with open(_THERMALS_FILE) as f:
    THERMALS = yaml.safe_load(f)
t_amb = THERMALS["ambient"]["t"]
_KG_PER_S_PER_LPM = float(THERMALS["coolant"]["rho"]) / 60000.0  # mdot = rho * LPM / 60000


def predict_radiator_thermal_resistance(cfm, Q, L, coeffs=KART_RAD_COEFFS):
    r"""
    Calculates custom kart radiator thermal resistance $R$ in $\text{K/W}$.

    Governing Equation:
        $$\ln(R) = c_0 + c_v \ln(v) + c_Q \ln(Q) + c_L \ln(L)
                 + c_{v^2} \ln^2(v) + c_{Q^2} \ln^2(Q) + c_{L^2} \ln^2(L)
                 + c_{vQ} \ln(v)\ln(Q) + c_{vL} \ln(v)\ln(L) + c_{QL} \ln(Q)\ln(L)$$

        $$R = \exp\left(\ln(R)\right)$$

    Valid ranges: $L \in [6, 10]\,\text{in}$, $Q \in [5, 9]\,\text{LPM}$, $v \in [5, 9]\,\text{m/s}$.
    """
    v = getv(cfm, "kart")
    lv, lq, ll = math.log(float(v)), math.log(float(Q)), math.log(float(L))
    if coeffs is KART_RAD_COEFFS:
        ln_R = (
            _C_INTERCEPT
            + _C_LV * lv
            + _C_LQ * lq
            + _C_LL * ll
            + _C_LV2 * (lv**2)
            + _C_LQ2 * (lq**2)
            + _C_LL2 * (ll**2)
            + _C_LV_LQ * (lv * lq)
            + _C_LV_LL * (lv * ll)
            + _C_LQ_LL * (lq * ll)
        )
    else:
        ln_R = (
            coeffs["intercept"]
            + coeffs["lv"] * lv
            + coeffs["lq"] * lq
            + coeffs["ll"] * ll
            + coeffs["lv2"] * (lv**2)
            + coeffs["lq2"] * (lq**2)
            + coeffs["ll2"] * (ll**2)
            + coeffs["lv_lq"] * (lv * lq)
            + coeffs["lv_ll"] * (lv * ll)
            + coeffs["lq_ll"] * (lq * ll)
        )
    return float(math.exp(ln_R))


def solve_radiator(t_in, lpm, L, cfm, spec_heat=None):
    r"""
    Solves radiator thermal resistance and coolant outlet temperature.

    Governing Equations:
        $$T_{\text{out}} = T_{\text{amb}} + (T_{\text{in}} - T_{\text{amb}}) \exp\left(-\frac{1}{\dot{m} \cdot C_p \cdot R}\right), \quad \dot{m} = \rho \cdot \text{LPM} / 60000$$

    Parameters
    ----------
    t_in : float
        Inlet water temperature ($^\circ\text{C}$).
    lpm : float
        Coolant volumetric flow rate ($\text{LPM}$).
    L : float
        Core length ($\text{inches}$).
    cfm : float
        Air volumetric flow rate ($\text{CFM}$).
    spec_heat : float, optional
        Specific heat capacity of water ($\text{J/(kg}\cdot\text{K)}$). If None, calculated at $t_{\text{in}}$.

    Returns
    -------
    t_out : float
        Outlet water temperature ($^\circ\text{C}$).
    r : float
        Radiator thermal resistance ($\text{K/W}$).
    """
    if spec_heat is None:
        spec_heat = calculate_cp_water(t_in)
    r = predict_radiator_thermal_resistance(cfm, lpm, L)
    t_out = t_amb + (t_in - t_amb) * math.exp(-1 / (((_KG_PER_S_PER_LPM * lpm) * spec_heat) * r))
    return t_out, r


if __name__ == "__main__":
    # Test within valid envelope: L = 6 to 10 in
    r_val = predict_radiator_thermal_resistance(cfm=17, Q=7.0, L=8.0)
    print(f"R at L=8in, Q=7LPM: {r_val:.5f} K/W")

    t_out, r_out = solve_radiator(t_in=55.0, lpm=7.0, L=8.0, cfm=17)
    print(f"T_out: {t_out:.2f} degC, R: {r_out:.5f} K/W")