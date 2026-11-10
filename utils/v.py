from pathlib import Path
import yaml

_COEFFS_FILE = Path(__file__).resolve().parent.parent / "radiator.yaml"
with open(_COEFFS_FILE) as f:
    coeffs = yaml.safe_load(f)


def getv(cfm, rad_type):
    if rad_type == "kart":
        sweep = coeffs["kart_radiator"]["airflow_sweep"]
        return sweep["a"] * (cfm**2) + sweep["b"] * cfm + sweep["c"]
    elif rad_type == "pc":
        sweep = coeffs["pc_radiator"]["airflow_sweep"]
        return sweep["a"] * (cfm**2) + sweep["b"] * cfm + sweep["c"]
