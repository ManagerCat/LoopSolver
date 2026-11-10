r"""
Module 3: Hydraulic Equilibrium Solver (hydraulics.py)

Purpose / Significance:
    Dynamically calculates total loop restriction as core size scales, and solves
    the exact closed-loop operating flow rate ($\text{LPM}^*$) where the pump curve
    intersects the system loss curve.

Governing Equations:
    1. Pump Head Curve:
       $$\Delta P_{\text{pump}} = a_2 \cdot (\text{LPM})^2 + a_1 \cdot \text{LPM} + a_0$$

    2. Base Chassis Loss Curve:
       $$\Delta P_{\text{chassis}} = K_{2,\text{chassis}} \cdot (\text{LPM})^2 + K_{1,\text{chassis}} \cdot \text{LPM}$$

    3. Radiator Hydraulic Restriction:
       - PC Core (1-Fan vs 2-Fan):
         $$K_{\text{1-fan}} = \frac{K_{\text{2-fan}}}{1.5}$$
       - Custom Kart Core (linearly scaled by height $H$):
         $$K_{\text{rad}}(H) = K_{\text{rad, ref}} \cdot \left(\frac{H_{\text{ref}}}{H}\right)$$
       - Radiator Loss Curve:
         $$\Delta P_{\text{rad}} = a_{\text{rad}} \cdot (\text{LPM})^2 + b_{\text{rad}} \cdot \text{LPM}$$

    4. Total Loop Impedance:
       $$K_{2,\text{total}} = K_{2,\text{chassis}} + a_{\text{rad}}$$
       $$K_{1,\text{total}} = K_{1,\text{chassis}} + b_{\text{rad}}$$
       $$\Delta P_{\text{loop}} = K_{2,\text{total}} \cdot (\text{LPM})^2 + K_{1,\text{total}} \cdot \text{LPM}$$

    5. Quadratic Equilibrium Intersection ($\Delta P_{\text{pump}} = \Delta P_{\text{loop}}$):
       $$A = a_2 - K_{2,\text{total}}, \quad B = a_1 - K_{1,\text{total}}, \quad C = a_0$$
       $$A \cdot (\text{LPM}^*)^2 + B \cdot \text{LPM}^* + C = 0$$
       $$\text{LPM}^* = \frac{-B - \sqrt{B^2 - 4AC}}{2A}$$
       $$\Delta P_{\text{operating}} = K_{2,\text{total}} \cdot (\text{LPM}^*)^2 + K_{1,\text{total}} \cdot \text{LPM}^*$$

    6. Series-Parallel Motor Loop (4_Motor_Parallel: 2 banks in parallel, 2 motors in series each):
       $$\Delta P_{\text{main}}(Q) = \Delta P_{\text{rad}}(Q) + \Delta P_{\text{main\_plumbing}}(Q) + \Delta P_{\text{tees}}(Q)$$
       $$\Delta P_{\text{parallel\_stage}}(Q) = 2 \cdot \Delta P_{\text{motor}}(Q/2) + \Delta P_{\text{branch\_lines}}(Q/2)$$
       $$\Delta P_{\text{sys}}(Q) = \Delta P_{\text{main}}(Q) + \Delta P_{\text{parallel\_stage}}(Q)$$
       All terms are $k_2 Q^2 + k_1 Q$, so $\Delta P_{\text{pump}} = \Delta P_{\text{sys}}$ is solved exactly by (5).
       $\Delta P_{\text{tees}}$ is currently neglected.
"""

from pathlib import Path
import math
import yaml

# Load component and pump catalogs
_COMPONENTS_FILE = Path(__file__).parent / "components.yaml"
with open(_COMPONENTS_FILE) as f:
    COMPONENTS = yaml.safe_load(f)

_PUMPS_FILE = Path(__file__).parent / "pumps.yaml"
with open(_PUMPS_FILE) as f:
    PUMPS = yaml.safe_load(f)




_CHASSIS_CACHE = {}
_PARALLEL_CHASSIS_CACHE = {}


def get_chassis_impedance(config, tubing_type="thick", components=COMPONENTS):
    r"""
    Computes base chassis loop restriction coefficients $(K_{2,\text{chassis}}, K_{1,\text{chassis}})$
    excluding the radiator core, from a loop configuration dictionary or YAML path.

    Parameters
    ----------
    config : dict or str or Path
        Loop configuration dictionary (or path to config.yaml) containing component counts:
        'sleeves', 'coldplates', 'qds', 'barbs', 'tubing', '90s'.
    tubing_type : str
        Either 'thick' or 'thin'.
    components : dict
        Loaded components catalog from components.yaml.

    Returns
    -------
    tuple[float, float]
        $(K_{2,\text{chassis}}, K_{1,\text{chassis}})$ where:
        $$\Delta P_{\text{chassis}} = K_{2,\text{chassis}} \cdot (\text{LPM})^2 + K_{1,\text{chassis}} \cdot \text{LPM}$$
    """
    use_cache = (components is COMPONENTS) and isinstance(config, dict)
    if use_cache:
        cache_key = (id(config), tubing_type)
        cached = _CHASSIS_CACHE.get(cache_key)
        if cached is not None:
            return cached

    if isinstance(config, (str, Path)):
        with open(config) as f:
            config = yaml.safe_load(f)

    if tubing_type not in ("thick", "thin"):
        raise ValueError(f"Unsupported tubing_type '{tubing_type}'. Expected 'thick' or 'thin'.")

    part_counts = (
        ("sleeve", config.get("sleeves", 0)),
        ("coldplate", config.get("coldplates", 0)),
        (f"qd_{tubing_type}", config.get("qds", 0)),
        (f"barb_{tubing_type}", config.get("barbs", 0)),
        (f"tubing_{tubing_type}", config.get("tubing", 0)),
        (f"90_{tubing_type}", config.get("90s", 0)),
    )

    k2_chassis = 0.0
    k1_chassis = 0.0
    for part_key, count in part_counts:
        part = components[part_key]
        k2_chassis += part["a"] * count
        k1_chassis += part["b"] * count

    res = (k2_chassis, k1_chassis)
    if use_cache:
        _CHASSIS_CACHE[cache_key] = res
    return res


def get_parallel_loop_impedance(
    config,
    tubing_type="thick",
    num_branches=2,
    components=COMPONENTS,
):
    r"""
    Splits the series-parallel motor loop (``4_Motor_Parallel``) into its main-loop
    and per-branch impedance terms, each referenced to the flow it actually sees.

    Topology (symmetric 2x2 series-parallel):
    - Main loop at $Q_{\text{total}}$: Pump -> Radiator -> Supply Tee ... Return Tee -> Pump
    - $N$ identical branch legs at $Q_{\text{branch}} = Q_{\text{total}} / N$, each containing
      $\text{sleeves} / N$ motors in series plus its own branch lines.

    Component split:
    - Main plumbing (at $Q_{\text{total}}$): radiator barbs ($2 \times \text{radiators}$),
      plus any main-trunk coldplates/QDs. The main trunk carries no tubing.
    - Tees (at $Q_{\text{total}}$): currently neglected ($\Delta P_{\text{tees}} = 0$).
    - Motor (per motor, at $Q_{\text{branch}}$): one sleeve.
    - Branch lines (per branch, at $Q_{\text{branch}}$): $(\text{barbs} - 2 \cdot \text{radiators}) / N$
      barbs, $\text{tubing} / N$ tubing length, $\text{90s} / N$ elbows.

    Returns
    -------
    dict
        {
          "main_plumbing": (k2, k1),      # at Q_total
          "tees": (k2, k1),               # at Q_total
          "motor": (k2, k1),              # single motor, at Q_branch
          "motors_per_branch": float,
          "branch_lines": (k2, k1),       # one branch, at Q_branch
          "num_branches": int,
        }
        where every term follows $\Delta P = k_2 Q^2 + k_1 Q$.
    """
    use_cache = (components is COMPONENTS) and isinstance(config, dict)
    if use_cache:
        cache_key = (id(config), tubing_type, num_branches)
        cached = _PARALLEL_CHASSIS_CACHE.get(cache_key)
        if cached is not None:
            return cached

    if isinstance(config, (str, Path)):
        with open(config) as f:
            config = yaml.safe_load(f)

    if tubing_type not in ("thick", "thin"):
        raise ValueError(f"Unsupported tubing_type '{tubing_type}'. Expected 'thick' or 'thin'.")

    def _sum(counts):
        k2 = 0.0
        k1 = 0.0
        for part_key, count in counts:
            part = components[part_key]
            k2 += part["a"] * count
            k1 += part["b"] * count
        return (k2, k1)

    num_radiators = config.get("radiators", 1)
    total_barbs = config.get("barbs", 0)
    rad_barbs = 2 * num_radiators
    branch_barbs_total = max(0, total_barbs - rad_barbs)

    # Main loop plumbing (sees full flow Q_total)
    main_plumbing = _sum((
        ("coldplate", config.get("coldplates", 0)),
        (f"qd_{tubing_type}", config.get("qds", 0)),
        (f"barb_{tubing_type}", rad_barbs),
    ))

    # Supply splitter + return collector tees (neglected for now)
    tees = (0.0, 0.0)

    # Single motor (sleeve), sees Q_branch
    motor = _sum((("sleeve", 1),))
    motors_per_branch = config.get("sleeves", 0) / num_branches

    # One branch's lines (sees Q_branch)
    branch_lines = _sum((
        (f"barb_{tubing_type}", branch_barbs_total / num_branches),
        (f"tubing_{tubing_type}", config.get("tubing", 0) / num_branches),
        (f"90_{tubing_type}", config.get("90s", 0) / num_branches),
    ))

    res = {
        "main_plumbing": main_plumbing,
        "tees": tees,
        "motor": motor,
        "motors_per_branch": motors_per_branch,
        "branch_lines": branch_lines,
        "num_branches": num_branches,
    }
    if use_cache:
        _PARALLEL_CHASSIS_CACHE[cache_key] = res
    return res


def _dp_quad(coeffs, q):
    r"""$\Delta P = k_2 Q^2 + k_1 Q$"""
    k2, k1 = coeffs
    return k2 * q * q + k1 * q


def dp_main(q_total, loop, rad_coeffs):
    r"""
    Main loop loss at $Q_{\text{total}}$:
    $$\Delta P_{\text{main}} = \Delta P_{\text{rad}} + \Delta P_{\text{main\_plumbing}} + \Delta P_{\text{tees}}$$
    """
    return (
        _dp_quad(rad_coeffs, q_total)
        + _dp_quad(loop["main_plumbing"], q_total)
        + _dp_quad(loop["tees"], q_total)
    )


def dp_motor_pair(q_branch, loop):
    r"""
    Motors in series in one branch:
    $$\Delta P_{\text{motor\_pair}}(Q_{\text{branch}}) = n \cdot \Delta P_{\text{motor}}(Q_{\text{branch}}), \quad n = 2$$
    """
    return loop["motors_per_branch"] * _dp_quad(loop["motor"], q_branch)


def dp_parallel_stage(q_total, loop):
    r"""
    Parallel stage loss (equal to the loss through one branch):
    $$\Delta P_{\text{parallel\_stage}}(Q_{\text{total}}) = \Delta P_{\text{motor\_pair}}\left(\frac{Q_{\text{total}}}{N}\right)
      + \Delta P_{\text{branch\_lines}}\left(\frac{Q_{\text{total}}}{N}\right)$$
    """
    q_branch = q_total / loop["num_branches"]
    return dp_motor_pair(q_branch, loop) + _dp_quad(loop["branch_lines"], q_branch)


def dp_sys_parallel(q_total, loop, rad_coeffs):
    r"""
    Total system loop resistance:
    $$\Delta P_{\text{sys}}(Q_{\text{total}}) = \Delta P_{\text{main}}(Q_{\text{total}}) + \Delta P_{\text{parallel\_stage}}(Q_{\text{total}})$$
    """
    return dp_main(q_total, loop, rad_coeffs) + dp_parallel_stage(q_total, loop)


def get_parallel_chassis_impedance(
    config,
    tubing_type="thick",
    num_branches=2,
    components=COMPONENTS,
):
    r"""
    Effective (non-radiator) loop coefficients $(K_{2,\text{chassis}}, K_{1,\text{chassis}})$
    referenced to $Q_{\text{total}}$ for the series-parallel loop, such that
    $$\Delta P_{\text{sys}}(Q) - \Delta P_{\text{rad}}(Q) = K_{2,\text{chassis}} Q^2 + K_{1,\text{chassis}} Q$$

    Because $Q_{\text{branch}} = Q_{\text{total}} / N$, branch quadratic terms scale by $1/N^2$
    and linear terms by $1/N$. Exact, since every term is of the form $k_2 Q^2 + k_1 Q$.
    """
    loop = get_parallel_loop_impedance(config, tubing_type, num_branches, components)
    n = loop["num_branches"]
    m = loop["motors_per_branch"]
    k2_main = loop["main_plumbing"][0] + loop["tees"][0]
    k1_main = loop["main_plumbing"][1] + loop["tees"][1]
    k2_branch = m * loop["motor"][0] + loop["branch_lines"][0]
    k1_branch = m * loop["motor"][1] + loop["branch_lines"][1]
    return (k2_main + k2_branch / (n ** 2), k1_main + k1_branch / n)


def get_pump_coefficients(pump_name, num_pumps=1, pumps=PUMPS):
    r"""
    Returns $[a_2, a_1, a_0]$ pump curve coefficients for:
    $$\Delta P_{\text{pump}} = a_2 \cdot (\text{LPM})^2 + a_1 \cdot \text{LPM} + a_0$$
    """
    if isinstance(pump_name, (list, tuple)):
        return [
            sum(pumps[p]["a"] for p in pump_name),
            sum(pumps[p]["b"] for p in pump_name),
            sum(pumps[p]["c"] for p in pump_name),
        ]
    pump = pumps[pump_name]
    return [
        pump["a"] * num_pumps,
        pump["b"] * num_pumps,
        pump["c"] * num_pumps,
    ]


def get_radiator_impedance(
    rad_type="pc",
    fan_scale=2,
    H=None,
    H_ref=10,
    k_rad_ref=None,
    num_radiators=1,
    components=COMPONENTS,
):
    r"""
    Calculates the radiator hydraulic impedance coefficients $(a_{\text{rad}}, b_{\text{rad}})$.

    Parameters
    ----------
    rad_type : str
        'pc' (discrete 1-fan or 2-fan PC radiator) or 'kart' (custom kart core scaled by height $H$).
    fan_scale : int or str
        For PC core: 1 (or '1-fan') vs 2 (or '2-fan'), where:
        $$K_{\text{1-fan}} = \frac{K_{\text{2-fan}}}{1.5}$$
    H : float, optional
        Candidate core height $H$ (inches) for custom Kart radiator.
    H_ref : float, optional
        Reference core height $H_{\text{ref}}$ (inches) for custom Kart radiator. Defaults to 10 (or kart_rad.H_ref in components.yaml).
    k_rad_ref : float, optional
        Reference restriction coefficient $K_{\text{rad, ref}}$ for custom Kart radiator. Defaults to kart_rad.a in components.yaml.
    num_radiators : int
        Number of radiators in series (defaults to 1).
    components : dict
        Loaded components catalog from components.yaml.

    Returns
    -------
    tuple[float, float]
        $(a_{\text{rad}}, b_{\text{rad}})$ where:
        $$\Delta P_{\text{rad}} = a_{\text{rad}} \cdot (\text{LPM})^2 + b_{\text{rad}} \cdot \text{LPM}$$
    """
    rad_key = rad_type.lower().strip()

    if rad_key in ("pc", "pc_rad"):
        k2_2fan = components["pc_rad"]["a"]
        k1_2fan = components["pc_rad"].get("b", 0.0)

        if fan_scale in (1, "1-fan", "1_fan", "1fan"):
            a_rad = k2_2fan / 1.5
            b_rad = k1_2fan / 1.5
        elif fan_scale in (2, "2-fan", "2_fan", "2fan"):
            a_rad = k2_2fan
            b_rad = k1_2fan
        else:
            raise ValueError(f"Unsupported fan_scale '{fan_scale}' for PC radiator. Expected 1 or 2.")

    elif rad_key in ("kart", "kart_rad", "custom"):
        if H is None or H <= 0:
            raise ValueError(f"Candidate height H must be a positive number for Kart radiator (got {H}).")

        kart_cfg = components.get("kart_rad", components["pc_rad"])
        if k_rad_ref is None:
            k_rad_ref = kart_cfg["a"]
        if H_ref is None:
            H_ref = kart_cfg.get("H_ref", 10)

        # Linear scaling with core height: $K_{\text{rad}}(H) = K_{\text{rad, ref}} \cdot \left(\frac{H_{\text{ref}}}{H}\right)$
        a_rad = k_rad_ref * (H_ref / H)
        b_rad = kart_cfg.get("b", 0.0) * (H_ref / H)

    else:
        raise ValueError(f"Unknown rad_type '{rad_type}'. Expected 'pc' or 'kart'.")

    return a_rad * num_radiators, b_rad * num_radiators


def solve_hydraulic_equilibrium(
    pump_coeffs,
    chassis_coeffs,
    rad_type="pc",
    fan_scale=2,
    H=None,
    H_ref=10,
    k_rad_ref=None,
    num_radiators=1,
    components=COMPONENTS,
):
    r"""
    Dynamically calculates total loop restriction as core size scales, and solves
    the exact closed-loop operating flow rate ($\text{LPM}^*$) where the pump curve intersects
    the system loss curve.

    Parameters
    ----------
    pump_coeffs : sequence[float] or dict or str
        Pump curve coefficients $[a_2, a_1, a_0]$ for:
        $$\Delta P_{\text{pump}} = a_2 \cdot (\text{LPM})^2 + a_1 \cdot \text{LPM} + a_0$$
        or a dict `{'a': a2, 'b': a1, 'c': a0}`, or a pump name key from pumps.yaml.
    chassis_coeffs : tuple[float, float]
        Base chassis loop coefficients $(K_{2,\text{chassis}}, K_{1,\text{chassis}})$.
    rad_type : str
        'pc' or 'kart'.
    fan_scale : int or str
        For PC core: 1 ($K_{\text{1-fan}} = K_{\text{2-fan}} / 1.5$) or 2 (2-Fan).
    H : float, optional
        For Custom Kart core: candidate height $H$.
    H_ref : float, optional
        Reference height $H_{\text{ref}}$ for Kart core scaling.
    k_rad_ref : float, optional
        Reference restriction $K_{\text{rad, ref}}$ for Kart core scaling.
    num_radiators : int
        Number of radiators in the loop (defaults to 1).
    components : dict
        Loaded components catalog.

    Returns
    -------
    tuple[float, float]
        (lpm_star, dp_operating)
        - lpm_star: operating flow rate $\text{LPM}^*$ in $\text{LPM}$
        - dp_operating: system pressure drop $\Delta P_{\text{operating}}$ at operating point in $\text{kPa}$
    """
    if isinstance(pump_coeffs, str):
        a2, a1, a0 = get_pump_coefficients(pump_coeffs)
    elif isinstance(pump_coeffs, dict):
        a2, a1, a0 = pump_coeffs["a"], pump_coeffs["b"], pump_coeffs["c"]
    else:
        a2, a1, a0 = pump_coeffs

    k2_chassis, k1_chassis = chassis_coeffs

    # Radiator impedance
    a_rad, b_rad = get_radiator_impedance(
        rad_type=rad_type,
        fan_scale=fan_scale,
        H=H,
        H_ref=H_ref,
        k_rad_ref=k_rad_ref,
        num_radiators=num_radiators,
        components=components,
    )

    # Total Loop Impedance:
    # $$K_{2,\text{total}} = K_{2,\text{chassis}} + a_{\text{rad}}, \quad K_{1,\text{total}} = K_{1,\text{chassis}} + b_{\text{rad}}$$
    k2_total = k2_chassis + a_rad
    k1_total = k1_chassis + b_rad

    # Quadratic Intersection ($\Delta P_{\text{pump}} = \Delta P_{\text{loop}}$):
    # $$A = a_2 - K_{2,\text{total}}, \quad B = a_1 - K_{1,\text{total}}, \quad C = a_0$$
    # $$\text{LPM}^* = \frac{-B - \sqrt{B^2 - 4AC}}{2A}$$
    A = a2 - k2_total
    B = a1 - k1_total
    C = a0

    discriminant = B**2 - 4.0 * A * C
    if discriminant < 0:
        raise ValueError("Pump curve and system loss curve do not intersect (negative discriminant).")

    lpm_star = (-B - math.sqrt(discriminant)) / (2.0 * A)
    if lpm_star <= 0:
        raise ValueError(f"No positive operating flow rate found (LPM* = {lpm_star:.4f}).")

    dp_operating = k2_total * (lpm_star**2) + k1_total * lpm_star

    return lpm_star, dp_operating


def solve_hydraulics(
    config,
    tubing_type,
    radiator_type,
    L=None,
    pump="ta60",
    components=COMPONENTS,
    pumps=PUMPS,
):
    r"""
    Master hydraulics function to solve the closed-loop operating flow rate
    and system pressure drop for a given loop configuration.

    Parameters
    ----------
    config : dict or str or Path
        Loop configuration dictionary or path to a YAML configuration file.
        Contains component counts (sleeves, coldplates, qds, barbs, tubing, 90s,
        radiators, pumps).
    tubing_type : str
        'thick' or 'thin'.
    radiator_type : str
        'kart' (custom kart core scaled by length/height $L$) or 'pc' (discrete PC core).
    L : float or int, optional
        Radiator geometry parameter:
        - For 'kart': core length / height in inches (defaults to 10 in if None).
        - For 'pc': fan count (1 or 2, defaults to 2 if None).
    pump : str
        Pump model name from pumps.yaml (defaults to 'ta60').
    components : dict, optional
        Loaded components catalog (defaults to COMPONENTS).
    pumps : dict, optional
        Loaded pumps catalog (defaults to PUMPS).

    Returns
    -------
    dict
        {"lpm": lpm_star, "dp": dp_operating}
        - lpm: Equilibrium flow rate in LPM (L/min)
        - dp: System pressure drop at operating flow rate in kPa
    """
    if isinstance(config, (str, Path)):
        with open(config) as f:
            config = yaml.safe_load(f)

    # Component counts for pumps and radiators
    num_pumps = config.get("pumps", 1)
    num_radiators = config.get("radiators", 1)

    # Chassis restriction coefficients
    chassis_coeffs = get_chassis_impedance(
        config=config,
        tubing_type=tubing_type,
        components=components,
    )

    # Pump curve coefficients scaled by number of pumps
    pump_coeffs = get_pump_coefficients(
        pump_name=pump,
        num_pumps=num_pumps,
        pumps=pumps,
    )

    # Radiator impedance handling
    rad_key = radiator_type.lower().strip()
    if rad_key in ("pc", "pc_rad"):
        fan_scale = 2 if L is None else L
        H = None
    elif rad_key in ("kart", "kart_rad", "custom"):
        H = 10 if L is None else L
        fan_scale = 2
    else:
        raise ValueError(f"Unknown radiator_type '{radiator_type}'. Expected 'pc' or 'kart'.")

    # Solve operating flow rate and pressure drop
    lpm_star, dp_operating = solve_hydraulic_equilibrium(
        pump_coeffs=pump_coeffs,
        chassis_coeffs=chassis_coeffs,
        rad_type=rad_key,
        fan_scale=fan_scale,
        H=H,
        num_radiators=num_radiators,
        components=components,
    )

    return {"lpm": lpm_star, "dp": dp_operating}


def solve_hydraulics_parallel(
    config,
    tubing_type,
    radiator_type,
    L=None,
    pump="ta60",
    num_branches=2,
    components=COMPONENTS,
    pumps=PUMPS,
):
    r"""
    Solves the closed-loop operating flow rates (at the radiator and at each motor branch)
    and system pressure drop for the parallel motor loop configuration.

    Parameters
    ----------
    config : dict or str or Path
        Loop configuration dictionary or path to a YAML configuration file.
    tubing_type : str
        'thick' or 'thin'.
    radiator_type : str
        'kart' (custom kart core scaled by length/height $L$) or 'pc' (discrete PC core).
    L : float or int, optional
        Radiator geometry parameter:
        - For 'kart': core length / height in inches (defaults to 10 in if None).
        - For 'pc': fan count (1 or 2, defaults to 2 if None).
    pump : str
        Pump model name from pumps.yaml (defaults to 'ta60').
    num_branches : int
        Number of parallel motor branches (defaults to 2).
    components : dict, optional
        Loaded components catalog (defaults to COMPONENTS).
    pumps : dict, optional
        Loaded pumps catalog (defaults to PUMPS).

    Returns
    -------
    dict
        {"lpm_rad": lpm_rad, "lpm_motor": lpm_motor, "dp": dp_operating}
        - lpm_rad: Equilibrium flow rate at the radiator/pump in LPM (L/min)
        - lpm_motor: Equilibrium flow rate through each motor branch in LPM (L/min)
        - dp: Total loop pressure drop at operating point in kPa
    """
    if isinstance(config, (str, Path)):
        with open(config) as f:
            config = yaml.safe_load(f)

    # Component counts for pumps and radiators
    num_pumps = config.get("pumps", 1)
    num_radiators = config.get("radiators", 1)

    # Main loop / branch impedance split for the series-parallel loop
    loop = get_parallel_loop_impedance(
        config=config,
        tubing_type=tubing_type,
        num_branches=num_branches,
        components=components,
    )
    # Effective non-radiator coefficients referenced to Q_total (exact reduction of
    # dP_main + dP_parallel_stage, since all terms are k2*Q^2 + k1*Q)
    chassis_coeffs = get_parallel_chassis_impedance(
        config=config,
        tubing_type=tubing_type,
        num_branches=num_branches,
        components=components,
    )

    # Pump curve coefficients scaled by number of pumps
    pump_coeffs = get_pump_coefficients(
        pump_name=pump,
        num_pumps=num_pumps,
        pumps=pumps,
    )

    # Radiator impedance handling
    rad_key = radiator_type.lower().strip()
    if rad_key in ("pc", "pc_rad"):
        fan_scale = 2 if L is None else L
        H = None
    elif rad_key in ("kart", "kart_rad", "custom"):
        H = 10 if L is None else L
        fan_scale = 2
    else:
        raise ValueError(f"Unknown radiator_type '{radiator_type}'. Expected 'pc' or 'kart'.")

    # Solve Q_total* where dP_pump(Q) = dP_sys(Q) (closed-form quadratic intersection)
    lpm_rad, _ = solve_hydraulic_equilibrium(
        pump_coeffs=pump_coeffs,
        chassis_coeffs=chassis_coeffs,
        rad_type=rad_key,
        fan_scale=fan_scale,
        H=H,
        num_radiators=num_radiators,
        components=components,
    )

    # Evaluate the explicit system curve at the operating point and verify the intercept
    rad_coeffs = get_radiator_impedance(
        rad_type=rad_key,
        fan_scale=fan_scale,
        H=H,
        num_radiators=num_radiators,
        components=components,
    )
    dp_main_op = dp_main(lpm_rad, loop, rad_coeffs)
    dp_stage_op = dp_parallel_stage(lpm_rad, loop)
    dp_operating = dp_main_op + dp_stage_op
    a2, a1, a0 = pump_coeffs
    dp_pump_op = a2 * lpm_rad ** 2 + a1 * lpm_rad + a0
    residual = dp_pump_op - dp_operating
    assert abs(residual) <= 1e-9 * max(1.0, abs(dp_pump_op)), (
        f"Parallel operating point residual too large: dP_pump - dP_sys = {residual:.3e} kPa"
    )

    lpm_motor = lpm_rad / num_branches

    return {
        "lpm_rad": lpm_rad,          # Q_total*
        "lpm_motor": lpm_motor,      # Q_branch* = Q_total* / N
        "dp": dp_operating,          # dP_sys(Q_total*)
        "dp_main": dp_main_op,
        "dp_parallel_stage": dp_stage_op,
    }



