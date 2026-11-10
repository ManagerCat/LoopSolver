# LoopSolver: Powertrain & Electronics Liquid Cooling Solver

LoopSolver is a coupled hydraulic and thermal simulation framework designed to evaluate, optimize, and size liquid cooling systems for high-performance electric vehicles (EVs, electric karts, and Formula Student/SAE prototypes).

The solver models a 4-wheel independent motor powertrain (Front-Left, Front-Right, Rear-Left, Rear-Right) along with power electronics/inverter coldplates. It calculates exact closed-loop hydraulic operating points ($\text{LPM}^*$), evaluates steady-state temperatures, sizes radiators, and balances thermal margins against loop mass and electrical power consumption.

---

## Table of Contents

- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Repository Structure](#repository-structure)
- [Installation & Quick Start](#installation--quick-start)
- [CLI Tools & Usage](#cli-tools--usage)
  - [1. Permutation Orchestrator (`orchestrator.py`)](#1-permutation-orchestrator-orchestratorpy)
  - [2. Series vs. Parallel Analysis (`svp.py`)](#2-series-vs-parallel-analysis-svppy)
  - [3. Single-Loop Heat Capacity Scaling (`single.py`)](#3-single-loop-heat-capacity-scaling-singlepy)
- [Configuration Files](#configuration-files)
- [Governing Physics & Models](#governing-physics--models)
  - [Hydraulic Equilibrium Solver](#hydraulic-equilibrium-solver)
  - [Motor Convective Sleeve Model](#motor-convective-sleeve-model)
  - [Coldplate Thermal Model](#coldplate-thermal-model)
  - [Radiator Performance Models](#radiator-performance-models)
- [Customizing Output Columns](#customizing-output-columns)

---

## Key Features

- **Coupled Hydraulic-Thermal Equilibrium**: Solves the intersection of non-linear pump pressure curves with loop hydraulic restriction curves, propagating the exact flow rate into convective thermal models.
- **Multiple Cooling Loop Architectures**: Out-of-the-box support for unified single loops, left-right splits, 4-motor series, and 4-motor parallel configurations.
- **Automated Design Space Sweeps**: Evaluates thousands of hardware permutations across pumps, fans, tubing gauges, and radiators, with automatic binary-search sizing for custom kart radiator lengths.
- **Granular Mass & Power Budgeting**: Tracks component-level weight (tubing, barbs, QDs, closeouts, radiators, fans, pumps) and electrical power draw for all passing configurations.
- **Detailed Component Performance**: Models Martin & Norris annular sleeve convection, Reynolds/Prandtl numbers, temperature-dependent water heat capacity ($C_p$), and multi-variable radiator thermal resistance response surfaces.

---

## System Architecture

The solver models five primary cooling loop architectures:

| Architecture | Description | Separate Coldplate Loop? | Loop Multiplier |
|---|---|:---:|:---:|
| `Single` | All 4 motors, coldplate, and dual radiators in a single unified loop | No | 1× |
| `Left_Right_Split` | Symmetrical left and right chassis loops (Left: FL + RL + half coldplate; Right: FR + RR + half coldplate) | No | 2× |
| `4_Motor_Parallel` | 2 parallel motor banks (RL+RR and FL+FR) with a shared radiator/pump | Yes | 1× |
| `4_Motor_Series_Counterclockwise` | All 4 motors in series (RL $\rightarrow$ RR $\rightarrow$ FR $\rightarrow$ FL) | Yes | 1× |
| `4_Motor_Series_Clockwise` | All 4 motors in series (FL $\rightarrow$ FR $\rightarrow$ RR $\rightarrow$ RL) | Yes | 1× |

---

## Repository Structure

```text
loopSolver/
├── orchestrator.py        # Automated permutation sweep & optimization engine
├── single.py              # Max proportional heat scaling tool for single loops
├── svp.py                 # Series vs. Parallel detailed comparison tool
├── architectures.py       # Loop flow routing, energy balance & iteration solvers
├── hydraulics.py          # Pump curves, chassis impedance & hydraulic solver
├── motor.py               # Motor sleeve convection & internal core thermal model
├── coldplate.py           # Inverter/electronics coldplate thermal solver
├── kart_radiator.py       # Custom kart radiator log-polynomial response surface
├── pc_radiator.py         # Standard PC radiator thermal resistance model
│
├── utils/
│   ├── cp.py              # Temperature-dependent specific heat of water Cp(T)
│   └── v.py               # Radiator face velocity calculated from fan CFM
│
├── components.yaml        # Hydraulic loss coefficients (a, b) and component masses
├── fans.yaml              # Fan models, airflow (max CFM), power, and mass
├── loops.yaml             # Bill of materials and tubing lengths per architecture
├── pumps.yaml             # Pump head quadratic coefficients, wattage, and mass
├── radiator.yaml          # Radiator empirical regression & airflow sweep coeffs
├── thermals.yaml          # Thermal thresholds, ambient temp, and nominal heat loads
└── output.yaml            # Configurable CSV column output settings for orchestrator
```

---

## Installation & Quick Start

### Requirements
- Python 3.9+
- `PyYAML`

### Setup

```bash
# Clone or navigate to the directory
cd loopSolver

# Create and activate a virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install pyyaml
```

---

## CLI Tools & Usage

### 1. Permutation Orchestrator (`orchestrator.py`)

Sweeps every combination of architecture, pump, tubing diameter, fan, and radiator. Radiator core lengths for the custom kart radiator are automatically resolved via binary search down to 0.25-inch increments to find the minimum core size that meets thermal targets ($T_{\text{motor}} \le 120^\circ\text{C}$, $T_{\text{coldplate}} \le 65^\circ\text{C}$).

```bash
# Run standard optimization sweep with progress bar (exports to results.csv)
python orchestrator.py

# Include failing configurations in the output dataset
python orchestrator.py --include-failed

# Run without thermal targets (forces all configurations to evaluate and record)
python orchestrator.py --no-targets

# Specify a custom output path
python orchestrator.py -o custom_results.csv --no-progress
```

**Options:**
- `-nt`, `--no-targets`: Ignores thermal ceilings so all configurations pass and are logged.
- `--include-failed`: Includes configurations that exceeded temperature limits.
- `--no-progress`: Disables the terminal progress bar.
- `-o`, `--output`: Specifies output CSV file path.

---

### 2. Series vs. Parallel Analysis (`svp.py`)

Directly compares hydraulic and thermal parameters between `4_Motor_Series_Counterclockwise` and `4_Motor_Parallel` under identical hardware constraints.

```bash
# Run with nominal default parameters
python svp.py

# Compare architectures using a PC radiator (2 fans), ta60 pump, and thick tubing
python svp.py --rad pc -L 2 --pump ta60 --tubing thick

# Compare using a custom 10-inch kart radiator with high-performance fan
python svp.py --rad kart -L 10 --pump intg3-570 --fan OD1238-24HB
```

**Metrics compared:**
- Operating flow rate ($\text{LPM}^*$) at the radiator and through each motor
- Total system pressure drop ($\Delta P_{\text{sys}}$) and radiator pressure drop ($\Delta P_{\text{rad}}$)
- Convective heat transfer coefficient (HTC) and sleeve thermal resistance ($R_{\text{sleeve}}$) for each motor
- Coolant inlet/outlet temperatures and motor core temperatures

---

### 3. Single-Loop Heat Capacity Scaling (`single.py`)

Evaluates the unified `Single` loop architecture and scales all motor and coldplate heat loads proportionally until the first thermal limit is reached, determining the loop's maximum heat rejection headroom.

```bash
# Run with default dual intg3-570 pumps in series
python single.py

# Evaluate a specific series pump combination, radiator, and fan
python single.py --pumps intg3-570 intg3-570 --rad kart -L 10 --fan OD1238-24HB --tubing thick

# Evaluate single pump with PC radiator
python single.py --pumps ta60 --rad pc -L 2 --fan 12038VA-24R-GA-00 --tubing thin
```

**Options:**
- `--pumps`, `--pump`: One or more pump models placed in hydraulic series (e.g. `--pumps intg3-570 intg3-570`).
- `--rad`: Radiator type (`kart` or `pc`).
- `-L`: Core geometry length ($L$ in inches for kart, 1 or 2 fans for PC).
- `--fan`: Fan model from `fans.yaml`.
- `--tubing`: Tubing gauge (`thick` or `thin`).

---

## Configuration Files

All physical constants, component catalogs, and architecture definitions are managed via YAML files:

| File | Purpose |
|---|---|
| [`components.yaml`](components.yaml) | Quadratic hydraulic loss curves ($\Delta P = a \cdot Q^2 + b \cdot Q$) and unit masses (lbs) for quick-disconnects (QDs), barbs, tubing lengths, motor sleeves, and coldplates. |
| [`pumps.yaml`](pumps.yaml) | 2nd-order pump head curves ($\Delta P = a \cdot Q^2 + b \cdot Q + c$), dry mass (lbs), and electrical power consumption (W). |
| [`fans.yaml`](fans.yaml) | Fan specifications: maximum airflow (CFM), mass (lbs), and electrical wattage (W), including a `No_Fan_Passive` mode. |
| [`thermals.yaml`](thermals.yaml) | Ambient temperature ($T_{\text{amb}} = 30^\circ\text{C}$), thermal limits ($T_{\text{motor, max}} = 120^\circ\text{C}$, $T_{\text{cp, max}} = 65^\circ\text{C}$), internal motor conduction resistance ($R = 0.07\text{ K/W}$), and heat loads ($Q_{\text{FL}}, Q_{\text{FR}}, Q_{\text{RL}}, Q_{\text{RR}}, Q_{\text{cp}}$). |
| [`radiator.yaml`](radiator.yaml) | Response surface regression coefficients for custom kart radiators and PC radiators, alongside airflow velocity sweep polynomials ($v(\text{CFM})$). |
| [`loops.yaml`](loops.yaml) | Component bills-of-materials (BOM) and tubing run lengths for each loop architecture. |
| [`output.yaml`](output.yaml) | Toggles and reorders columns included in the generated output CSV. |

---

## Governing Physics & Models

### Hydraulic Equilibrium Solver

Chassis components, fittings, and radiators produce pressure losses following a quadratic restriction model:

$$\Delta P_{\text{loss}}(Q) = K_2 Q^2 + K_1 Q$$

Pumps deliver head pressure characterized by:

$$\Delta P_{\text{pump}}(Q) = a_2 Q^2 + a_1 Q + a_0$$

When $N$ identical or distinct pumps are installed in series:

$$\Delta P_{\text{pump, total}}(Q) = \sum_{i=1}^N (a_{2,i} Q^2 + a_{1,i} Q + a_{0,i})$$

Equilibrium operating flow rate ($\text{LPM}^*$) is resolved analytically by finding the positive root of $\Delta P_{\text{pump}}(Q) - \Delta P_{\text{loss}}(Q) = 0$:

$$A = a_{2,\text{total}} - K_2, \quad B = a_{1,\text{total}} - K_1, \quad C = a_{0,\text{total}}$$

$$\text{LPM}^* = \frac{-B - \sqrt{B^2 - 4AC}}{2A}$$

---

### Motor Convective Sleeve Model

Coolant flows through an annular jacket surrounding each motor stator. Convective heat transfer is modeled using empirical correlations for curved/annular ducts (Martin & Norris formulation):

1. **Velocity & Reynolds Number**:
   $$v = \frac{Q / 60000}{H \cdot W}, \quad \text{Re} = \frac{\rho \cdot v \cdot D_{\text{hyd}}}{\mu}$$
2. **Nusselt / Martin-Norris Correlation**:
   $$f_{\text{MN}} = \left( \frac{0.3 (D_{\text{hyd}} / D)^{0.5}}{\text{Re}^{0.2} (D_{\text{hyd}} / D)^{0.4}} \right) \left[ 1 + \frac{0.112}{\text{Re}^{0.2} (D_{\text{hyd}} / D)^{0.4}} \right]$$
   $$\text{MN} = 0.04 \cdot \left(\frac{D_{\text{hyd}}}{D}\right)^{0.077} \cdot f_{\text{MN}} \cdot \text{Re}^{1.13} \cdot \text{Pr}^{0.4}$$
3. **Convective Resistance ($R_{\text{sleeve}}$)**:
   $$\text{HTC} = \text{MN} \cdot \frac{k}{D_{\text{hyd}}}, \quad c = \text{HTC} \cdot A_{\text{effective}}, \quad R_{\text{sleeve}} = \frac{1}{c}$$
4. **Motor Core Temperature ($T_{\text{core}}$)**:
   $$T_{\text{out}} = T_{\text{in}} + \frac{Q}{\dot{m} \cdot C_p(T)}$$
   $$T_{\text{motor}} = \frac{e^{\Delta T / (Q / c)} \cdot T_{\text{out}} - T_{\text{in}}}{e^{\Delta T / (Q / c)} - 1}$$
   $$T_{\text{core}} = T_{\text{motor}} + Q \cdot R_{\text{internal}}$$

---

### Coldplate Thermal Model

The inverter coldplate thermal resistance is modeled as an empirical power law of flow rate:

$$R_{\text{coldplate}} = 0.0283 \cdot (\text{LPM})^{-0.448177}$$

$$T_{\text{coldplate}} = \frac{T_{\text{in}} + T_{\text{out}}}{2} + Q_{\text{cp}} \cdot R_{\text{coldplate}}$$

---

### Radiator Performance Models

- **PC Radiators**: Sized by discrete fan increments ($L \in \{1, 2\}$). Thermal resistance is derived from air face velocity and coolant flow rate:
  $$T_{\text{out}} = T_{\text{amb}} + (T_{\text{in}} - T_{\text{amb}}) \exp\left( -\frac{1}{\dot{m} C_p R_{\text{rad}}} \right)$$
- **Custom Kart Radiators**: Uses a multi-variable log-polynomial surface fitted over experimental and CFD sweeps:
  $$\ln(R_{\text{rad}}) = c_0 + c_v \ln(v) + c_Q \ln(Q) + c_L \ln(L) + c_{v^2}\ln^2(v) + c_{Q^2}\ln^2(Q) + c_{L^2}\ln^2(L) + \text{cross-terms}$$
  Where $v$ is face velocity ($\text{m/s}$), $Q$ is coolant flow ($\text{LPM}$), and $L$ is radiator core length ($\text{in}$).

---

## Customizing Output Columns

The columns exported by `orchestrator.py` to CSV are configurable via `output.yaml`. Enable (`true`) or disable (`false`) columns as needed:

```yaml
output_file: "results.csv"

columns:
  Architecture: true
  Radiator: true
  L: true
  Core_Area_in2: true
  Pump: true
  Pump_LPM: true
  Tubing: true
  Fan_CFM: true

  # Motor and Coldplate Temperatures (deg C)
  FL_Temp_C: true
  FR_Temp_C: true
  RL_Temp_C: true
  RR_Temp_C: true
  Coldplate_Temp_C: true

  # Power and Weight Breakdown
  Total_Power_W: true
  Total_Mass_lbs: true
  Radiator_Mass_lbs: true
  Pump_Mass_lbs: true
  Fan_Mass_lbs: true
  Tubing_Mass_lbs: true
```

