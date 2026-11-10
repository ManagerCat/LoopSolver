def calculate_cp_water(temp_c):
    """
    Calculates specific heat capacity (Cp) of liquid water at 1 atm.
    Valid range: 0°C to 100°C
    Input: Temperature in Celsius (°C)
    Output: Cp in J/(kg·K)
    """
    # Clamp to liquid water domain [0, 100] deg C so extreme failing search states do not raise
    t = 0.0 if temp_c < 0.0 else (100.0 if temp_c > 100.0 else float(temp_c))

    # Empirical polynomial coefficients
    c0 = 4217.4
    c1 = -3.7202
    c2 = 0.14125
    c3 = -0.002054
    c4 = 0.0000116

    return c0 + t * (c1 + t * (c2 + t * (c3 + t * c4)))


if __name__ == "__main__":
    # Quick test at room temp
    print(calculate_cp_water(20))  # Outputs approx 4182 J/(kg·K)
