#!/usr/bin/env python3
"""
Shooter LUT Regression Analysis
================================
Pools data from multiple tuning versions (excluding ExodusEndPush and sentinel points)
to find best-fit equations for velocity and hood angle as a function of distance.
"""

import numpy as np
from scipy.optimize import curve_fit
import warnings
warnings.filterwarnings('ignore')

# ============================================================
# 1. DATA DEFINITIONS
# ============================================================

versions_ordered = [
    "V5.4.2",
    "V6.0.2",
    "V6.CompReady",
    "V6/5.2 (stable)",
    "Tuned Power",
    "18BallAuto",
    "Init Changes",
]

velocity_data = {
    "V5.4.2": [
        (44,1750),(65,2000),(69,2000),(78,2150),(99,2300),(135,2600),(139,2650),(147,2700),(159,2800)
    ],
    "V6.0.2": [
        (44,1750),(65,2000),(69,2000),(78,2150),(99,2300),(130,2600),(135,2600),(139,2650),(147,2700),(159,2800)
    ],
    "V6.CompReady": [
        (40,1850),(49,1820),(57,1900),(65,1950),(70,2000),(74,2020),(83,2100),(95,2180),
        (104,2300),(115,2400),(126,2700),(135,2720),(140,2760),(145,2800),(153,2800),(162,2900)
    ],
    "V6/5.2 (stable)": [
        (42,1720),(55,1820),(66,1870),(80,2020),(90,2120),(100,2200),(113,2470),(128.6,2570),
        (140.5,2670),(146,2720),(150,2820)
    ],
    "Tuned Power": [
        (45,1640),(55,1750),(65,1880),(75,1950),(87,2160),(95,2250),(105,2300),(115,2460),
        (130,2520),(140,2620),(150,2720),(156,2720)
    ],
    "18BallAuto": [
        (46,1640),(56,1750),(66,1880),(76,1950),(88,2160),(96,2250),(106,2300),(113,2470),
        (128.6,2570),(140.5,2670),(146,2720),(150,2820)
    ],
    "Init Changes": [
        (38,1700),(49,1740),(58,1780),(68.5,1900),(76.5,2020),(85,2060),(101,2280),(128,2550),
        (140,2620),(148,2680),(151,2700),(160,2880)
    ],
}

hood_data = {
    "V5.4.2": [
        (44,30),(65,40),(69,40),(78,43),(99,45),(135,63),(139,63),(147,63),(159,63)
    ],
    "V6.0.2": [
        (44,30),(65,40),(69,40),(78,43),(99,45),(130,61),(135,63),(139,63),(147,63),(159,63)
    ],
    "V6.CompReady": [
        (40,30),(49,32),(57,36),(65,40),(70,40),(74,40),(83,44),(95,45),(104,49),(115,52),
        (126,56),(135,56),(140,56),(145,56),(153,54),(162,53)
    ],
    "V6/5.2 (stable)": [
        (42,30),(55,34),(66,40),(80,45),(90,45.5),(100,46),(113,51),(128.6,52),(140.5,52.5),
        (146,54),(150,55)
    ],
    "Tuned Power": [
        (45,32),(55,36),(65,40),(75,43),(87,46),(95,50),(105,50),(115,51),(130,53),(140,53),
        (150,55),(156,52)
    ],
    "18BallAuto": [
        (46,32),(56,36),(66,40),(76,43),(88,46),(96,50),(106,50),(113,51),(128.6,52),(140.5,52.5),
        (146,54),(150,55)
    ],
    "Init Changes": [
        (38,32),(49,35),(58,37),(68.5,42),(76.5,45),(85,46.5),(101,48),(128,52),(140,53),
        (148,54),(151,54.5),(160,54.5)
    ],
}

# ============================================================
# 2. POOL DATA (with V6/5.2 weighted double)
# ============================================================

def pool_data(data_dict, double_weight_key="V6/5.2 (stable)"):
    """Pool all data points, weighting one version double."""
    all_x = []
    all_y = []
    for version, points in data_dict.items():
        for x, y in points:
            all_x.append(x)
            all_y.append(y)
            if version == double_weight_key:
                all_x.append(x)
                all_y.append(y)
    return np.array(all_x, dtype=float), np.array(all_y, dtype=float)

vel_x, vel_y = pool_data(velocity_data)
hood_x, hood_y = pool_data(hood_data)

print("=" * 80)
print("SHOOTER LUT REGRESSION ANALYSIS")
print("=" * 80)
print(f"\nPooled velocity data points: {len(vel_x)} (including double-weighted V6/5.2)")
print(f"Pooled hood angle data points: {len(hood_x)} (including double-weighted V6/5.2)")
print(f"Distance range: {min(vel_x):.1f}\" to {max(vel_x):.1f}\"")

# ============================================================
# 3. MODEL DEFINITIONS
# ============================================================

def linear_model(x, a, b):
    return a * x + b

def power_model(x, a, p, b):
    return a * np.power(x, p) + b

def make_fixed_power_model(p):
    def model(x, a, b):
        return a * np.power(x, p) + b
    return model

def r_squared(y_actual, y_predicted):
    ss_res = np.sum((y_actual - y_predicted) ** 2)
    ss_tot = np.sum((y_actual - np.mean(y_actual)) ** 2)
    return 1 - ss_res / ss_tot

def fit_and_report(x, y, label):
    """Fit all models and report results."""
    print(f"\n{'~' * 80}")
    print(f"  {label}")
    print(f"{'~' * 80}")

    results = []

    # --- Linear (power 1) ---
    try:
        popt, _ = curve_fit(linear_model, x, y)
        y_pred = linear_model(x, *popt)
        r2 = r_squared(y, y_pred)
        mae = np.mean(np.abs(y - y_pred))
        max_err = np.max(np.abs(y - y_pred))
        eq = f"y = {popt[0]:.4f} * d + {popt[1]:.2f}"
        results.append(("Linear (p=1)", eq, r2, mae, max_err, popt, 1.0))
    except Exception as e:
        print(f"  Linear fit failed: {e}")

    # --- Fixed power models ---
    for p in [1.25, 1.5, 1.75, 2.0]:
        try:
            model = make_fixed_power_model(p)
            popt, _ = curve_fit(model, x, y, p0=[1.0, 0.0], maxfev=10000)
            y_pred = model(x, *popt)
            r2 = r_squared(y, y_pred)
            mae = np.mean(np.abs(y - y_pred))
            max_err = np.max(np.abs(y - y_pred))
            eq = f"y = {popt[0]:.6f} * d^{p} + {popt[1]:.2f}"
            name = f"Power (p={p})"
            if p == 2.0:
                name = "Quadratic (p=2)"
            results.append((name, eq, r2, mae, max_err, popt, p))
        except Exception as e:
            print(f"  Power {p} fit failed: {e}")

    # --- Free power model: y = a * d^p + b ---
    best_free = None
    try:
        best_cost = float('inf')
        for p0_guess in [0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5]:
            try:
                popt, _ = curve_fit(power_model, x, y, p0=[1.0, p0_guess, 0.0], maxfev=50000)
                y_pred = power_model(x, *popt)
                cost = np.sum((y - y_pred) ** 2)
                if cost < best_cost:
                    best_cost = cost
                    best_free = popt
            except:
                pass

        if best_free is not None:
            y_pred = power_model(x, *best_free)
            r2 = r_squared(y, y_pred)
            mae = np.mean(np.abs(y - y_pred))
            max_err = np.max(np.abs(y - y_pred))
            eq = f"y = {best_free[0]:.6f} * d^{best_free[1]:.4f} + {best_free[2]:.2f}"
            results.append(("Free Power (p=fit)", eq, r2, mae, max_err, best_free, best_free[1]))
    except Exception as e:
        print(f"  Free power fit failed: {e}")

    # --- Print results table ---
    print(f"\n  {'Model':<22} {'R-sq':>8} {'MAE':>10} {'Max Err':>10}")
    print(f"  {'_'*22} {'_'*8} {'_'*10} {'_'*10}")
    for name, eq, r2, mae, max_err, _, _ in results:
        print(f"  {name:<22} {r2:>8.5f} {mae:>10.2f} {max_err:>10.2f}")

    print(f"\n  Equations:")
    for name, eq, r2, mae, max_err, _, _ in results:
        print(f"    {name:<22}  {eq}")

    # Find best by R-sq
    if results:
        best = max(results, key=lambda r: r[2])
        print(f"\n  >>> BEST FIT: {best[0]}  (R-sq = {best[2]:.5f}, MAE = {best[3]:.2f}, Max Err = {best[4]:.2f})")
        print(f"      {best[1]}")
        if best[0] == "Free Power (p=fit)":
            print(f"      Optimal power exponent: p = {best[6]:.4f}")

    return results

# ============================================================
# 4. RUN FITS
# ============================================================

print("\n")
vel_results = fit_and_report(vel_x, vel_y, "VELOCITY vs DISTANCE (ticks/sec vs inches)")
hood_results = fit_and_report(hood_x, hood_y, "HOOD ANGLE vs DISTANCE (degrees vs inches)")

# ============================================================
# 5. VERSION-TO-VERSION TREND ANALYSIS
# ============================================================

print(f"\n\n{'=' * 80}")
print("VERSION-TO-VERSION TREND ANALYSIS")
print("=" * 80)

analysis_distances = [50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150]

def interpolate_version(data_points, target_d):
    """Linear interpolation within a version's data."""
    pts = sorted(data_points, key=lambda p: p[0])
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    if target_d < xs[0] or target_d > xs[-1]:
        return None
    return float(np.interp(target_d, xs, ys))

print("\n--- Velocity at common distances (by version, in chronological order) ---")
print(f"  {'Version':<20}", end="")
for d in analysis_distances:
    print(f"  {d:>5}\"", end="")
print()
print(f"  {'_'*20}", end="")
for d in analysis_distances:
    print(f"  {'_'*6}", end="")
print()

vel_by_dist = {d: [] for d in analysis_distances}
for ver in versions_ordered:
    print(f"  {ver:<20}", end="")
    for d in analysis_distances:
        val = interpolate_version(velocity_data[ver], d)
        if val is not None:
            print(f"  {val:>5.0f}", end="")
            vel_by_dist[d].append(val)
        else:
            print(f"  {'---':>5}", end="")
    print()

print(f"\n  {'MEAN':<20}", end="")
for d in analysis_distances:
    if vel_by_dist[d]:
        print(f"  {np.mean(vel_by_dist[d]):>5.0f}", end="")
    else:
        print(f"  {'---':>5}", end="")
print()

print(f"  {'STD DEV':<20}", end="")
for d in analysis_distances:
    if len(vel_by_dist[d]) >= 2:
        print(f"  {np.std(vel_by_dist[d]):>5.0f}", end="")
    else:
        print(f"  {'---':>5}", end="")
print()

print(f"  {'SPREAD (max-min)':<20}", end="")
for d in analysis_distances:
    if len(vel_by_dist[d]) >= 2:
        spread = max(vel_by_dist[d]) - min(vel_by_dist[d])
        print(f"  {spread:>5.0f}", end="")
    else:
        print(f"  {'---':>5}", end="")
print()

print("\n--- Hood Angle at common distances (by version, in chronological order) ---")
print(f"  {'Version':<20}", end="")
for d in analysis_distances:
    print(f"  {d:>6}\"", end="")
print()
print(f"  {'_'*20}", end="")
for d in analysis_distances:
    print(f"  {'_'*7}", end="")
print()

hood_by_dist = {d: [] for d in analysis_distances}
for ver in versions_ordered:
    print(f"  {ver:<20}", end="")
    for d in analysis_distances:
        val = interpolate_version(hood_data[ver], d)
        if val is not None:
            print(f"  {val:>6.1f}", end="")
            hood_by_dist[d].append(val)
        else:
            print(f"  {'---':>6}", end="")
    print()

print(f"\n  {'MEAN':<20}", end="")
for d in analysis_distances:
    if hood_by_dist[d]:
        print(f"  {np.mean(hood_by_dist[d]):>6.1f}", end="")
    else:
        print(f"  {'---':>6}", end="")
print()

print(f"  {'STD DEV':<20}", end="")
for d in analysis_distances:
    if len(hood_by_dist[d]) >= 2:
        print(f"  {np.std(hood_by_dist[d]):>6.1f}", end="")
    else:
        print(f"  {'---':>6}", end="")
print()

print(f"  {'SPREAD (max-min)':<20}", end="")
for d in analysis_distances:
    if len(hood_by_dist[d]) >= 2:
        spread = max(hood_by_dist[d]) - min(hood_by_dist[d])
        print(f"  {spread:>6.1f}", end="")
    else:
        print(f"  {'---':>6}", end="")
print()

# --- Trend direction analysis ---
print("\n--- Trend Direction (early versions vs late versions) ---")
early_versions = ["V5.4.2", "V6.0.2", "V6.CompReady"]
late_versions = ["Tuned Power", "18BallAuto", "Init Changes"]

print("\n  Velocity trends:")
for d in [80, 100, 130, 150]:
    early_vals = []
    late_vals = []
    for ver in early_versions:
        val = interpolate_version(velocity_data[ver], d)
        if val is not None:
            early_vals.append(val)
    for ver in late_versions:
        val = interpolate_version(velocity_data[ver], d)
        if val is not None:
            late_vals.append(val)
    if early_vals and late_vals:
        early_mean = np.mean(early_vals)
        late_mean = np.mean(late_vals)
        diff = late_mean - early_mean
        direction = "INCREASED" if diff > 0 else "DECREASED"
        print(f"    d={d:>3}\": early avg={early_mean:>7.0f}, late avg={late_mean:>7.0f}, "
              f"change={diff:>+7.0f} ({direction})")

print("\n  Hood angle trends:")
for d in [80, 100, 130, 150]:
    early_vals = []
    late_vals = []
    for ver in early_versions:
        val = interpolate_version(hood_data[ver], d)
        if val is not None:
            early_vals.append(val)
    for ver in late_versions:
        val = interpolate_version(hood_data[ver], d)
        if val is not None:
            late_vals.append(val)
    if early_vals and late_vals:
        early_mean = np.mean(early_vals)
        late_mean = np.mean(late_vals)
        diff = late_mean - early_mean
        direction = "INCREASED" if diff > 0 else "DECREASED"
        print(f"    d={d:>3}\": early avg={early_mean:>6.1f}, late avg={late_mean:>6.1f}, "
              f"change={diff:>+6.1f} ({direction})")

# ============================================================
# 6. CONSENSUS LUT FROM BEST FIT
# ============================================================

print(f"\n\n{'=' * 80}")
print("RECOMMENDED CONSENSUS LUT (from best-fit equations)")
print("=" * 80)

standard_distances = [40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160]

vel_best = max(vel_results, key=lambda r: r[2])
hood_best = max(hood_results, key=lambda r: r[2])

print(f"\n  Using best velocity model: {vel_best[0]}")
print(f"    {vel_best[1]}")
print(f"  Using best hood model: {hood_best[0]}")
print(f"    {hood_best[1]}")

vel_popt = vel_best[5]
vel_power = vel_best[6]
hood_popt = hood_best[5]
hood_power = hood_best[6]

def predict(d, popt, power, model_name):
    """Predict value at distance d using the fitted model."""
    if model_name == "Linear (p=1)":
        return popt[0] * d + popt[1]
    elif model_name == "Free Power (p=fit)":
        return popt[0] * (d ** popt[1]) + popt[2]
    else:
        return popt[0] * (d ** power) + popt[1]

print(f"\n  {'Distance':>10} {'Velocity':>12} {'Vel (round)':>12} {'Hood Angle':>12} {'Hood (round)':>13}")
print(f"  {'_'*10} {'_'*12} {'_'*12} {'_'*12} {'_'*13}")

lut_vel = []
lut_hood = []
for d in standard_distances:
    v = predict(d, vel_popt, vel_power, vel_best[0])
    h = predict(d, hood_popt, hood_power, hood_best[0])
    h = max(30.0, min(63.0, h))
    v_round = round(v / 10) * 10
    h_round = round(h * 2) / 2
    lut_vel.append((d, v_round))
    lut_hood.append((d, h_round))
    print(f"  {d:>8}\"  {v:>10.1f}  {v_round:>10.0f}  {h:>10.1f}  {h_round:>11.1f}")

# Java InterpLUT format
print(f"\n\n  --- Java InterpLUT format (copy-paste ready) ---")
print(f"\n  // VELOCITY LUT (best fit: {vel_best[0]})")
print(f"  // {vel_best[1]}")
print(f"  // R-sq = {vel_best[2]:.5f}, MAE = {vel_best[3]:.1f}")
print(f"  VELOCITY_DATA = new double[][]{{")
print(f"      {{0, {lut_vel[0][1]:.0f}}},  // clamp low")
for d, v in lut_vel:
    print(f"      {{{d}, {v:.0f}}},")
print(f"      {{300, {lut_vel[-1][1]:.0f}}}  // clamp high")
print(f"  }};")

print(f"\n  // HOOD ANGLE LUT (best fit: {hood_best[0]})")
print(f"  // {hood_best[1]}")
print(f"  // R-sq = {hood_best[2]:.5f}, MAE = {hood_best[3]:.1f}")
print(f"  HOOD_DATA = new double[][]{{")
print(f"      {{0, {lut_hood[0][1]:.1f}}},  // clamp low")
for d, h in lut_hood:
    print(f"      {{{d}, {h:.1f}}},")
print(f"      {{300, {lut_hood[-1][1]:.1f}}}  // clamp high")
print(f"  }};")

# ============================================================
# 7. COMPARISON: best-fit vs per-version interpolated averages
# ============================================================

print(f"\n\n{'=' * 80}")
print("COMPARISON: Best-Fit Equation vs Cross-Version Interpolated Average")
print("=" * 80)
print(f"\n  {'Distance':>10} {'Fit Vel':>10} {'Avg Vel':>10} {'Diff':>8} {'Fit Hood':>10} {'Avg Hood':>10} {'Diff':>8}")
print(f"  {'_'*10} {'_'*10} {'_'*10} {'_'*8} {'_'*10} {'_'*10} {'_'*8}")

for d in [50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150]:
    fit_v = predict(d, vel_popt, vel_power, vel_best[0])
    fit_h = predict(d, hood_popt, hood_power, hood_best[0])
    fit_h = max(30.0, min(63.0, fit_h))

    vel_vals = []
    hood_vals = []
    for ver in versions_ordered:
        vv = interpolate_version(velocity_data[ver], d)
        hv = interpolate_version(hood_data[ver], d)
        if vv is not None:
            vel_vals.append(vv)
            if ver == "V6/5.2 (stable)":
                vel_vals.append(vv)
        if hv is not None:
            hood_vals.append(hv)
            if ver == "V6/5.2 (stable)":
                hood_vals.append(hv)

    avg_v = np.mean(vel_vals) if vel_vals else float('nan')
    avg_h = np.mean(hood_vals) if hood_vals else float('nan')

    dv = fit_v - avg_v
    dh = fit_h - avg_h
    print(f"  {d:>8}\"  {fit_v:>8.0f}  {avg_v:>8.0f}  {dv:>+7.0f}  {fit_h:>8.1f}  {avg_h:>8.1f}  {dh:>+7.1f}")

# ============================================================
# 8. RESIDUAL ANALYSIS per version
# ============================================================

print(f"\n\n{'=' * 80}")
print("RESIDUAL ANALYSIS: How well does the best fit match each version?")
print("=" * 80)

for label, data_dict, popt, power, model_name in [
    ("VELOCITY", velocity_data, vel_popt, vel_power, vel_best[0]),
    ("HOOD ANGLE", hood_data, hood_popt, hood_power, hood_best[0]),
]:
    print(f"\n  --- {label} ({model_name}) ---")
    print(f"  {'Version':<20} {'N pts':>6} {'MAE':>8} {'Max Err':>10} {'Bias':>8}")
    print(f"  {'_'*20} {'_'*6} {'_'*8} {'_'*10} {'_'*8}")
    for ver in versions_ordered:
        pts = data_dict[ver]
        xs = np.array([p[0] for p in pts], dtype=float)
        ys = np.array([p[1] for p in pts], dtype=float)
        preds = np.array([predict(d, popt, power, model_name) for d in xs])
        residuals = ys - preds
        mae = np.mean(np.abs(residuals))
        max_err = np.max(np.abs(residuals))
        bias = np.mean(residuals)
        print(f"  {ver:<20} {len(pts):>6} {mae:>8.1f} {max_err:>10.1f} {bias:>+8.1f}")

print(f"\n\n{'=' * 80}")
print("ANALYSIS COMPLETE")
print("=" * 80)
