"""
Localization analysis for the paper's results section.

Reads the drone logs (run this from the 'controllers' folder after a simulation):
    exp_drone/exp_drone1_log.csv, exp_drone2/exp_drone2_log.csv, relay_drone/relay_drone_log.csv
Log columns: time, x, y (ground truth), est_x, est_y (dead-reckoned/GPS estimate), err, sigma, gps_ok

Outputs:
    localization_summary.csv   - one row per drone (RMSE/mean/max error, inside vs outside forest)
    forest_crossings.csv       - one row per GPS-denied crossing (distance, final error, error/distance)
    localization_error.png     - error vs time (GPS-available periods shaded) + true vs estimated path
"""
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FILES = {
    "Exploration Drone 1": "exp_drone/exp_drone1_log.csv",
    "Exploration Drone 2": "exp_drone2/exp_drone2_log.csv",
    "Relay Drone": "relay_drone/relay_drone_log.csv",
}
MIN_CROSSING_SAMPLES = 100

summary, crossings, data = [], [], {}

for name, path in FILES.items():
    if not os.path.exists(path):
        print("missing:", path)
        continue
    d = pd.read_csv(path)
    d = d[d.time > 5.0].reset_index(drop=True)      # ignore take-off
    data[name] = d
    ins, out = d[d.gps_ok == 0], d[d.gps_ok == 1]
    rmse = lambda g: float(np.sqrt((g.err ** 2).mean())) if len(g) else float("nan")
    summary.append({
        "drone": name,
        "duration_s": round(d.time.iloc[-1] - d.time.iloc[0], 1),
        "rmse_inside_m": round(rmse(ins), 3),
        "mean_inside_m": round(ins.err.mean(), 3) if len(ins) else float("nan"),
        "max_inside_m": round(ins.err.max(), 3) if len(ins) else float("nan"),
        "rmse_outside_m": round(rmse(out), 3),
    })
    seg = (d.gps_ok != d.gps_ok.shift()).cumsum()
    for _, g in d[d.gps_ok == 0].groupby(seg):
        if len(g) < MIN_CROSSING_SAMPLES:
            continue
        dist = float(np.hypot(g.est_x.diff(), g.est_y.diff()).fillna(0).sum())
        crossings.append({
            "drone": name,
            "start_s": round(g.time.iloc[0], 1),
            "distance_m": round(dist, 2),
            "final_error_m": round(g.err.iloc[-1], 3),
            "error_per_metre": round(g.err.iloc[-1] / dist, 4),
            "inside_sigma": bool(g.err.iloc[-1] <= g.sigma.iloc[-1]),
        })

s = pd.DataFrame(summary)
c = pd.DataFrame(crossings)
s.to_csv("localization_summary.csv", index=False)
c.to_csv("forest_crossings.csv", index=False)
print(s.to_string(index=False))
if len(c):
    print("\nGPS-denied forest crossings: n=%d, mean distance %.1f m, mean final error %.2f m, "
          "error/distance mean %.3f (max %.3f), final error within reported sigma in %.0f %% of crossings"
          % (len(c), c.distance_m.mean(), c.final_error_m.mean(), c.error_per_metre.mean(),
             c.error_per_metre.max(), 100 * c.inside_sigma.mean()))

if data:
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 5))
    for name, d in data.items():
        a1.plot(d.time, d.err, linewidth=1.2, label=name)
    first = next(iter(data.values()))
    gps = first.gps_ok.values
    t = first.time.values
    a1.fill_between(t, 0, 1, where=gps == 1, transform=a1.get_xaxis_transform(),
                    color="lightgreen", alpha=0.3, label="GPS available (Exploration Drone 1)")
    a1.set_xlabel("Time (s)")
    a1.set_ylabel("Position estimation error (m)")
    a1.set_title("Estimation error: grows in the forest, resets at GPS fixes")
    a1.grid(True)
    a1.legend(fontsize=8)

    name0 = "Exploration Drone 1" if "Exploration Drone 1" in data else next(iter(data))
    d = data[name0]
    a2.plot(d.x, d.y, color="royalblue", linewidth=1.5, label="True path")
    a2.plot(d.est_x, d.est_y, color="orange", linewidth=1.0, linestyle="--", label="Estimated path")
    a2.add_patch(plt.Rectangle((-10, -10), 20, 20, fill=False, edgecolor="darkgreen", linestyle="--"))
    a2.set_xlim(-13, 13)
    a2.set_ylim(-13, 13)
    a2.set_aspect("equal")
    a2.set_xlabel("x (m)")
    a2.set_ylabel("y (m)")
    a2.set_title(name0 + ": true vs estimated path")
    a2.grid(True)
    a2.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig("localization_error.png", dpi=200)
    print("saved localization_error.png")
