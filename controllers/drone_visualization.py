import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

# STYLE
plt.style.use("seaborn-v0_8")

# SIDE BY SIDE WINDOW
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))


# -------------------------------------------------
# COMPUTE DISTANCE
# -------------------------------------------------

def compute_distance(file):

    data = pd.read_csv(file)

    if len(data) < 2:
        return [], [], data

    x = data["x"].values
    y = data["y"].values
    t = data["time"].values

    dist = [0]

    for i in range(1, len(x)):

        d = np.sqrt(
            (x[i] - x[i - 1])**2 +
            (y[i] - y[i - 1])**2
        )

        dist.append(dist[-1] + d)

    return t, dist, data


# -------------------------------------------------
# LIVE UPDATE FUNCTION
# -------------------------------------------------

def update(frame):

    ax1.clear()
    ax2.clear()

    try:

        # LOAD CSV FILES

        t1, d1, data1 = compute_distance(
            r"exp_drone/exp_drone1_log.csv"
        )

        t2, d2, data2 = compute_distance(
            r"exp_drone2/exp_drone2_log.csv"
        )

        t3, d3, data3 = compute_distance(
            r"relay_drone/relay_drone_log.csv"
        )

        # =====================================================
        # DISTANCE GRAPH
        # =====================================================

        if len(t1) > 0:
            ax1.plot(
                t1,
                d1,
                color="royalblue",
                linewidth=2.5,
                label="Exploration Drone 1"
            )

        if len(t2) > 0:
            ax1.plot(
                t2,
                d2,
                color="red",
                linewidth=2.5,
                label="Exploration Drone 2"
            )

        if len(t3) > 0:
            ax1.plot(
                t3,
                d3,
                color="green",
                linewidth=2.5,
                label="Relay Drone"
            )

        ax1.set_title(
            "Distance Covered by Drones",
            fontsize=15,
            fontweight="bold"
        )

        ax1.set_xlabel("Time (seconds)", fontsize=12)
        ax1.set_ylabel("Distance (meters)", fontsize=12)

        ax1.grid(True)

        ax1.legend(
            loc="upper left",
            fontsize=9
        )

        # =====================================================
        # DRONE MOVEMENT MAP
        # =====================================================

        # Exploration Drone 1

        if len(data1) > 0:

            ax2.plot(
                data1["x"],
                data1["y"],
                color="royalblue",
                linewidth=2.5,
                label="Exploration Drone 1"
            )

            ax2.scatter(
                data1["x"].iloc[-1],
                data1["y"].iloc[-1],
                color="royalblue",
                s=100
            )

        # Exploration Drone 2

        if len(data2) > 0:

            ax2.plot(
                data2["x"],
                data2["y"],
                color="red",
                linewidth=2.5,
                label="Exploration Drone 2"
            )

            ax2.scatter(
                data2["x"].iloc[-1],
                data2["y"].iloc[-1],
                color="red",
                s=100
            )

        # Relay Drone

        if len(data3) > 0:

            ax2.plot(
                data3["x"],
                data3["y"],
                color="green",
                linewidth=2.5,
                label="Relay Drone"
            )

            ax2.scatter(
                data3["x"].iloc[-1],
                data3["y"].iloc[-1],
                color="green",
                s=100
            )

        # =====================================================
        # FIRE POSITION
        # =====================================================

        fire_x = 4
        fire_y = -5

        ax2.scatter(
            fire_x,
            fire_y,
            color="orange",
            marker="*",
            s=350,
            label="Fire Location"
        )

        # =====================================================
        # SMOKE POSITION
        # =====================================================

        smoke_x = 4
        smoke_y = 6

        ax2.scatter(
            smoke_x,
            smoke_y,
            color="gray",
            marker="X",
            s=220,
            label="Smoke Location"
        )

        # =====================================================
        # MAP SETTINGS
        # =====================================================

        ax2.set_title(
            "Drone Movement Map",
            fontsize=15,
            fontweight="bold"
        )

        ax2.set_xlabel("X Position", fontsize=12)
        ax2.set_ylabel("Y Position", fontsize=12)

        ax2.set_xlim(-13, 13)
        ax2.set_ylim(-13, 13)

        # Forest boundary (+-10 m); GPS is only available outside it
        ax2.add_patch(patches.Rectangle((-10, -10), 20, 20, fill=False,
                                        edgecolor="darkgreen", linestyle="--",
                                        linewidth=1.5, label="Forest boundary"))

        ax2.set_aspect("equal")

        ax2.grid(True)

        ax2.legend(
            loc="upper right",
            fontsize=10
        )

    except Exception as e:
        print(e)


# -------------------------------------------------
# LIVE ANIMATION
# -------------------------------------------------

ani = FuncAnimation(
    fig,
    update,
    interval=1000
)

plt.tight_layout()

plt.show()
