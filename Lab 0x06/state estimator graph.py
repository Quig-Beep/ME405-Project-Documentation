import numpy as np
import matplotlib.pyplot as plt

# =========================
# Read File
# =========================

graphing_data = []

with open("Romi Path Data.txt", "r") as file:
    for line in file:
        line = line.strip()
        if line:  # skip blank lines
            arc, psi = line.split(",")
            graphing_data.append([float(arc), float(psi)])

print("Loaded", len(graphing_data), "data points")

# Convert to numpy arrays
data = np.array(graphing_data)
arc_length = data[:, 0]   # mm
heading = data[:, 1]      # radians

# =========================
# Convert to Cartesian
# =========================

x = np.zeros(len(arc_length))
y = np.zeros(len(arc_length))

for k in range(1, len(arc_length)):
    ds = arc_length[k] - arc_length[k-1]
    x[k] = x[k-1] + ds * np.cos(heading[k])
    y[k] = y[k-1] + ds * np.sin(heading[k])

# =========================
# Plot Path
# =========================

plt.figure()
plt.plot(x, y)
plt.axis("equal")  # ensures circle looks like circle
plt.xlabel("x [mm]")
plt.ylabel("y [mm]")
plt.title("Romi Estimated Path")
plt.grid(True)
plt.show()
