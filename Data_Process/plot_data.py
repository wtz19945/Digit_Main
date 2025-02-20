import matplotlib.pyplot as plt
import numpy as np

# Load path from file
path = []
with open("./src/Digit_Main/Data_Process/path.txt", "r") as f:
    for line in f:
        x, y = map(int, line.split())
        path.append((y + .5, x + .5))


with open("./src/Digit_Main/Data_Process/grid.txt", "r") as f:
    # Read grid size
    rows, cols = map(int, f.readline().split())

    # Read grid data
    grid = []
    for _ in range(rows):
        grid.append(list(map(int, f.readline().split())))

new_grid = np.array(grid)
fig, ax = plt.subplots()

for y in range(rows):
        for x in range(cols):
            if new_grid[y, x] == 1:
                ax.add_patch(plt.Rectangle((x, y), 1, 1, color='black'))  # Obstacle


# Extract x, y coordinates
x_coords, y_coords = zip(*path)

ax.plot(x_coords, y_coords, marker='o', color='red', label="A* Path")
# Set limits and labels
ax.set_xlim(-1, cols)
ax.set_ylim(rows, -1)  # Flip y-axis for correct visualization
ax.set_xticks(range(cols))
ax.set_yticks(range(rows))
ax.grid(True, linestyle="--", linewidth=0.5)
ax.set_aspect("equal")
plt.legend()
plt.title("A* Pathfinding Visualization")
plt.show()