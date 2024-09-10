import matplotlib.pyplot as plt
import numpy as np
import random
import os

# Constants for maze size
MAZE_SIZE = 8

# Directions for movement: (dx, dy, opposite direction index)
DIRECTIONS = {
    "top": (0, -1, "bottom"),
    "bottom": (0, 1, "top"),
    "left": (-1, 0, "right"),
    "right": (1, 0, "left"),
}

# Mapping for opposite directions
OPPOSITE = {"top": 1, "bottom": 0, "left": 3, "right": 2}


# Simulated maze generation function using recursive backtracking
def generate_test_maze():
    # The maze is represented as a grid of cells with walls (top, bottom, left, right)
    maze = np.ones((MAZE_SIZE, MAZE_SIZE, 4), dtype=bool)  # Start with all walls intact
    visited = np.zeros((MAZE_SIZE, MAZE_SIZE), dtype=bool)

    def remove_wall(x, y, direction):
        dx, dy, opposite = DIRECTIONS[direction]
        nx, ny = x + dx, y + dy
        if 0 <= nx < MAZE_SIZE and 0 <= ny < MAZE_SIZE and not visited[ny, nx]:
            visited[ny, nx] = True
            maze[y, x][list(DIRECTIONS.keys()).index(direction)] = False
            maze[ny, nx][list(DIRECTIONS.keys()).index(opposite)] = False
            generate_maze_from(nx, ny)

    # Recursive backtracking maze generation
    def generate_maze_from(x, y):
        visited[y, x] = True
        directions = list(DIRECTIONS.keys())
        random.shuffle(directions)  # Shuffle directions to make the maze random
        for direction in directions:
            remove_wall(x, y, direction)

    # Start maze generation from a random cell
    start_x, start_y = random.randint(0, MAZE_SIZE - 1), random.randint(
        0, MAZE_SIZE - 1
    )
    generate_maze_from(start_x, start_y)

    return maze


# Simulated get_walls function using the test maze
def get_walls(x, y, test_maze):
    # Get the walls for the current cell (x, y) from the test maze
    return test_maze[y, x]


# DFS function to explore the maze
def explore_maze(maze, visited, x, y, test_maze, ax, step_counter):
    if x < 0 or x >= MAZE_SIZE or y < 0 or y >= MAZE_SIZE or visited[y, x]:
        return

    visited[y, x] = True
    top, bottom, left, right = get_walls(x, y, test_maze)

    # Store the walls in the maze structure
    maze[y, x, 0] = top
    maze[y, x, 1] = bottom
    maze[y, x, 2] = left
    maze[y, x, 3] = right

    # Increment the global step counter
    step_counter[0] += 1

    # Plot the current exploration state and save the plot
    plot_maze(maze, ax, visited, step_counter[0])

    # DFS for unvisited neighbors
    if not top:
        explore_maze(maze, visited, x, y - 1, test_maze, ax, step_counter)
    if not bottom:
        explore_maze(maze, visited, x, y + 1, test_maze, ax, step_counter)
    if not left:
        explore_maze(maze, visited, x - 1, y, test_maze, ax, step_counter)
    if not right:
        explore_maze(maze, visited, x + 1, y, test_maze, ax, step_counter)


# Function to plot the maze and save the figure
def plot_maze(maze, ax, visited, explored_steps):
    ax.clear()
    for y in range(MAZE_SIZE):
        for x in range(MAZE_SIZE):
            top, bottom, left, right = maze[y, x]
            if top:
                ax.plot([x, x + 1], [y, y], color="black")
            if bottom:
                ax.plot([x, x + 1], [y + 1, y + 1], color="black")
            if left:
                ax.plot([x, x], [y, y + 1], color="black")
            if right:
                ax.plot([x + 1, x + 1], [y, y + 1], color="black")

            # Mark the explored cells
            if visited[y, x]:
                ax.fill_between([x, x + 1], y, y + 1, color="lightblue")

    ax.set_xlim(0, MAZE_SIZE)
    ax.set_ylim(0, MAZE_SIZE)
    ax.set_aspect("equal")
    plt.gca().invert_yaxis()  # Invert y-axis to match the grid orientation
    plt.draw()

    # save_dir = "./maze/maze_plots"
    # os.makedirs(save_dir, exist_ok=True)

    # Save the current plot with a filename based on the step number
    # plt.savefig(f"{save_dir}/{explored_steps:03d}.png")

    plt.pause(0.5)  # Pause to see exploration progress


# Main function to explore the maze step by step
def main():
    # Initialize maze with no walls: (top, bottom, left, right) for each cell
    maze = np.zeros((MAZE_SIZE, MAZE_SIZE, 4), dtype=bool)
    visited = np.zeros((MAZE_SIZE, MAZE_SIZE), dtype=bool)

    # Generate a test maze
    test_maze = generate_test_maze()

    # Set up the plot
    fig, ax = plt.subplots(figsize=(6, 6))
    plt.ion()  # Enable interactive mode for live updating

    # Start exploration from the center of the maze
    start_x, start_y = MAZE_SIZE // 2, MAZE_SIZE // 2
    step_counter = [0]  # Using a list to keep a mutable counter for step count
    explore_maze(maze, visited, start_x, start_y, test_maze, ax, step_counter)

    plt.ioff()  # Disable interactive mode
    plt.show()


if __name__ == "__main__":
    main()
