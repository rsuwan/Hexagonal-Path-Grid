import tkinter as tk
from heapq import heappop, heappush
import math

# Network dimensions
height = 10
width = 10

# The window
window = tk.Tk()

# Create the canvas
canvas = tk.Canvas(width=width * 60, height=height * 50, bg='white')

# Draw the hexagonal grid
canvas_polygons = []

# Draw the hexagonal grid
for i in range(height):
    for j in range(width):
        x = j * 50 + (i % 2) * 25
        y = i * 45
        polygon_id = canvas.create_polygon(
            x + 25, y,
            x + 50, y + 15,
            x + 50, y + 45,
            x + 25, y + 60,
            x, y + 45,
            x, y + 15,
            fill='#285943',
            outline='#519D9E'
        )
        canvas_polygons.append(polygon_id)

# Create a list to store the colors of the cells
cell_colors = ['#285943' for _ in range(width * height)]

# Define the start and goal points
start = None
goal = None
goal_selected = False

# Function to handle click event
def on_click(event):
    global start, goal, goal_selected

    # Get the coordinates of the click
    x, y = event.x, event.y

    # Get the index of the cell that was clicked
    cell_col = x // 50
    cell_row = y // 45

    if cell_row % 2 == 0:
        cell_index = cell_row * width + cell_col
    else:
        cell_index = cell_row * width + (cell_col - 1)

    # Check if the cell that was clicked is already the start or goal point
    if cell_index == start or cell_index == goal:
        return

    # Set the start point and color
    if start is None:
        start = cell_index
        cell_colors[start] = 'yellow'
        # Reset the goal point and color
        goal = None
        goal_selected = False
    elif not goal_selected and cell_index != start:
        # Set the goal point and color
        goal = cell_index
        goal_selected = True
        cell_colors[goal] = '#75D701'

    # Run A* search algorithm
    if start is not None and goal is not None:
        path = a_star_search(start, goal)
        if path is not None:
            # Color the path cells
            for cell in path:
                if cell != start and cell != goal:
                    cell_colors[cell] = '#a6172d'

    # Update the canvas
    update_canvas()

    # Update the color of the goal cell
    if goal is not None:
        canvas.itemconfig(canvas_polygons[goal], fill='#75D701')

# Function to update the canvas
def update_canvas():
    for index, color in enumerate(cell_colors):
        canvas.itemconfig(canvas_polygons[index], fill=color)


# Bind the click event to the canvas
canvas.bind('<Button-1>', on_click)

# Show the canvas
canvas.pack()


# Calculate the Euclidean distance between two points
def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

# Calculate the Manhattan distance between two cells
def distance(cell1, cell2):
    row1, col1 = divmod(cell1, width)
    row2, col2 = divmod(cell2, width)
    dx = abs(col1 - col2)
    dy = abs(row1 - row2)
    diagonal_steps = min(dx, dy)
    straight_steps = abs(dx - dy)
    return diagonal_steps + straight_steps


def get_neighbors(cell):
    row, col = divmod(cell, width)
    neighbors = []
    
    if row % 2 == 0:
        if col > 0:
            neighbors.append(cell - 1)
        if col < width - 1:
            neighbors.append(cell + 1)
        if row > 0:
            neighbors.append(cell - width)
        if row < height - 1:
            neighbors.append(cell + width)
        if row > 0 and col > 0:
            neighbors.append(cell - width - 1)
        if row < height - 1 and col > 0:
            neighbors.append(cell + width - 1)
    else:
        if col > 0:
            neighbors.append(cell - 1)
        if col < width - 1:
            neighbors.append(cell + 1)
        if row > 0:
            neighbors.append(cell - width)
        if row < height - 1:
            neighbors.append(cell + width)
        if row > 0 and col < width - 1:
            neighbors.append(cell - width + 1)
        if row < height - 1 and col < width - 1:
            neighbors.append(cell + width + 1)

    return neighbors

 

# Run the A* search algorithm
def a_star_search(start, goal):
    open_set = [(0, start)]
    came_from = {}
    g_scores = {start: 0}
    f_scores = {start: distance(start, goal)}
    visited = set()

    while open_set:
        current = heappop(open_set)[1]

        if current == goal:
            path = reconstruct_path(came_from, current)
            return path

        visited.add(current)

        for neighbor in get_neighbors(current):
            g_score = g_scores[current] + distance(current, neighbor)

            if neighbor in visited and g_score >= g_scores.get(neighbor, float('inf')):
                continue

            if g_score < g_scores.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_scores[neighbor] = g_score
                f_scores[neighbor] = g_score + distance(neighbor, goal)

            if neighbor not in visited:
                heappush(open_set, (f_scores[neighbor], neighbor))

    return None
# Reconstruct the path from the goal to the start
def reconstruct_path(came_from, current):
    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    return path[::-1]


# Run the main event loop
window.mainloop()
