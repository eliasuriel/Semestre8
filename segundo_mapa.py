import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def generate_random_point(map_size, obstacles, min_distance=1):
    while True:
        x = np.random.uniform(0, map_size)
        y = np.random.uniform(0, map_size)
        valid = True
        for obstacle in obstacles:
            if np.sqrt((x - obstacle[0])**2 + (y - obstacle[1])**2) < obstacle[2] + min_distance:
                valid = False
                break
        if valid:
            return x, y

def is_valid_point(point, obstacles, obstacle_clearance):
    for obstacle in obstacles:
        x, y, width, height = obstacle
        if (point[0] - x) >= -obstacle_clearance and (point[0] - x - width) <= obstacle_clearance and \
           (point[1] - y) >= -obstacle_clearance and (point[1] - y - height) <= obstacle_clearance:
            return False
    return True

def nearest_node(nodes, point):
    min_dist = float('inf')
    nearest = None
    for node in nodes:
        dist = np.sqrt((node.x - point[0])**2 + (node.y - point[1])**2)
        if dist < min_dist:
            min_dist = dist
            nearest = node
    return nearest

def new_point_within_bounds(nearest, random_point, step_size, obstacles, obstacle_clearance):
    dist = np.sqrt((random_point[0] - nearest.x)**2 + (random_point[1] - nearest.y)**2)
    if dist < step_size:
        return random_point

    theta = np.arctan2(random_point[1] - nearest.y, random_point[0] - nearest.x)
    new_x = nearest.x + step_size * np.cos(theta)
    new_y = nearest.y + step_size * np.sin(theta)
    new_point = (new_x, new_y)

    # Check if the line segment between nearest and new_point intersects any obstacles
    line_segment = (nearest.x, nearest.y, new_x - nearest.x, new_y - nearest.y)
    for obstacle in obstacles:
        if does_line_intersect_obstacle(line_segment, obstacle, obstacle_clearance):
            # If the line intersects an obstacle, don't add this point
            return None

    return new_point

def does_line_intersect_obstacle(line, obstacle, obstacle_clearance):
    x1, y1, dx, dy = line
    x2, y2, width, height = obstacle

    # Check if line segment intersects with obstacle rectangle
    t1 = ((x1 - x2) * dx + (y1 - y2) * dy) / (dx**2 + dy**2)
    t2 = max(0, min(1, t1))

    closest_x = x2 + t2 * dx
    closest_y = y2 + t2 * dy

    distance_squared = (x1 - closest_x)**2 + (y1 - closest_y)**2

    return distance_squared < (obstacle_clearance**2)

def rrt(map_size, obstacles, num_iterations, step_size, obstacle_clearance):
    start = Node(0, 0)
    nodes = [start]

    for _ in range(num_iterations):
        random_point = generate_random_point(map_size, obstacles, min_distance=obstacle_clearance)
        nearest = nearest_node(nodes, random_point)
        new_point = new_point_within_bounds(nearest, random_point, step_size, obstacles, obstacle_clearance)
        if new_point is not None:  # Check if new_point_within_bounds returned a valid point
            if is_valid_point(new_point, obstacles, obstacle_clearance):
                new_node = Node(new_point[0], new_point[1])
                new_node.parent = nearest
                nodes.append(new_node)

    return nodes


def plot_map(map_size, obstacles, nodes):
    fig, ax = plt.subplots()
    ax.set_xlim(0, map_size)
    ax.set_ylim(0, map_size)

    for obstacle in obstacles:
        ax.add_patch(plt.Rectangle((obstacle[0], obstacle[1]), obstacle[2], obstacle[3], color='r'))

    for node in nodes:
        if node.parent:
            ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b')
    
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

if __name__ == "__main__":
    map_size = 10
    obstacles = [(3, 3, 2, 2), (7, 7, 2, 2)]  # Format: (x, y, width, height)
    num_iterations = 2000
    step_size = 0.5
    obstacle_clearance = 0.2  # Minimum distance from obstacles

    nodes = rrt(map_size, obstacles, num_iterations, step_size, obstacle_clearance)
    plot_map(map_size, obstacles, nodes)
