import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def load_image(filename):
    image = Image.open(filename).convert("L")  # Convert image to grayscale
    map_size = max(image.size)  # Assuming the image is square
    obstacle_pixels = np.array(image)  # Obtaining pixel values as numpy array
    obstacles = []

    # Extract obstacle information from image
    for i in range(image.size[0]):  # Iterate over image width
        for j in range(image.size[1]):  # Iterate over image height
            if obstacle_pixels[i, j] > 127:  # Black pixels considered as obstacles
                obstacles.append((i, j))

    return map_size, obstacles

def generate_random_point(map_size, obstacles, min_distance=1):
    while True:
        x = np.random.uniform(0, map_size)
        y = np.random.uniform(0, map_size)
        valid = True
        for obstacle in obstacles:
            if np.sqrt((x - obstacle[0])**2 + (y - obstacle[1])**2) < min_distance:
                valid = False
                break
        if valid:
            return x, y

def is_valid_point(point, obstacles, obstacle_clearance):
    for obstacle in obstacles:
        x, y = obstacle
        if np.sqrt((point[0] - x)**2 + (point[1] - y)**2) < obstacle_clearance:
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

    for obstacle in obstacles:
        if np.sqrt((new_x - obstacle[0])**2 + (new_y - obstacle[1])**2) < obstacle_clearance:
            return None

    return new_point

def rrt(map_size, obstacles, num_iterations, step_size, obstacle_clearance):
    #start = Node(0, 0)
    start = Node(180, 68)
    nodes = [start]

    for _ in range(num_iterations):
        random_point = generate_random_point(map_size, obstacles, min_distance=obstacle_clearance)
        nearest = nearest_node(nodes, random_point)
        new_point = new_point_within_bounds(nearest, random_point, step_size, obstacles, obstacle_clearance)
        if new_point is not None:
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
        ax.add_patch(plt.Rectangle((obstacle[0], obstacle[1]), 1, 1, color='k'))

    for node in nodes:
        if node.parent:
            ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b')
    
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

def imagen_a_matriz(imagen_path):
    imagen_or = Image.open(imagen_path)
    imagen = imagen_or.convert('L')
    ancho, alto = imagen.size
    matriz = []
    for y in range(alto):
        fila = []
        for x in range(ancho):
            valor_pixel = imagen.getpixel((x, y))  # Obtener el valor del píxel en la posición (x, y)
            if valor_pixel == 0:  # Pixel negro
                fila.append(0)  # Considerar como obstáculo
            else:  # Otros píxeles (grises o blancos)
                fila.append(1)  # Considerar como espacio libre
        matriz.append(fila)
    
    return matriz

if __name__ == "__main__":
    map_filename = "mapa.png"  # Ruta de tu archivo de imagen
    map_matrix = imagen_a_matriz(map_filename)
    map_size = len(map_matrix[0])  # Tamaño de la matriz (se asume que es cuadrada)
    obstacles = []

    for y in range(len(map_matrix)):
        for x in range(len(map_matrix[y])):
            if map_matrix[y][x] == 1:  # Considerar los píxeles negros como obstáculos
                obstacles.append((x, y))

    num_iterations = 1000
    step_size = 0.5
    obstacle_clearance = 0.8

    nodes = rrt(map_size, obstacles, num_iterations, step_size, obstacle_clearance)
    plot_map(map_size, obstacles, nodes)
