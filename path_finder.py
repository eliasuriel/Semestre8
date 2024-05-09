import heapq
import numpy as np
import matplotlib.pyplot as plt
from segundo_mapa import get_rrt_nodes,imagen_a_matriz,rrt,plot_map

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = float('inf')
        self.parent = None

    def __lt__(self, other):
        return self.cost < other.cost

class DijkstraAlgorithm:
    def __init__(self, nodes):
        self.nodes = nodes

    def dijkstra(self, start_node):
        start_node.cost = 0
        visited = set()
        pq = [(start_node.cost, start_node)]

        while pq:
            current_cost, current_node = heapq.heappop(pq)

            if current_node in visited:
                continue

            visited.add(current_node)

            for neighbor in self.nodes:
                if neighbor == current_node.parent:
                    continue

                dist_to_neighbor = np.sqrt((current_node.x - neighbor.x)**2 + (current_node.y - neighbor.y)**2)
                new_cost = current_cost + dist_to_neighbor

                if new_cost < neighbor.cost:
                    neighbor.cost = new_cost
                    neighbor.parent = current_node
                    heapq.heappush(pq, (new_cost, neighbor))

map_filename = "mapa2.png"  # Ruta de tu archivo de imagen
map_matrix = imagen_a_matriz(map_filename)
map_size = len(map_matrix[0])  # Tamaño de la matriz (se asume que es cuadrada)
obstacles = []

for y in range(len(map_matrix)):
        for x in range(len(map_matrix[y])):
            if map_matrix[y][x] == 1:  # Considerar los píxeles negros como obstáculos
                obstacles.append((x, y))

num_iterations = 300
step_size = 0.5
obstacle_clearance = 0.8

nodes = get_rrt_nodes(map_size, obstacles, num_iterations, step_size, obstacle_clearance)

# Ejecutar algoritmo de Dijkstra
algorithm = DijkstraAlgorithm(nodes)
start_node = nodes[0]  # Nodo de inicio
algorithm.dijkstra(start_node)



#plot_map(map_size, obstacles, algorithm.dijkstra(start_node))


# Mostrar los resultados
for node in nodes:
    path_cost = node.cost
    path = [node]
    current_node = node
    while current_node.parent:
        path.append(current_node.parent)
        current_node = current_node.parent
    path.reverse()
    print(f"Nodo ({node.x}, {node.y}): Costo del camino mas corto = {path_cost}, Ruta = {[n.x for n in path]}")

# Visualización de nodos y conexiones
plt.figure(figsize=(6, 6))
for node in nodes:
    plt.scatter(node.x, node.y, color='blue')
    #plt.text(node.x + 0.05, node.y + 0.05, f'({node.x}, {node.y})')
    if node.parent:
        plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color='gray')
plt.title('Grafo con nodos y conexiones')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.show()
