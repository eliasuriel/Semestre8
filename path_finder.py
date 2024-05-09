import heapq
import matplotlib.pyplot as plt
import random
from segundo_mapa import get_rrt_nodes,imagen_a_matriz,rrt,plot_map,load_image

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = float('inf')
        self.parent = None

    def __lt__(self, other):
        return self.cost < other.cost

class PathPlanner:
    def __init__(self, nodes, start_node, goal_node):
        self.nodes = nodes
        self.start_node = start_node
        self.goal_node = goal_node
        self.start_node.cost = 0

    def euclidean_distance(self, node1, node2):
        return ((node1.x - node2.x)**2 + (node1.y - node2.y)**2)**0.5

    def dijkstra(self):
        pq = [(self.start_node.cost, self.start_node)]
        visited = set()

        while pq:
            current_cost, current_node = heapq.heappop(pq)

            if current_node == self.goal_node:
                path = []
                node = current_node
                while node:
                    path.append(node)
                    node = node.parent
                return path[::-1]

            visited.add(current_node)

            for neighbor in self.nodes:
                if neighbor not in visited:
                    new_cost = current_cost + self.euclidean_distance(current_node, neighbor)
                    if new_cost < neighbor.cost:
                        neighbor.cost = new_cost
                        neighbor.parent = current_node
                        heapq.heappush(pq, (new_cost, neighbor))

        return []
    
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

# Después de obtener la lista de nodos con get_rrt_nodes()
node_list = get_rrt_nodes(map_size, obstacles, num_iterations, step_size, obstacle_clearance)
#print(len(node_list))
start_node_x, start_node_y = 146, 61  # Coordenadas del nodo de inicio
start_node = Node(start_node_x, start_node_y)

# Asignar los nodos de inicio y destino
#start_node = node_list[0]  # Asumiendo que el primer nodo es el nodo de inicio
numero_random = random.randint(0, len(node_list))
goal_node = node_list[numero_random]  # Definir las coordenadas del nodo de destino

path_planner = PathPlanner(node_list, start_node, goal_node)
shortest_path = path_planner.dijkstra()
print(goal_node)
plot_map(map_size, obstacles, node_list, shortest_path)
if shortest_path:
    print("Ruta mas corta encontrada:")
    for node in shortest_path:
        print(f"({node.x}, {node.y})")
else:
    print("No se encontro una ruta valida.")
