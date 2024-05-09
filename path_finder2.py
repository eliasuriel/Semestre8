import heapq
import matplotlib.pyplot as plt
import random
from rrt import RRT, Node

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

def plot_path(nodes, path):
    plt.figure(figsize=(8, 8))
    plt.imshow(rrt._RRT__map, cmap='Greys', origin='lower')

    # Plotear nodos RRT
    for node in nodes:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color='blue')

    # Plotear ruta más corta
    path_x = [node.x for node in path]
    path_y = [node.y for node in path]
    plt.plot(path_x, path_y, color='red', linewidth=2)

    plt.show()

if __name__ == "__main__":
    rrt = RRT()
    node_list = rrt.get_rrt_nodes()

    start_node_x, start_node_y = 146, 61  # Coordenadas del nodo de inicio
    start_node = Node(start_node_x, start_node_y)

    # Asignar el nodo de destino de forma aleatoria
    numero_random = random.randint(0, len(node_list) - 1)
    goal_node = node_list[numero_random]

    path_planner = PathPlanner(node_list, start_node, goal_node)
    shortest_path = path_planner.dijkstra()

    if shortest_path:
        print("Ruta mas corta encontrada:")
        for node in shortest_path:
            print(f"({node.x}, {node.y})")

        plot_path(node_list, shortest_path)
    else:
        print("No se encontro una ruta valida.")