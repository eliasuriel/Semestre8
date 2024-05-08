import heapq
import numpy as np

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = float('inf')
        self.parent = None

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

# Ejemplo de uso
nodes = [Node(0, 0), Node(1, 2), Node(3, 1)]  # Lista de nodos de ejemplo
algorithm = DijkstraAlgorithm(nodes)
start_node = nodes[0]  # Nodo de inicio
algorithm.dijkstra(start_node)
