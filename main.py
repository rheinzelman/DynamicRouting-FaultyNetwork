import sys
import numpy as np
import matplotlib.pyplot as plt
import igraph as ig
import random

def gen_weighted_adj_matrix(rows, cols):
    n = rows * cols
    M = np.zeros((n, n))
    for r in list(range(rows)):
        for c in list(range(cols)):
            i = r * cols + c
            # Two inner diagonals
            if c > 0: M[i - 1, i] = M[i, i - 1] = random.randint(1,10)
            # Two outer diagonals
            if r > 0: M[i - cols, i] = M[i, i - cols] = random.randint(1,10)
    return M

adj_matrix = gen_weighted_adj_matrix(5, 5)

g = ig.Graph.Weighted_Adjacency(adj_matrix, "min")

def print_result(previous_nodes, shortest_path, start_node, target_node):
    path = []
    node = target_node

    while node != start_node:
        path.append(node)
        node = previous_nodes[node]

    # Add the start node manually
    path.append(start_node)
    print("We found the following best path with a value of {}.".format(shortest_path[target_node]))
    print(" -> ", path)

def dijkstra(graph, start_node):
    unvisited_nodes = ig.VertexSeq(graph).indices
    shortest_path = {}
    previous_nodes = {}
    max_value = sys.maxsize
    for node in unvisited_nodes:
        shortest_path[node] = max_value
    shortest_path[start_node] = 0

    while unvisited_nodes:
        current_min_node = None
        for node in unvisited_nodes:
            if current_min_node == None:
                current_min_node = node
            elif shortest_path[node] < shortest_path[current_min_node]:
                current_min_node = node

        neighbors = graph.neighbors(current_min_node)
        for neighbor in neighbors:
            # getting index of 0 because graph.es.select returns a list even if it is size zero
            tentative_value = shortest_path[current_min_node] + graph.es.select(_source=current_min_node, _target=neighbor)['weight'][0]
            if tentative_value < shortest_path[neighbor]:
                shortest_path[neighbor] = tentative_value
                previous_nodes[neighbor] = current_min_node

        unvisited_nodes.remove(current_min_node)

    return previous_nodes, shortest_path

previous_nodes, shortest_path = dijkstra(g, 0)
print("previous nodes")
print(previous_nodes.keys())
print(previous_nodes.values())
print("shortest path")
print(shortest_path.keys())
print(shortest_path.values())
print_result(previous_nodes, shortest_path, start_node=0, target_node=19)


fig, ax = plt.subplots(figsize=(10, 10))
ig.plot(
    g,
    target=ax,
    layout="grid",
    #layout="fruchterman_reingold",  # print nodes in a circular layout
    edge_label=g.es["weight"],
    vertex_size=.25,
    vertex_shape='rectangle',
    vertex_frame_width=1.0,
    vertex_frame_color="white",
    vertex_label_size=7.0,
    bbox=(1024, 1024),
    margin=10
)
plt.savefig("fig")
plt.show()