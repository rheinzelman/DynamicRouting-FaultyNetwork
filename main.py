import sys
import numpy as np
import matplotlib.pyplot as plt
import igraph as ig
import random
import math

def gen_weighted_adj_matrix(rows, cols, min_dist, max_dist):
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

def dijkstra(graph, start_node, max_dist):
    # initialization
    unvisited_nodes = ig.VertexSeq(graph).indices
    shortest_path = {}
    previous_nodes = {}
    for node in unvisited_nodes:
        shortest_path[node] = float('inf')
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

def get_shortest_path(previous_nodes, shortest_path, start_node, target_node):
    path = []
    node = target_node

    while node != start_node:
        path.append(node)
        node = previous_nodes[node]

    # Add the start node manually
    path.append(start_node)
    print("We found the following best path with a value of {}.".format(shortest_path[target_node]))
    print(" -> ", path)
    return path

def delete_node_on_path(graph, path):
    node_index = random.choice(path[1:len(path)-1])
    edges = graph.vs[node_index].all_edges()
    graph.delete_edges(edges)
    return node_index

def delete_node_probability(graph, connectivity_matrix, p, cols):
    deleted_nodes = []
    for i in range(len(connectivity_matrix)):
        for j in range(len(connectivity_matrix[0])):
            if(p < connectivity_matrix[cols - 1 - i, j]):
                graph.delete_edges(graph.vs[(cols*i) + j].all_edges())
                deleted_nodes.append((cols*i) + j)
    return deleted_nodes

def node_connectivity(rows, cols):
    connectivity_matrix = np.zeros((rows,cols))
    for row in range(rows):
        for col in range(cols):
            connectivity_matrix[row, col] = np.random.random()
    return connectivity_matrix


# min and max distances in the graph
min_dist = 1
max_dist = 10

# number of iterations where one node along the shortest path is deleted
iterations = 5

# rows and columns must be equal
rows = 10
cols = rows

connectivity_matrix = node_connectivity(rows, cols)

# defining the search nodes
start_node = 0
target_node = 99

# this is kept track of for coloring purposes
disconnected_nodes = []

# generate the adjacency matrix used to create the graph
adj_matrix = gen_weighted_adj_matrix(rows, cols, min_dist, max_dist)

for i in np.arange(0.5, 1, .025):
    # instantiate an igraph graph through our adjacency matrix
    g = ig.Graph.Weighted_Adjacency(adj_matrix, "max")
    disconnected_nodes = delete_node_probability(g, connectivity_matrix, i, cols)
    g.vs["color"] = "black"
    # get the list of prev nodes and the shortest path for each from the dijkstra function
    previous_nodes, shortest_path = dijkstra(g, 0, max_dist)
    # get the shortest node path from our start_node to our target_node
    try:
        if(not previous_nodes == {} and not shortest_path == {}):
            path = get_shortest_path(previous_nodes, shortest_path, start_node=start_node, target_node=target_node)
        else:
            print("No possible path to the target node")
            path = []
    except:
        pass
    # color each node in the path
    for node in path:
        g.vs[node]["color"] = "green"
    for node in disconnected_nodes:
        g.vs[node]["color"] = "red"

    fig, ax = plt.subplots(figsize=(10, 10))
    ig.plot(
        g,
        target=ax,
        layout="grid",
        #layout="fruchterman_reingold",  # print nodes in a circular layout
        edge_label=g.es["weight"],
        vertex_size=.5,
        vertex_shape='rectangle',
        vertex_frame_width=1.0,
        vertex_frame_color="white",
        vertex_label_size=7.0,
        bbox=(1024, 1024),
        margin=10
    )
    plt.show()
    g.vs["color"] = "black"